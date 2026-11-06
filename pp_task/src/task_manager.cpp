#include "pp_task/task_manager.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_path.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace pp_task
{

TaskManager::TaskManager(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("task_manager", options)
{
}

TaskManager::~TaskManager()
{
  stopWorker();
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

TaskManager::CallbackReturn TaskManager::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  param_listener_ = std::make_shared<task_manager::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // MoveGroupInterface needs a plain rclcpp::Node: create one with the same parameters
  // (use_sim_time, robot_description_kinematics, ...), spun by its own executor.
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  std::vector<rclcpp::Parameter> overrides;
  for (const auto & [name, value] : get_node_parameters_interface()->get_parameter_overrides()) {
    overrides.emplace_back(name, value);
  }
  options.parameter_overrides(overrides);
  moveit_node_ = std::make_shared<rclcpp::Node>(std::string(get_name()) + "_moveit", options);
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(moveit_node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  context_ = std::make_shared<TaskContext>();
  context_->node = moveit_node_;
  try {
    context_->arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      moveit_node_, params_.arm_group);
    context_->gripper = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      moveit_node_, params_.gripper_group);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "MoveIt is not available: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  context_->execute_client = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(
    moveit_node_, "execute_trajectory");
  context_->arm->setPlanningTime(5.0);
  context_->arm->setNumPlanningAttempts(3);
  context_->attacher = std::make_shared<SimGraspAttacher>(moveit_node_, "object_", 0.1);
  context_->motion_velocity_scaling = params_.motion_velocity_scaling;
  context_->grasp_params.preferred_yaw = params_.preferred_grasp_yaw;
  context_->grasp_params.approach_distance = params_.approach_distance;
  context_->place_origin =
    Eigen::Vector3d(params_.place_origin[0], params_.place_origin[1], params_.place_origin[2]);
  context_->place_step =
    Eigen::Vector3d(params_.place_step[0], params_.place_step[1], params_.place_step[2]);

  detections_sub_ = moveit_node_->create_subscription<vision_msgs::msg::Detection3DArray>(
    params_.detections_topic, 10,
    [context = context_](vision_msgs::msg::Detection3DArray::SharedPtr msg) {
      std::scoped_lock lock(context->detections_mutex);
      context->detections = std::move(msg);
    });

  try {
    std::filesystem::path prefix;
    ament_index_cpp::get_package_prefix("pp_task", prefix);
    for (const auto & library : params_.plugin_libraries) {
      factory_.registerFromPlugin((prefix / "lib" / ("lib" + library + ".so")).string());
    }
    const std::string tree_file =
      params_.tree_file.empty()
        ? (ament_index_cpp::get_package_share_path("pp_task") / "behavior_trees" / "pick_place.xml")
            .string()
        : params_.tree_file;
    auto blackboard = BT::Blackboard::create();
    blackboard->set("context", context_);
    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(tree_file, blackboard));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Cannot build the behavior tree: %s", e.what());
    return CallbackReturn::FAILURE;
  }
  result_pub_ =
    create_publisher<std_msgs::msg::String>("~/result", rclcpp::QoS(1).transient_local());
  RCLCPP_INFO(get_logger(), "Configured");
  return CallbackReturn::SUCCESS;
}

TaskManager::CallbackReturn TaskManager::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  result_pub_->on_activate();
  context_->stop_requested = false;
  worker_ = std::thread([this]() { run(); });
  return CallbackReturn::SUCCESS;
}

bool TaskManager::waitForControllers()
{
  // The simulation starts the controllers asynchronously: MoveIt would reject the first
  // trajectories if the task started before them.
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(params_.wait_timeout);
  for (const auto & name : params_.wait_for_actions) {
    const auto client = rclcpp_action::create_client<FollowJointTrajectory>(moveit_node_, name);
    while (!context_->stopRequested() && !client->wait_for_action_server(std::chrono::seconds(1))) {
      if (std::chrono::steady_clock::now() > deadline) {
        RCLCPP_ERROR(get_logger(), "Action server %s not available", name.c_str());
        return false;
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for the action server %s", name.c_str());
    }
  }
  return !context_->stopRequested();
}

void TaskManager::run()
{
  const bool ready = waitForControllers();
  std::this_thread::sleep_for(std::chrono::duration<double>(params_.start_delay));
  RCLCPP_INFO(get_logger(), "Starting the pick-and-place task");
  // Task and cycle times in simulated time: independent of the real-time factor.
  const rclcpp::Time start = context_->node->now();
  BT::NodeStatus status = ready ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  try {
    while (!context_->stopRequested() && status == BT::NodeStatus::RUNNING) {
      status = tree_->tickOnce();
      tree_->sleep(std::chrono::milliseconds(10));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Behavior tree error: %s", e.what());
    status = BT::NodeStatus::FAILURE;
  }
  context_->closeRecord(context_->node->now());
  const double total = (context_->node->now() - start).seconds();

  int placed = 0;
  for (const auto & r : context_->records) {
    placed += r.placed ? 1 : 0;
  }
  std::ostringstream summary;
  summary << "status=" << BT::toStr(status) << " objects=" << context_->records.size()
          << " placed=" << placed << " time=" << total;
  if (context_->stopRequested()) {
    // Stopped from outside (Ctrl+C, deactivation, trial timeout): not a result.
    RCLCPP_INFO(get_logger(), "Task interrupted: %s", summary.str().c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Task finished: %s", summary.str().c_str());
  std_msgs::msg::String msg;
  msg.data = summary.str();
  result_pub_->publish(msg);
  writeResults(total);

  if (params_.shutdown_when_done) {
    rclcpp::shutdown();
  }
}

void TaskManager::writeResults(double total_time) const
{
  if (params_.results_file.empty()) {
    return;
  }
  const std::filesystem::path file(params_.results_file);
  std::error_code ec;
  std::filesystem::create_directories(file.parent_path(), ec);
  const bool new_file = !std::filesystem::exists(file);
  std::ofstream out(file, std::ios::app);
  if (new_file) {
    out << "run_id,object,model,shape,detection_error_m,grasped,placed,place_error_m,cycle_time_s,"
           "planning_time_s,total_time_s\n";
  }
  for (std::size_t i = 0; i < context_->records.size(); ++i) {
    const auto & r = context_->records[i];
    out << params_.run_id << ',' << i << ',' << r.model << ',' << r.shape << ','
        << r.detection_error << ',' << (r.grasped ? 1 : 0) << ',' << (r.placed ? 1 : 0) << ','
        << r.place_error << ',' << r.cycle_time << ',' << r.planning_time << ',' << total_time
        << '\n';
  }
}

void TaskManager::stopWorker()
{
  if (context_) {
    context_->stop_requested = true;
  }
  if (worker_.joinable()) {
    worker_.join();
  }
}

TaskManager::CallbackReturn TaskManager::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  stopWorker();
  result_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}

TaskManager::CallbackReturn TaskManager::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  tree_.reset();
  detections_sub_.reset();
  context_.reset();
  result_pub_.reset();
  return CallbackReturn::SUCCESS;
}

}  // namespace pp_task

RCLCPP_COMPONENTS_REGISTER_NODE(pp_task::TaskManager)
