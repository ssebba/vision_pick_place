#ifndef PP_TASK__TASK_MANAGER_HPP_
#define PP_TASK__TASK_MANAGER_HPP_

#include <memory>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

#include "pp_task/task_context.hpp"
#include "pp_task/task_manager_parameters.hpp"

namespace pp_task
{

/**
 * @brief Lifecycle component running the pick-and-place behavior tree.
 *
 * configure: MoveIt interfaces (on an internal node spun by a dedicated executor), grasp
 *            attacher, detection subscription, BT plugins and tree;
 * activate:  ticks the tree in a worker thread; at the end publishes a summary on
 *            `~/result`, appends the per-object records to `results_file` and, if
 *            requested, shuts the process down (one benchmark trial per launch).
 */
class TaskManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit TaskManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~TaskManager() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
  bool waitForControllers();
  void run();
  void writeResults(double total_time) const;
  void stopWorker();

  std::shared_ptr<task_manager::ParamListener> param_listener_;
  task_manager::Params params_;

  rclcpp::Node::SharedPtr moveit_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;

  std::shared_ptr<TaskContext> context_;
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detections_sub_;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::thread worker_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr result_pub_;
};

}  // namespace pp_task

#endif  // PP_TASK__TASK_MANAGER_HPP_
