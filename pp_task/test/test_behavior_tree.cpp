// The real pick-and-place tree is built from the XML with the nodes registered by the plugin:
// catches typos in node names and ports without MoveIt or a simulator.

#include <memory>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include "behaviortree_cpp/bt_factory.h"

#include "pp_task/task_context.hpp"

TEST(BehaviorTree, PluginRegistersAllTheNodes)
{
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(BT_PLUGIN_PATH);
  std::set<std::string> registered;
  for (const auto & [id, manifest] : factory.manifests()) {
    registered.insert(id);
  }
  for (const char * id :
       {"MoveToNamed", "MoveToPose", "MoveLinear", "SetGripper", "DetectObjects", "SelectTarget",
        "PlanGrasp", "AttachObject", "DetachObject", "VerifyLift", "NextPlacePose", "VerifyPlace",
        "MarkFailed"}) {
    EXPECT_TRUE(registered.count(id)) << id;
  }
}

TEST(BehaviorTree, PickPlaceTreeLoads)
{
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(BT_PLUGIN_PATH);
  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", std::make_shared<pp_task::TaskContext>());
  BT::Tree tree;
  ASSERT_NO_THROW(tree = factory.createTreeFromFile(TREE_FILE, blackboard));
  std::size_t nodes = 0;
  tree.applyVisitor([&nodes](const BT::TreeNode *) { ++nodes; });
  EXPECT_GT(nodes, 20u);
}
