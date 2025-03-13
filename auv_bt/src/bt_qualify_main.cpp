#include <memory>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("qualifying_node");

  node->declare_parameter("on_device", false);
  auto on_device = node->get_parameter("on_device").as_bool();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("auv_elapsed_bt_node"));
  factory.registerFromPlugin(loader.getOSName("auv_forward_bt_node"));
  factory.registerFromPlugin(loader.getOSName("auv_turn_bt_node"));
  factory.registerFromPlugin(loader.getOSName("auv_descend_bt_node"));
  factory.registerFromPlugin(loader.getOSName("auv_set_led_color_bt_node"));
  factory.registerFromPlugin(loader.getOSName("auv_set_mode_bt_node"));
  factory.registerFromPlugin(loader.getOSName("auv_is_armed_bt_node"));
  factory.registerFromPlugin(
      loader.getOSName("auv_is_depth_calibrated_bt_node"));
  factory.registerFromPlugin(loader.getOSName("auv_calibrate_depth_bt_node"));
  factory.registerFromPlugin(loader.getOSName("auv_gate_bt_node"));
  factory.registerFromPlugin(loader.getOSName("auv_follow_gate_bt_node"));
  std::string pkgpath = ament_index_cpp::get_package_share_directory("auv_bt");
  std::string xml_file;

  if (on_device) {
    xml_file = pkgpath + "/trees/elapsed_descend_test.xml";
  } else {
    xml_file =
        pkgpath + "/behavior_tree_xml/gate_detection_forward_follow_test.xml";
    // xml_file = pkgpath + "/behavior_tree_xml/gate_detection_test.xml";
  }
  RCLCPP_INFO(node->get_logger(), "reading behavior tree from file at : %s",
              xml_file.c_str());

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
