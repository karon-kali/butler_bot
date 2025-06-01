#include <behaviortree_cpp_v3/bt_factory.h>
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

int main(int argc, char **argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create a ROS 2 node
  auto node = rclcpp::Node::make_shared("bt_executor");

  // Create a behavior tree factory
  BT::BehaviorTreeFactory factory;

  // Register custom nodes from plugins
  factory.registerFromPlugin("libtopic_sub_plugin.so");
  factory.registerFromPlugin("libsimple_topic_sub_plugin.so");
  factory.registerFromPlugin("libdisplay_plugin.so");
  
  // Register the three new custom nodes
  factory.registerFromPlugin("liborder_queue_manager_plugin.so");
  factory.registerFromPlugin("libfor_each_plugin.so");
  factory.registerFromPlugin("libremove_from_queue_plugin.so");
  
  // Register NavigationPoseManager plugin (seems to be missing)
  // factory.registerFromPlugin("libnavigation_pose_manager_plugin.so");
  
  // Register Nav2 behavior tree nodes
  factory.registerFromPlugin("libnav2_navigate_to_pose_action_bt_node.so");
  factory.registerFromPlugin("libnav2_wait_action_bt_node.so");
  factory.registerFromPlugin("libnav2_recovery_node_bt_node.so");
  
  // Note: Control flow nodes (Fallback, Sequence, ReactiveSequence, etc.) 
  // are built into BehaviorTree.CPV3 and don't need to be registered as plugins

  // Create a blackboard and set the node before creating the tree
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("bt_loop_duration", std::chrono::milliseconds(10));
  blackboard->set("server_timeout", std::chrono::milliseconds(20));
  blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(1000));
  
  // Set default poses for the butler robot
  blackboard->set("home_pose", "{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}");
  blackboard->set("kitchen_pose", "{position: {x: 0.0, y: 2.64, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.7, w: 0.7}}");

  // Get the package share directory and construct the full path to the XML file
  std::string package_share_directory;
  try {
    package_share_directory = ament_index_cpp::get_package_share_directory("butler_bot");
  } catch (const std::exception& e) {
    std::cerr << "Error getting package share directory: " << e.what() << std::endl;
    rclcpp::shutdown();
    return -1;
  }
  
  // Change to butler behavior tree file
  std::string tree_path = package_share_directory + "/behaviour_trees/butler_tree.xml";
  
  try {
    auto tree = factory.createTreeFromFile(tree_path, blackboard);
    
    std::cout << "Butler behavior tree loaded successfully from: " << tree_path << std::endl;
    std::cout << "Starting butler robot execution..." << std::endl;
    
    // Execute the tree
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING && rclcpp::ok()) {
      status = tree.tickRoot();
      
      // Process ROS callbacks
      rclcpp::spin_some(node);
      
      // Sleep to avoid busy waiting
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Print final status
    if (status == BT::NodeStatus::SUCCESS) {
      std::cout << "Butler behavior tree completed successfully!" << std::endl;
    } else if (status == BT::NodeStatus::FAILURE) {
      std::cout << "Butler behavior tree failed!" << std::endl;
    } else {
      std::cout << "Butler behavior tree execution interrupted." << std::endl;
    }
    
  } catch (const std::exception& e) {
    std::cerr << "Error loading or executing behavior tree: " << e.what() << std::endl;
    std::cerr << "Make sure the file exists at: " << tree_path << std::endl;
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}