cmake_minimum_required(VERSION 3.8)
project(butler_bot)

if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)

# Include directories
include_directories(include)

########################################################################################
# Add the library for the 'topic_sub_node' plugin
add_library(topic_sub_plugin SHARED
  plugins/topic_sub_node.cpp
)

# Link dependencies for 'topic_sub_node' plugin
ament_target_dependencies(topic_sub_plugin
  behaviortree_cpp_v3
  rclcpp
  std_msgs
  nav2_behavior_tree
  nav2_msgs
)

# Export the 'topic_sub_node' plugin
target_compile_definitions(topic_sub_plugin PRIVATE "BT_PLUGIN_EXPORT")

########################################################################################
# Add the library for the 'simple_topic_sub_node' plugin
add_library(simple_topic_sub_plugin SHARED
  plugins/simple_topic_sub_node.cpp
)

# Link dependencies for 'topic_sub_node' plugin
ament_target_dependencies(simple_topic_sub_plugin
  behaviortree_cpp_v3
  rclcpp
  std_msgs
  nav2_behavior_tree
  nav2_msgs
)

# Export the 'topic_sub_node' plugin
target_compile_definitions(simple_topic_sub_plugin PRIVATE "BT_PLUGIN_EXPORT")

########################################################################################

# Add the library for the 'display_bt_node' plugin
add_library(display_plugin SHARED
  plugins/display_bt_node.cpp
)

# Link dependencies for 'display_plugin' plugin
ament_target_dependencies(display_plugin
  behaviortree_cpp_v3
  rclcpp
)

# Export the 'display_plugin' plugin
target_compile_definitions(display_plugin PRIVATE "BT_PLUGIN_EXPORT")

########################################################################################
########################################################################################
# Add the library for the 'order_queue_manager' plugin
add_library(order_queue_manager_plugin SHARED
  plugins/OrderQueueManager.cpp
)

# Link dependencies for 'order_queue_manager' plugin
ament_target_dependencies(order_queue_manager_plugin
  behaviortree_cpp_v3
  rclcpp
  std_msgs
)

# Export the 'order_queue_manager' plugin
target_compile_definitions(order_queue_manager_plugin PRIVATE "BT_PLUGIN_EXPORT")

########################################################################################
# Add the library for the 'for_each' plugin
add_library(for_each_plugin SHARED
  plugins/ForEach.cpp
)

# Link dependencies for 'for_each' plugin
ament_target_dependencies(for_each_plugin
  behaviortree_cpp_v3
  rclcpp
  std_msgs
)

# Export the 'for_each' plugin
target_compile_definitions(for_each_plugin PRIVATE "BT_PLUGIN_EXPORT")

########################################################################################
# Add the library for the 'remove_from_queue' plugin
add_library(remove_from_queue_plugin SHARED
  plugins/RemoveFromQueue.cpp
)

# Link dependencies for 'remove_from_queue' plugin
ament_target_dependencies(remove_from_queue_plugin
  behaviortree_cpp_v3
  rclcpp
  std_msgs
)

# Export the 'remove_from_queue' plugin
target_compile_definitions(remove_from_queue_plugin PRIVATE "BT_PLUGIN_EXPORT")

########################################################################################

# Add the executable for bt_exe.cpp (executing the behavior tree)
add_executable(bt_executor src/bt_exe.cpp)

# Link dependencies for the main executable
ament_target_dependencies(bt_executor
  behaviortree_cpp_v3
  rclcpp
  std_msgs
  ament_index_cpp
)

# Link the plugin libraries to the executable
target_link_libraries(bt_executor
  topic_sub_plugin
  simple_topic_sub_plugin
  display_plugin
  order_queue_manager_plugin
  for_each_plugin
  remove_from_queue_plugin
)

# Install targets
install(TARGETS
  bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  topic_sub_plugin
  simple_topic_sub_plugin
  display_plugin
  order_queue_manager_plugin
  for_each_plugin
  remove_from_queue_plugin
  LIBRARY DESTINATION lib
)

# Install launch scripts, params, world, map, and src directories
install(
  DIRECTORY
    launch
    params
    world
    map
    src
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts and make them executable
install(
  PROGRAMS
    src/web.py
  DESTINATION share/${PROJECT_NAME}/src
)

# Install butler_robot_node.py as an executable in lib directory
install(
  PROGRAMS
    src/butler_robot_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install plugin source files (optional, for reference)
install(DIRECTORY plugins/
        DESTINATION lib/${PROJECT_NAME}/plugins)

# Install behavior tree files if you have them
install(DIRECTORY behaviour_trees/
        DESTINATION share/${PROJECT_NAME}/behaviour_trees)

ament_package()