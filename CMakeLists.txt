cmake_minimum_required(VERSION 3.16)
project(master3_bt3)


set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(dependencies
    rclcpp
    rclcpp_lifecycle
    rclcpp_action
    behaviortree_cpp_v3
    action_msgs
    lifecycle_msgs
    geometry_msgs
    nav2_msgs
    ament_index_cpp
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(br2_recharge_bt_node SHARED src/br2_bt_patrolling/Recharge.cpp)
add_library(br2_patrol_bt_node SHARED src/br2_bt_patrolling/Patrol.cpp)
add_library(br2_saysomething_bt_node SHARED src/br2_bt_patrolling/SaySomething.cpp)
add_library(br2_move_bt_node SHARED src/br2_bt_patrolling/Move.cpp)
add_library(br2_get_waypoint_bt_node SHARED src/br2_bt_patrolling/GetWaypoint.cpp)
add_library(br2_battery_checker_bt_node SHARED src/br2_bt_patrolling/BatteryChecker.cpp)
add_library(br2_track_objects_bt_node SHARED src/br2_bt_patrolling/TrackObjects.cpp)
list(APPEND plugin_libs
  br2_recharge_bt_node
  br2_patrol_bt_node
  br2_move_bt_node
  br2_get_waypoint_bt_node
  br2_battery_checker_bt_node
  br2_track_objects_bt_node
  br2_saysomething_bt_node
)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(behavior_main src/behavior_main.cpp)
ament_target_dependencies(behavior_main ${dependencies})
target_link_libraries(behavior_main ${ZMQ_LIBRARIES})


install(TARGETS
  ${plugin_libs}
  behavior_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

######################################################
# INSTALL Behavior.xml's, ROS config and launch files

install(DIRECTORY behavior_trees launch
  DESTINATION share/${PROJECT_NAME}
)

######################################################
# INSTALL plugins for other packages to load

install(TARGETS
  # sleep_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
  )


# ament_export_dependencies(behaviortree_ros2 btcpp_ros2_interfaces)

ament_package()
