cmake_minimum_required(VERSION 3.4)
project(cloth_manipulation)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  hardware_interface
  joint_limits_interface
  message_generation
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_ros
  pluginlib
  realtime_tools
  roscpp
  rospy
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  trajectory_msgs
  urdf
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# merge Franka + INCLUDE_DIRS in topological order
list_insert_in_workspace_order(catkin_INCLUDE_DIRS ${Franka_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add_message_files(FILES
#   JointTorqueComparison.msg
# )

# generate_messages()

# generate_dynamic_reconfigure_options(
#   cfg/compliance_param.cfg
#   cfg/desired_mass_param.cfg
#   cfg/dual_arm_compliance_param.cfg
#   cfg/teleop_param.cfg
#   cfg/teleop_gripper_param.cfg
# )

catkin_package(
# INCLUDE_DIRS include
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    hardware_interface
    joint_limits_interface
    message_runtime
    moveit_core
    moveit_ros_planning_interface
    pluginlib
    realtime_tools
    roscpp
    trajectory_msgs
    urdf
    visualization_msgs
  DEPENDS Franka
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_executable(joint_space_planner src/joint_space_planner.cpp)
target_link_libraries(joint_space_planner ${catkin_LIBRARIES})

add_executable(full_grasping_planner src/full_grasping_planner.cpp)
target_link_libraries(full_grasping_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(color_mask src/color_mask.cpp)
target_link_libraries(color_mask ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS joint_space_planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS full_grasping_planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS color_mask
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

