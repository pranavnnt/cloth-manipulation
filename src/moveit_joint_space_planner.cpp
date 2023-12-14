#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "move_group_interface_example");
    ros::NodeHandle node_handle;

    // Create a MoveGroupInterface for the Panda robot arm
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");

    // Set the planner to use for joint space planning
    move_group.setPlannerId("RRTConnectkConfigDefault");

    // Set a joint space target
    std::vector<double> joint_values = {0.3481298, -0.475555, -0.3236598, -2.714270, -0.5073398, 2.789098, 1.453249};
    move_group.setJointValueTarget(joint_values);

    // Plan the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        ROS_INFO("Planning successfully completed. Executing the trajectory.");
        // Execute the planned trajectory
        move_group.execute(my_plan);
    }
    else
    {
        ROS_ERROR("Planning failed!");
    }

    ros::shutdown();
    return 0;
}