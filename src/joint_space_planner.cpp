#include <ros/ros.h>
#include <fstream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_pose_planning_node");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Load Controller Configuration
    node_handle.setParam("/move_group/controller_list", "config/simple_moveit_controllers.yaml");

    ROS_INFO("Reached here! (1)");

    // Planning group
    static const std::string PLANNING_GROUP = "panda_arm";
    ROS_INFO("Reached here! (1.1)");
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    ROS_INFO("Reached here! (1.2)");

    // Set planner
    move_group.setPlannerId("position_joint_trajectory_controller");

    ROS_INFO("Reached here! (1.3)");
    // Set the joint target values
    std::vector<double> joint_target = {0.3481298279961115, -0.4755552899338467, -0.32365981495171264, -2.714270300580744, -0.5073398909303876, 2.7890981041325706, 1.4532492337442104};

    ROS_INFO("Reached here! (2)");

    // Set the joint target
    move_group.setJointValueTarget(joint_target);
    move_group.setNumPlanningAttempts(5);
    move_group.setPlanningTime(5.0);

    ROS_INFO("Reached here! (3)");

    // Call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = static_cast<bool>(move_group.plan(my_plan));

    ROS_INFO("Reached here! (4)");

    ros::Duration(15.0).sleep();

    if (success)
    {
        ROS_INFO("Planning successful. Executing the plan.");

        // Execute the plan
        move_group.execute(my_plan);
    }
    else
    {
        ROS_ERROR("Planning failed");
    }

    ros::Duration(15.0).sleep();

    ros::shutdown();
    return 0;
}