#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/PointCloud2.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

bool detect_grasping_point;

tf2_ros::Buffer tf_buffer;
tf2_ros::TransformListener tf_listener(tf_buffer);

//pre-grasp motion
bool preGraspMovement()
{
    // Load Controller Configuration : loaded in main function
    //nh.setParam("/move_group/controller_list", "config/simple_moveit_controllers.yaml");

    // Planning group
    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    
    // Set planner
    move_group.setPlannerId("position_joint_trajectory_controller");

    // Set the joint target values
    std::vector<double> pre_grasp_target = {0.3481298279961115, -0.4755552899338467, -0.32365981495171264, -2.714270300580744,
                                           -0.5073398909303876, 2.7890981041325706, 1.4532492337442104};

    // Set the joint target
    move_group.setJointValueTarget(pre_grasp_target);
    move_group.setNumPlanningAttempts(5);
    move_group.setPlanningTime(5.0);

    // Call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
    bool success = static_cast<bool>(move_group.plan(pre_grasp_plan));

    ros::Duration(5.0).sleep();

    if (success)
    {
        ROS_INFO("Pre-grasp plan successful. Executing the plan.");

        // Execute the plan
        move_group.execute(pre_grasp_plan);
        return true;
    }
    else
    {
        ROS_ERROR("Planning failed");
        return false;
    }
}

//point cloud sub callback

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg)
{
    if(!detect_grasping_point) return;
        
    //convert to pcl type
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    //transform point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl_ros::transformPointCloud("panda_link0", *pcl_cloud, *transformed_cloud, tf_buffer);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(transformed_cloud);
    vox.setLeafSize(0.005f, 0.005f, 0.005f);
    vox.filter (*filtered_cloud);

    //publish new point cloud
    ros::Publisher  base_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/transformed/color/points", 1);

    filtered_cloud->header.frame_id = "panda_link0";    
    base_pcl_pub.publish(filtered_cloud);

    // float max_height = -6.0;

    // BOOST_FOREACH 
}


//main driver function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloth_manipulation_node");
    nh.setParam("/move_group/controller_list", "config/simple_moveit_controllers.yaml");

    ros::NodeHandle nh;
    
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // bool pre_grasp_success = preGraspMovement();

    // if(!pre_grasp_success)
    // {
    //     ros::shutdown();
    //     return 0;
    // }

    detect_grasping_point = false;

    ros::Subscriber pcl_sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, pointCloudCallback); 

    // ros::shutdown();
    // return 0;

    ros::spin ();
}