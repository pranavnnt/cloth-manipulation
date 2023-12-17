#include "cloth_manipulation/full_grasping_node.hpp"

Grasping::Grasping(ros::NodeHandle& nh_ref) : nh(nh_ref), tf_listener(tf_buffer)
{
    base_pcl_pub = nh_ref.advertise<sensor_msgs::PointCloud2>("/camera/depth/color/points_filtered", 10);
    pcl_sub = nh_ref.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 10, &Grasping::pointCloudCallback, this);

    // Load Controller Configuration
    nh.setParam("/move_group/controller_list", "config/simple_moveit_controllers.yaml");
}


//pre-grasp motion
bool Grasping::preGraspMovement()
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

void Grasping::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg)
{
    // if(!detect_grasping_point) return;
        
    //convert to pcl type
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    //transform point cloud
    try{
        if (tf_buffer.canTransform("panda_link0", points_msg->header.frame_id, points_msg->header.stamp, ros::Duration(1.0)))
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl_ros::transformPointCloud("panda_link0", *pcl_cloud, *transformed_cloud, tf_buffer);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::VoxelGrid<pcl::PointXYZRGB> vox;
            vox.setInputCloud(transformed_cloud);
            vox.setLeafSize(0.005f, 0.005f, 0.005f);
            vox.filter (*filtered_cloud);

            filtered_cloud->header.frame_id = "panda_link0";    
            base_pcl_pub.publish(filtered_cloud);
            // ROS_INFO("Publish success!!");
        }
        else
        {
            ROS_WARN("Transform not available for the given frame and time.");
        }
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("Transform failed: %s", ex.what());
    }

    // float max_height = -6.0;

    // BOOST_FOREACH 
}


//main driver function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloth_manipulation_node");
    ros::NodeHandle nh;
    
    Grasping grasp_obj(nh);

    // ros::AsyncSpinner spinner(3);
    // spinner.start();

    // bool pre_grasp_success = preGraspMovement();

    // if(!pre_grasp_success)
    // {
    //     ros::shutdown();
    //     return 0;
    // }

    // ros::shutdown();
    // return 0;

    ros::spin();
    return 0;
}