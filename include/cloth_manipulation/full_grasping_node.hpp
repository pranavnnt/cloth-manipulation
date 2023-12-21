#include <ros/ros.h>
#include <vector>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <trajectory_msgs/JointTrajectory.h>

class Grasping
{
    private:
        ros::NodeHandle nh;

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;

        ros::Publisher  base_pcl_pub;
        ros::Publisher  ft_abs_sum_pub;
        ros::Subscriber pcl_sub;
        ros::Subscriber joint_sub;

        const double tau = 2 * M_PI;

        moveit_msgs::Grasp grasp_pose;
        pcl::PointXYZRGB highest_blue_point;

        std::string PLANNING_GROUP, GRIPPER_GROUP;
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group, move_group_gripper;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        std::vector<double> open_gripper, closed_gripper;

    public:
        Grasping(ros::NodeHandle& nh);
        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg);
        void jointStateCallback(const sensor_msgs::JointState::ConstPtr& state);
        void viewingMovement();
        void planningRoutine(geometry_msgs::Pose grasp_pose);
        
        void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

        int detect_grasping_point;
        bool check_avg;
        std::vector<float> ft_abs_sum_avg;
};

void Grasping::addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "panda_link0";
    
    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;
    
    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = -0.2;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL
    
    collision_objects[0].operation = collision_objects[0].ADD;
    
    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "panda_link0";
    
    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.4;
    
    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.5;
    collision_objects[1].primitive_poses[0].position.z = 0.2;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL
    
    collision_objects[1].operation = collision_objects[1].ADD;
    
    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[2].header.frame_id = "panda_link0";
    collision_objects[2].id = "object";
    
    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.2;
    
    /* Define the pose of the object. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 2.5;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL
    
    collision_objects[2].operation = collision_objects[2].ADD;
    
    planning_scene_interface.applyCollisionObjects(collision_objects);
}

