#include <ros/ros.h>

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
        ros::Subscriber pcl_sub;

        int detect_grasping_point;

        // Planning group
        const std::string PLANNING_GROUP;
        moveit::planning_interface::MoveGroupInterface move_group;

        const double tau = 2 * M_PI;

    public:
        Grasping(ros::NodeHandle& nh);
        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg);
        bool preGraspMovement();

        void openGripper(trajectory_msgs::JointTrajectory& posture);
        void closedGripper(trajectory_msgs::JointTrajectory& posture);

        void pick(moveit_msgs::Grasp grasp_pose);
        void place(std::vector<moveit_msgs::PlaceLocation> place_location);
        
};

