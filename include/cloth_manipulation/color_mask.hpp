#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <sensor_msgs/PointCloud2.h>

class ColorMask
{
    private:
        ros::NodeHandle nh;

        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;

        ros::Publisher  hsv_pcl_pub, color_mask_pub;
        ros::Subscriber pcl_sub;

        const double tau = 2 * M_PI;

        std::vector<int> LILAC = std::vector<int>{200, 162, 200};
        std::vector<int> ORANGE = std::vector<int>{255, 165, 0};

        double LILAC_mag = std::inner_product(LILAC.begin(), LILAC.end(), LILAC.begin(), 0);
        double ORANGE_mag = std::inner_product(ORANGE.begin(), ORANGE.end(), ORANGE.begin(), 0);

        double matching_parameter = 0.9;
        int minmax_color = 180;

        std::vector<int> current_rgb = std::vector<int>{0, 0, 0};


    public:
        ColorMask(ros::NodeHandle& nh);
        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg);        
};

