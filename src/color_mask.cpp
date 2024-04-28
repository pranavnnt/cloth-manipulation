#include "cloth_manipulation/color_mask.hpp"

ColorMask::ColorMask(ros::NodeHandle& nh_ref) : nh(nh_ref), tf_listener(tf_buffer)
{
    hsv_pcl_pub = nh_ref.advertise<sensor_msgs::PointCloud2>("/camera/depth/color/points_hsv", 10);
    color_mask_pub = nh_ref.advertise<sensor_msgs::PointCloud2>("/camera/depth/color/points_masked", 10);
    pcl_sub = nh_ref.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 10, &ColorMask::pointCloudCallback, this);

    //move_group.setPlanningTime(45.0);
}

//point cloud sub callback
void ColorMask::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg)
{
    ROS_INFO("Entered subscriber!!");
        
    //convert to pcl type
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    //transform point cloud
    
    // try{
    // if (tf_buffer.canTransform("panda_link0", points_msg->header.frame_id, points_msg->header.stamp, ros::Duration(1.0)))
    
    if(true)
    {
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        // pcl_ros::transformPointCloud("panda_link0", *pcl_cloud, *transformed_cloud, tf_buffer);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::VoxelGrid<pcl::PointXYZRGB> vox;
        vox.setInputCloud(pcl_cloud);
        // vox.setLeafSize(0.01f, 0.01f, 0.01f);
        // vox.filter(*filtered_cloud);
        *filtered_cloud = *pcl_cloud;

        pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud(new pcl::PointCloud<pcl::PointXYZHSV>);
        hsv_cloud->points.resize(filtered_cloud->size());

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        hsv_cloud->header.frame_id = pcl_cloud->header.frame_id;
        new_cloud->header.frame_id = pcl_cloud->header.frame_id;     

        for(int nIndex = 0; nIndex < filtered_cloud->points.size(); nIndex++)
        {
            pcl::PointXYZRGBtoXYZHSV(filtered_cloud->points[nIndex], hsv_cloud->points[nIndex]);

            // if)hsv_cloud->points[nIndex].h << " S: " << hsv_cloud->points[nIndex].s << " V: " << hsv_cloud->points[nIndex].v << std::endl;
            if(hsv_cloud->points[nIndex].h > 5 && hsv_cloud->points[nIndex].h < 45 && hsv_cloud->points[nIndex].v > 0.5 && hsv_cloud->points[nIndex].s > 0.15)
            {
                new_cloud->points.push_back(filtered_cloud->points[nIndex]);
            }

            // double rgb_mag = std::inner_product(rgb.begin(), rgb.end(), rgb.begin(), 0);

            // double dot_LILAC = std::inner_product(rgb.begin(), rgb.end(), LILAC.begin(), 0);
            // double dot_ORANGE = std::inner_product(rgb.begin(), rgb.end(), ORANGE.begin(), 0);

            // double mag = sqrt(rgb_mag * LILAC_mag);
            // double cos_theta_LILAC = dot_LILAC / mag;

            // mag = sqrt(rgb_mag * ORANGE_mag);
            // double cos_theta_ORANGE = dot_ORANGE / mag;

            // if(cos_theta_LILAC > matching_parameter)
            // {
            //     current_rgb = LILAC;
            // }
            // else if(cos_theta_ORANGE > matching_parameter)
            // {
            //     current_rgb = ORANGE;
            // }
            // else
            // {
            //     current_rgb = std::vector<int>{0, 0, 0};
            // }

            // pcl::PointXYZHSV hsv_point;
            // hsv_point.x = point.x;
            // hsv_point.y = point.y;
            // hsv_point.z = point.z;
            // hsv_point.h = current_rgb[0];
            // hsv_point.s = current_rgb[1];
            // hsv_point.v = current_rgb[2];

            // hsv_pcl.push_back(hsv_point);
        // }
        // pcl::PointXYZRGBtoXYZHSV(*filtered_cloud, hsv_pcl);

                 
        }
    hsv_pcl_pub.publish(hsv_cloud); 
    color_mask_pub.publish(new_cloud); 
    // float max_height = -6.0;
    }

    // BOOST_FOREACH 
}

//main driver function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_mask");
    ros::NodeHandle nh;

    ColorMask color_mask(nh);

    ros::spin();

    ros::shutdown();
    return 0;
}