#include "cloth_manipulation/full_grasping_node.hpp"

Grasping::Grasping(ros::NodeHandle& nh_ref) : nh(nh_ref), tf_listener(tf_buffer)
{
    base_pcl_pub = nh_ref.advertise<sensor_msgs::PointCloud2>("/camera/depth/color/points_filtered", 10);
    pcl_sub = nh_ref.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 10, &Grasping::pointCloudCallback, this);

    ft_abs_sum_pub = nh_ref.advertise<std_msgs::Float32>("/joint_states_abs_summed", 100);
    joint_sub = nh_ref.subscribe<sensor_msgs::JointState>("/joint_states", 100, &Grasping::jointStateCallback, this);

    PLANNING_GROUP = "panda_arm";
    GRIPPER_GROUP = "panda_hand";
    move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
    move_group_gripper.reset(new moveit::planning_interface::MoveGroupInterface(GRIPPER_GROUP));

    open_gripper = {0.035,0.035};
    closed_gripper = {0.002,0.002};

    move_group->setPlannerId("AnytimePathShortening");

    addCollisionObjects(planning_scene_interface);

    move_group->setPlanningTime(5.0);
    move_group_gripper->setPlanningTime(15.0);

    detect_grasping_point = -10;
    check_avg = false;

    pre_mean = 0;
    pre_std = 0;
    post_mean = 0;
    post_std = 0;
    fault_detection;
}

void Grasping::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_msg)
{
    if(!check_avg) 
        return;
    // ROS_INFO("Entered here!!!!!!!");

    if(ft_abs_sum_avg.size() == 10)
        ft_abs_sum_avg.clear();

    std_msgs::Float32 sum_msg, abs_sum_msg;
    // for (int i = 0; i < 7; i++)
    // {
    //     // abs_sum_msg.data += std::abs(joint_msg->effort[i]);
    //      abs_sum_msg.data += (joint_msg->effort[i]);
    // }
    abs_sum_msg.data = joint_msg->effort[6];

    ft_abs_sum_avg.push_back(abs_sum_msg.data);
    ft_abs_sum_pub.publish(abs_sum_msg);

    if(ft_abs_sum_avg.size() == 25) check_avg = false;
}

//pre-grasp motion
void Grasping::viewingMovement()
{
    // Load Controller Configuration : loaded in main function
    //nh.setParam("/move_group/controller_list", "config/simple_moveit_controllers.yaml");
    
    // Set planner

    ROS_INFO("Entered pregrasp!");
    
    move_group->setMaxVelocityScalingFactor(0.25);
    move_group->setMaxAccelerationScalingFactor(0.25);

    // Set the joint target values
    std::vector<double> viewing_target = {2.5633511613743347, -1.4671690645033286, 0.10065576503901413, -1.8405446478358485, 0.32594827116271596, 1.1669870363606438, 0.5493170644657479};

    // Set the joint target
    move_group->setJointValueTarget(viewing_target);
    move_group->setNumPlanningAttempts(5);
    move_group->setPlanningTime(5.0);

    // Call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan viewing_plan;
    bool success = static_cast<bool>(move_group->plan(viewing_plan));

    //ros::Duration(5.0).sleep();

    if (success)
    {
        ROS_INFO("Pre-grasp plan successful. Executing the plan.");

        // Execute the plan
        move_group->execute(viewing_plan);
        detect_grasping_point++;
    }
    else
    {
        ROS_ERROR("Planning failed");
    }
    check_avg = true;

    move_group_gripper->setJointValueTarget(open_gripper);
    // Call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    success = static_cast<bool>(move_group_gripper->plan(gripper_plan));
    if (success)
    {
        ROS_INFO("Gripper action successful. Executing the plan.");
        move_group_gripper->execute(gripper_plan);
    }
}

//point cloud sub callback
void Grasping::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& points_msg)
{
    //ROS_INFO_STREAM(detect_grasping_point);
    if(detect_grasping_point == -9) 
    {
        ROS_INFO("Entered subscriber for finding grasping point!!");

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

                // Initialize variables to store the highest blue point
                bool found_blue_point = false;

                // Use PCL iterators for optimized iteration
                for(int nIndex = 0; nIndex < filtered_cloud->points.size(); nIndex++)
                {
                    auto it = filtered_cloud->points[nIndex];
                    // ROS_INFO_STREAM("x is " << it.x << ", y is " << it.y << ", z is " << it.z);
                    // ROS_INFO_STREAM("Red is " << (int)it.r << ", green is " << (int)it.g << ", blue is " << (int)it.b);
                    // ROS_INFO_STREAM("--------------------------------------------------------------------");
                    // Check if the point is blue (you may need to adjust these thresholds)
                    if ((int)it.b > 80 && ((int)it.b*1.0)/std::min((int)it.r,(int)it.g) > 4.0 && it.x > -0.7)
                    {   
                        // ROS_INFO("Found a blue point!!");
                        // Check if the point has higher z-coordinate than the current highest point
                        if (!found_blue_point || it.z > highest_blue_point.z)
                        {
                            highest_blue_point = it;
                            found_blue_point = true;
                        }
                    }
                }
                if (found_blue_point == true)
                {
                    ROS_INFO_STREAM("Location of highest blue point is : x="<< highest_blue_point.x <<", y="<< highest_blue_point.y <<", z="<< highest_blue_point.z);
                    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group->getEndEffectorLink().c_str());

                    tf2::Quaternion orientation;
                    orientation.setRPY(- tau/2, 0, 0);
                    grasp_pose.orientation = tf2::toMsg(orientation);

                    grasp_pose.position.x = highest_blue_point.x;
                    grasp_pose.position.y = highest_blue_point.y;
                    grasp_pose.position.z = highest_blue_point.z + 0.093;

                    detect_grasping_point++;
                }
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
    }
    else if(detect_grasping_point == -7 || detect_grasping_point == -5 || detect_grasping_point == -3)
    {
        ROS_INFO("Entered subscriber for fault detection!");
        
        //convert to pcl type
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*points_msg, *pcl_cloud);
        
        // Use PCL iterators for optimized iteration
        for(int nIndex = 0; nIndex < pcl_cloud->points.size(); nIndex++)
        {
            auto it = pcl_cloud->points[nIndex];
            // ROS_INFO_STREAM("x is " << it.x << ", y is " << it.y << ", z is " << it.z);
            // ROS_INFO_STREAM("Red is " << (int)it.r << ", green is " << (int)it.g << ", blue is " << (int)it.b);
            // ROS_INFO_STREAM("--------------------------------------------------------------------");
            // Check if the point is blue (you may need to adjust these thresholds)
            if ((int)it.b > 80 && ((int)it.b*1.0)/std::min((int)it.r,(int)it.g) > 4.0 && it.z < -0.7 || (int)it.b > 80 && it.z < -0.5)
            {   
                ROS_INFO("Looks like the napkin is there!!!");
                fault_detection = true;
            }
        }
    }
}

int Grasping::planningRoutine()
{
    move_group->setPoseTarget(grasp_pose);

    moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
    bool success = static_cast<bool>(move_group->plan(grasp_plan));

    if(success)
    {
        ROS_INFO("Can plan path to grasp!");
        ROS_INFO("But first, we will do a pre-grasp maneuver");

        geometry_msgs::Pose pre_grasp_pose;
        pre_grasp_pose.orientation = grasp_pose.orientation;
        pre_grasp_pose.position.x = grasp_pose.position.x;
        pre_grasp_pose.position.y = grasp_pose.position.y;
        pre_grasp_pose.position.z = grasp_pose.position.z + 0.4;

        move_group->setPoseTarget(pre_grasp_pose);

        moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
        success = static_cast<bool>(move_group->plan(pre_grasp_plan));

        if(success)
        {
            ROS_INFO("Can plan pre grasp pose too! Let's execute it.");
            move_group->execute(pre_grasp_plan);

            move_group_gripper->setJointValueTarget(open_gripper);
            move_group_gripper->move();

            ros::WallDuration(2.0).sleep();
            ROS_INFO("Checking ft value");
            check_avg = true;
            ros::WallDuration(2.0).sleep();

            //ft_avg code
            std::vector<float> v1 = ft_abs_sum_avg;
            double sum = std::accumulate(v1.begin(), v1.end(), 0.0);
            pre_mean = sum / v1.size();
            double sq_sum = std::inner_product(v1.begin(), v1.end(), v1.begin(), 0.0);
            pre_std = std::sqrt(sq_sum / v1.size() - pre_mean * pre_mean);
            ROS_INFO_STREAM("Pre-grasp ft values are " << pre_mean << " and " <<pre_std);

            move_group->setPoseTarget(grasp_pose);

            moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
            bool success = static_cast<bool>(move_group->plan(grasp_plan));
            if(success)
            {
                ROS_INFO("Can still plan grasp pose! Let's execute it.");

                move_group->execute(grasp_plan);

                move_group_gripper->setJointValueTarget(closed_gripper);
                move_group_gripper->move();

                move_group->setPoseTarget(pre_grasp_pose);
                move_group->move();

                detect_grasping_point++;

                //put ft_avg code here
                ros::WallDuration(2.0).sleep();
                ROS_INFO("Checking ft value");
                check_avg = true;
                ros::WallDuration(2.0).sleep();

                //ft_avg code
                std::vector<float> v1 = ft_abs_sum_avg;
                double sum = std::accumulate(v1.begin(), v1.end(), 0.0);
                post_mean = sum / v1.size();

                double sq_sum = std::inner_product(v1.begin(), v1.end(), v1.begin(), 0.0);
                post_std = std::sqrt(sq_sum / v1.size() - post_mean * post_mean);

                ROS_INFO_STREAM("Pre-grasp ft values are " << pre_mean << " and " <<pre_std);
                ROS_INFO_STREAM("Post-grasp ft values are " << post_mean << " and " <<post_std);

                if(fault_detection)
                {
                    ROS_INFO("Congratulations, you have a blue napkin!");
                    return 1;
                }
                else
                {
                    ROS_INFO("Failed the grasp, try again!");
                    return 0;
                }
            }
            else
            {
                ROS_WARN("Cannot plan grasp pose! (3)");
                return 2;
            }
        }
        else
        {
            ROS_WARN("Cannot plan pre-grasp pose! (2)");
            return 2;
        }
    }
    else
    {
        ROS_WARN("Cannot plan grasp pose! (1)");
        return 2;
    }
}

//main driver function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloth_manipulation_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);
    spinner.start();
    
    Grasping grasp_obj(nh);

    // double pre_mean, pre_std, post_mean, post_std;
    
    grasp_obj.viewingMovement();
 
    while(ros::ok())
    {
        if(grasp_obj.detect_grasping_point == -8)
        {
            bool grasp_success = grasp_obj.planningRoutine();
            if(grasp_success)
            {
                grasp_obj.detect_grasping_point++;
                break;
            }
            else
            {
                grasp_success = grasp_obj.planningRoutine();
                if(grasp_success)
                {
                    grasp_obj.detect_grasping_point++;
                    break;
                }
                else
                {
                    grasp_success = grasp_obj.planningRoutine();
                    if(grasp_success)
                    {
                        grasp_obj.detect_grasping_point++;
                        break;
                    }
                    else
                    {
                        ROS_INFO("It's not meant to be...");
                        //detect_grasping_point++;
                        break;
                    }
                }
            }
        }
        else
        {
            continue;
        }
    }

    grasp_obj.viewingMovement();

    ros::shutdown();
    return 0;
}