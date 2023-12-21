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
    closed_gripper = {0.005,0.005};

    move_group->setPlannerId("AnytimePathShortening");

    addCollisionObjects(planning_scene_interface);

    move_group->setPlanningTime(5.0);
    move_group_gripper->setPlanningTime(15.0);

    detect_grasping_point = -10;
    check_avg = false;
}

void Grasping::openGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.035;
  posture.points[0].positions[1] = 0.035;
  posture.points[0].time_from_start = ros::Duration(30.0);
}

void Grasping::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.000;
  posture.points[0].positions[1] = 0.000;
  posture.points[0].time_from_start = ros::Duration(30.0);
  // END_SUB_TUTORIAL
}

void Grasping::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_msg)
{
    if(!check_avg) 
        return;

    if(ft_abs_sum_avg.size() == 25)
        ft_abs_sum_avg.clear();

    std_msgs::Float32 sum_msg, abs_sum_msg;
    for (int i = 0; i < 7; i++)
    {
        abs_sum_msg.data += std::abs(joint_msg->effort[i]);
    }

    ft_abs_sum_avg.push_back(abs_sum_msg.data);
    ft_abs_sum_pub.publish(abs_sum_msg);

    if(ft_abs_sum_avg.size() == 25) check_avg = false;
}

//pre-grasp motion
void Grasping::preGraspMovement()
{
    // Load Controller Configuration : loaded in main function
    //nh.setParam("/move_group/controller_list", "config/simple_moveit_controllers.yaml");
    
    // Set planner

    ROS_INFO("Entered pregrasp!");
    
    move_group->setMaxVelocityScalingFactor(0.25);
    move_group->setMaxAccelerationScalingFactor(0.25);

    // Set the joint target values
    std::vector<double> pre_grasp_target = {2.5633511613743347, -1.4671690645033286, 0.10065576503901413, -1.8405446478358485, 0.32594827116271596, 1.1669870363606438, 0.5493170644657479};

    // Set the joint target
    move_group->setJointValueTarget(pre_grasp_target);
    move_group->setNumPlanningAttempts(5);
    move_group->setPlanningTime(5.0);

    // Call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
    bool success = static_cast<bool>(move_group->plan(pre_grasp_plan));

    //ros::Duration(5.0).sleep();

    if (success)
    {
        ROS_INFO("Pre-grasp plan successful. Executing the plan.");

        // Execute the plan
        move_group->execute(pre_grasp_plan);
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
    ROS_INFO_STREAM(detect_grasping_point);
    if(!(detect_grasping_point == -9)) return;

    ROS_INFO("Entered subscriber!!");
        
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
                if ((int)it.b > 80 && ((int)it.b*1.0)/std::min((int)it.r,(int)it.g) > 5.0)
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

                geometry_msgs::Pose target_pose1;

                tf2::Quaternion orientation;
                orientation.setRPY(- tau/2, 0, 0);
                target_pose1.orientation = tf2::toMsg(orientation);
                
                target_pose1.position.x = highest_blue_point.x;
                target_pose1.position.y = highest_blue_point.y;
                target_pose1.position.z = highest_blue_point.z + 0.103;

                move_group->setPoseTarget(target_pose1);
                move_group->move();

                move_group_gripper->setJointValueTarget(closed_gripper);
                move_group_gripper->move();

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

    // float max_height = -6.0;

    // BOOST_FOREACH 
}

void Grasping::pick()
{
    move_group->setPlanningTime(5.0);
    move_group->setMaxVelocityScalingFactor(0.2);
    move_group->setMaxAccelerationScalingFactor(0.2);

  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
  // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
  // transform from `"panda_link8"` to the palm of the end effector.
//   grasps[0].grasp_pose.header.frame_id = "panda_link0";
//   tf2::Quaternion orientation;
//   orientation.setRPY(- tau/2, 0, 0);
//   grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
//   grasps[0].grasp_pose.pose.position.x = -0.45;
//   grasps[0].grasp_pose.pose.position.y = 0.25;
//   grasps[0].grasp_pose.pose.position.z = 0.0 + 0.103;

//   // Setting pre-grasp approach
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
//   /* Direction is set as positive x axis */
//   grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
//   grasps[0].pre_grasp_approach.min_distance = 0.06;
//   grasps[0].pre_grasp_approach.desired_distance = 0.115;

//   // Setting post-grasp retreat
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
//   /* Direction is set as positive z axis */
//   grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
//   grasps[0].post_grasp_retreat.min_distance = 0.1;
//   grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  move_group->setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  move_group->pick("object", grasp_pose);
}

void Grasping::place()
{
    // BEGIN_SUB_TUTORIAL place
    // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
    // location in verbose mode." This is a known issue. |br|
    // |br|
    // Ideally, you would create a vector of place locations to be attempted although in this example, we only create
    // // a single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);
  
    // Setting place location pose
    // +++++++++++++++++++++++++++
    ROS_INFO("Entered the place function (1)!!");
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-tau/2, 0, 0);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
    ROS_INFO("Entered the place function (2)!!");
  
    /* For place location, we set the value to the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = highest_blue_point.x;
    place_location[0].place_pose.pose.position.y = highest_blue_point.y;
    place_location[0].place_pose.pose.position.z = highest_blue_point.z;

    ROS_INFO("Entered the place function (3)!!");
  
    // Setting pre-place approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.x = 1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;
  
    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.x = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    ROS_INFO("Entered the place function (4)!!");
  
    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);
  
    ROS_INFO("Entered the place function (5)!!");
    // Set support surface as table2.
    move_group->setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    move_group->place("object", place_location);
    ROS_INFO("Entered the place function (6)!!");
    // END_SUB_TUTORIAL
}

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

//main driver function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloth_manipulation_node");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);
    spinner.start();
    
    Grasping grasp_obj(nh);

    double pre_mean, pre_std, post_mean, post_std;
    
    grasp_obj.preGraspMovement();
 
    while(ros::ok())
    {
        if(grasp_obj.detect_grasping_point == -8)
        {
            // ROS_INFO("Here");
            if(grasp_obj.check_avg) continue;
            else
            {
                ROS_INFO("Here also");
                
                //pregrasp mean and stddev
                std::vector<float> v1 = grasp_obj.ft_abs_sum_avg;
                double sum = std::accumulate(v1.begin(), v1.end(), 0.0);
                pre_mean = sum / v1.size();

                double sq_sum = std::inner_product(v1.begin(), v1.end(), v1.begin(), 0.0);
                pre_std = std::sqrt(sq_sum / v1.size() - pre_mean * pre_mean);

                ROS_INFO_STREAM("here are " << pre_mean << " and " <<pre_std);
            }
            ROS_INFO_STREAM("The pre-grasp ft mean is " << pre_mean <<" while the std is " << pre_std);
            ros::WallDuration(1.0).sleep();
            //grasp_obj.pick();

            break;
        }
        else
        {
            continue;
        }
    }

    grasp_obj.preGraspMovement();

    while(ros::ok())
    {
        if(grasp_obj.check_avg) continue;
            else
            {
                //pregrasp mean and stddev
                std::vector<float> v1 = grasp_obj.ft_abs_sum_avg;
                double sum = std::accumulate(v1.begin(), v1.end(), 0.0);
                post_mean = sum / v1.size();

                double sq_sum = std::inner_product(v1.begin(), v1.end(), v1.begin(), 0.0);
                post_std = std::sqrt(sq_sum / v1.size() - post_mean * post_mean);

                ROS_INFO_STREAM("here are " << pre_mean << " and " <<pre_std);
                break;
            }
    }
    ROS_INFO_STREAM("The pre-grasp ft mean is " << pre_mean <<" while the std is " << pre_std);
    ROS_INFO_STREAM("The post-grasp ft mean is " << post_mean);

    ros::shutdown();
    return 0;
}