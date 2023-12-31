#include "cloth_manipulation/full_grasping_node.hpp"

Grasping::Grasping(ros::NodeHandle& nh_ref) : nh(nh_ref), tf_listener(tf_buffer)
{
    base_pcl_pub = nh_ref.advertise<sensor_msgs::PointCloud2>("/camera/depth/color/points_filtered", 10);
    pcl_sub = nh_ref.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 10, &Grasping::pointCloudCallback, this);

    //move_group.setPlanningTime(45.0);

    detect_grasping_point = 0;
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
  posture.points[0].positions[0] = 0.005;
  posture.points[0].positions[1] = 0.005;
  posture.points[0].time_from_start = ros::Duration(30.0);
  // END_SUB_TUTORIAL
}

//pre-grasp motion
bool Grasping::preGraspMovement(moveit::planning_interface::MoveGroupInterface& move_group)
{
    // Load Controller Configuration : loaded in main function
    //nh.setParam("/move_group/controller_list", "config/simple_moveit_controllers.yaml");
    
    // Set planner
    

    // Set the joint target values
    std::vector<double> pre_grasp_target = {2.166085604918468, -1.3538849044637642, 0.33954661402785985, -2.9432076017150175, 0.4971391740110185, 1.9677453207013593, 0.12029780698200249};

    // Set the joint target
    move_group.setJointValueTarget(pre_grasp_target);
    move_group.setNumPlanningAttempts(5);
    //move_group.setPlanningTime(5.0);

    // Call the planner to compute the plan
    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
    bool success = static_cast<bool>(move_group.plan(pre_grasp_plan));

    //ros::Duration(5.0).sleep();

    if (success)
    {
        ROS_INFO("Pre-grasp plan successful. Executing the plan.");

        // Execute the plan
        move_group.execute(pre_grasp_plan);
        detect_grasping_point++;
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
    if(!(detect_grasping_point == 1)) return;

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
                
                grasp_pose.grasp_pose.header.frame_id = "panda_link0";
                tf2::Quaternion orientation;
                orientation.setRPY(- tau/2, 0, 0);
                grasp_pose.grasp_pose.pose.orientation = tf2::toMsg(orientation);
                grasp_pose.grasp_pose.pose.position.x = highest_blue_point.x;
                grasp_pose.grasp_pose.pose.position.y = highest_blue_point.y;
                grasp_pose.grasp_pose.pose.position.z = highest_blue_point.z;
  
                // Setting pre-grasp approach
                // ++++++++++++++++++++++++++
                /* Defined with respect to frame_id */
                grasp_pose.pre_grasp_approach.direction.header.frame_id = "panda_link0";
                /* Direction is set as positive x axis */
                grasp_pose.pre_grasp_approach.direction.vector.z = -1.0;
                grasp_pose.pre_grasp_approach.min_distance = 0.095;
                grasp_pose.pre_grasp_approach.desired_distance = 0.115;
  
                // Setting post-grasp retreat
                // ++++++++++++++++++++++++++
                /* Defined with respect to frame_id */
                grasp_pose.post_grasp_retreat.direction.header.frame_id = "panda_link0";
                /* Direction is set as positive z axis */
                grasp_pose.post_grasp_retreat.direction.vector.z = 1.0;
                grasp_pose.post_grasp_retreat.min_distance = 0.1;
                grasp_pose.post_grasp_retreat.desired_distance = 0.25;

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

void Grasping::pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // Make sure that when you set the grasp_pose, you are setting it to be the pose of the last link in
    // your manipulator which in this case would be `"panda_link8"` You will have to compensate for the
    // transform from `"panda_link8"` to the palm of the end effector.
    // grasp_pose.grasp_pose.header.frame_id = "panda_link0";
    // tf2::Quaternion orientation;
    // orientation.setRPY(0, -tau / 8, 0);
    // grasp_pose.grasp_pose.pose.orientation = tf2::toMsg(orientation);
    // grasp_pose.grasp_pose.pose.position.x = 0.415;
    // grasp_pose.grasp_pose.pose.position.y = 0;
    // grasp_pose.grasp_pose.pose.position.z = 0.5;
  
    // // Setting pre-grasp approach
    // // ++++++++++++++++++++++++++
    // /* Defined with respect to frame_id */
    // grasp_pose.pre_grasp_approach.direction.header.frame_id = "panda_link0";
    // /* Direction is set as positive x axis */
    // grasp_pose.pre_grasp_approach.direction.vector.x = 1.0;
    // grasp_pose.pre_grasp_approach.min_distance = 0.095;
    // grasp_pose.pre_grasp_approach.desired_distance = 0.115;
  
    // // Setting post-grasp retreat
    // // ++++++++++++++++++++++++++
    // /* Defined with respect to frame_id */
    // grasp_pose.post_grasp_retreat.direction.header.frame_id = "panda_link0";
    // /* Direction is set as positive z axis */
    // grasp_pose.post_grasp_retreat.direction.vector.z = 1.0;
    // grasp_pose.post_grasp_retreat.min_distance = 0.1;
    // grasp_pose.post_grasp_retreat.desired_distance = 0.25;

    move_group.setPlannerId("geometric::RRT");
  
    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    ROS_INFO("Entered the pick function (1)!!");
    openGripper(grasp_pose.pre_grasp_posture);
    // END_SUB_TUTORIAL
    ROS_INFO("Entered the pick function (2)!!");
    // BEGIN_SUB_TUTORIAL pick2
    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasp_pose.grasp_posture);
    // END_SUB_TUTORIAL
    ROS_INFO("Entered the pick function (3)!!");
    // BEGIN_SUB_TUTORIAL pick3
    // Set support surface as table1.
    move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    ROS_INFO("Entered the pick function (4)!!");
    move_group.pick("object", grasp_pose);
    ROS_INFO("Entered the pick function (5)!!");
    // END_SUB_TUTORIAL
}

void Grasping::place(moveit::planning_interface::MoveGroupInterface& move_group)
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
  
    move_group.setPlannerId("geometric::RRTstar");
    
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
    move_group.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    move_group.place("object", place_location);
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
    collision_objects.resize(4);

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
    collision_objects[0].primitive_poses[0].position.z = -2;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[1].header.frame_id = "panda_link0";
    collision_objects[1].id = "object";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.05;
    collision_objects[1].primitives[0].dimensions[1] = 0.05;
    collision_objects[1].primitives[0].dimensions[2] = 0.05 ;

    /* Define the pose of the object. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = -0.5;
    collision_objects[1].primitive_poses[0].position.z = -0.2;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

     // Add the first table where the cube will originally be kept.
    collision_objects[2].id = "table1";
    collision_objects[2].header.frame_id = "panda_link0";

    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 2;

    /* Define the pose of the object. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.5;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

//main driver function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloth_manipulation_node");
    ros::NodeHandle nh;

    // Load Controller Configuration
    nh.setParam("/move_group/controller_list", "config/simple_moveit_controllers.yaml");

    const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    Grasping grasp_obj(nh);
    grasp_obj.addCollisionObjects(planning_scene_interface);

    bool pre_grasp_success = grasp_obj.preGraspMovement(move_group);

    if(!pre_grasp_success)
    {
        ros::shutdown();
        return 0;
    }

    while(ros::ok())
    {
        if(grasp_obj.detect_grasping_point == 2)
        {
            ros::WallDuration(1.0).sleep();
            grasp_obj.pick(move_group);

            ros::WallDuration(1.0).sleep();
            grasp_obj.place(move_group);
            break;
        }
        else
        {
            continue;
        }
    }

    ros::shutdown();
    return 0;
}