/*
This ROS node is called in the intitial setup in mmp.launch. It initialises the moveit planning scene and sets the default pose for the robot joints.
*/

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> head_control_client;

void look_down(){
    std::string head_joint_names[2] = {"head_pan_joint", "head_tilt_joint"};
    float head_pos[2] = {0.0, 0.75};
    head_control_client head_client("/head_controller/follow_joint_trajectory", true);
    head_client.waitForServer(); //will wait for infinite time

    control_msgs::FollowJointTrajectoryGoal head_goal;  // Define the goal for the head tilt
    head_goal.trajectory.joint_names.push_back(head_joint_names[0]);
    head_goal.trajectory.joint_names.push_back(head_joint_names[1]);
    head_goal.trajectory.points.resize(1);
    head_goal.trajectory.points[0].positions.resize(2);
    head_goal.trajectory.points[0].positions[0] = head_pos[0];
    head_goal.trajectory.points[0].positions[1] = head_pos[1];

    head_goal.trajectory.points[0].velocities.resize(2);
    for (int j = 0; j < 2; ++j) {
        head_goal.trajectory.points[0].velocities[j] = 0.0;
    }
    head_goal.trajectory.points[0].time_from_start = ros::Duration(3.0);
    head_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    head_client.sendGoal(head_goal);
    head_client.waitForResult(ros::Duration(3.0));
}

void collisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    // Table collision 
    collision_objects[0].id = "table";
    collision_objects[0].header.frame_id = "base_link";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 1;
    collision_objects[0].primitives[0].dimensions[1] = 1;
    collision_objects[0].primitives[0].dimensions[2] = 0.379;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 1;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.195;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    collision_objects[0].primitive_poses[0].orientation.w = q.w();
    collision_objects[0].primitive_poses[0].orientation.x = q.x();
    collision_objects[0].primitive_poses[0].orientation.y = q.y();
    collision_objects[0].primitive_poses[0].orientation.z = q.z();

    collision_objects[0].operation = collision_objects[0].ADD;

    //     // object collision 
    // collision_objects[1].id = "object";
    // collision_objects[1].header.frame_id = "panda_link0";

    // /* Define the primitive and its dimensions. */
    // collision_objects[1].primitives.resize(1);
    // collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
    // collision_objects[1].primitives[0].dimensions.resize(2);
    // collision_objects[1].primitives[0].dimensions[0] = 1;
    // collision_objects[1].primitives[0].dimensions[1] = 1;


    // /* Define the pose of the table. */
    // collision_objects[1].primitive_poses.resize(1);
    // collision_objects[1].primitive_poses[0].position.x = 0.711;
    // collision_objects[1].primitive_poses[0].position.y = 0;
    // collision_objects[1].primitive_poses[0].position.z = 0;

    // q.setRPY(0, 0, 0);
    // collision_objects[1].primitive_poses[0].orientation.w = q.w();
    // collision_objects[1].primitive_poses[0].orientation.x = q.x();
    // collision_objects[1].primitive_poses[0].orientation.y = q.y();
    // collision_objects[1].primitive_poses[0].orientation.z = q.z();

    // collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "scene_pub");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Placing collision object for the table");
    moveit::planning_interface::PlanningSceneInterface pi;
    collisionObjects(pi);

    ROS_INFO_STREAM("Going to default head pose");
    look_down();

    ros::spin();
}