#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>

void default_pose(moveit::planning_interface::MoveGroupInterface& move_group) {
    std::vector<double> default_arm_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 2.1, 0.55};
    move_group.setJointValueTarget(default_arm_pos);
    move_group.move();
}

void collisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "kinect";
    collision_objects[0].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.073;
    collision_objects[0].primitives[0].dimensions[1] = 0.276;
    collision_objects[0].primitives[0].dimensions[2] = 0.072;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.115;
    collision_objects[0].primitive_poses[0].position.y = -0.341;
    collision_objects[0].primitive_poses[0].position.z = 0.632;

    tf2::Quaternion q;
    q.setRPY(0, 0.395, 0.443);
    collision_objects[0].primitive_poses[0].orientation.w = q.w();
    collision_objects[0].primitive_poses[0].orientation.x = q.x();
    collision_objects[0].primitive_poses[0].orientation.y = q.y();
    collision_objects[0].primitive_poses[0].orientation.z = q.z();

    collision_objects[0].operation = collision_objects[0].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "scene_pub");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Placing collision object for depth camera");
    moveit::planning_interface::PlanningSceneInterface pi;
    collisionObjects(pi);

    ROS_INFO_STREAM("Going to default pose");
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    default_pose(move_group);
    ros::spin();
    return 0;
}