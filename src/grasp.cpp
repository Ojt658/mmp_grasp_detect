#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_control_client;

class GraspIt {
    public:
        bool move_to_pose(moveit::planning_interface::MoveGroupInterface&);
        void gripper();
};

void GraspIt::gripper() {
    float gripper_pose = 10;
    gripper_control_client gripper_client("/panda_hand_controller/gripper_action", true);
    gripper_client.waitForServer(); //will wait for infinite time
    control_msgs::FollowJointTrajectoryActionGoal gripper_goal;
    gripper_goal.command.max_effort = 11.0;
    gripper_goal.command.position = gripper_pose;
    gripper_client.sendGoal(gripper_goal);
    gripper_client.waitForResult(ros::Duration(5.0));
}

bool GraspIt::move_to_pose(moveit::planning_interface::MoveGroupInterface& move_group) {  // Position above the object/bucket using the transform frame
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Pose pose;

    ros::Duration(1.0).sleep();

    try {
        transformStamped = tfBuffer.lookupTransform("panda_link0", "grasp", ros::Time(0));
    } catch (tf2::TransformException &ex) {
        // ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }

    // ROS_INFO("Translation: [%f, %f, %f]", transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    pose.position.x = transformStamped.transform.translation.x;
    pose.position.y = transformStamped.transform.translation.y;
    pose.position.z = transformStamped.transform.translation.z; // the z offset makes the robot position the arm above the object to later move down on
    pose.orientation = transformStamped.transform.rotation;

    move_group.setPoseTarget(pose);
    return move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grasp_it");
    ros::NodeHandle nh;
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    GraspIt gi;
    gi.move_to_pose(move_group);
    
    return 0;
}
