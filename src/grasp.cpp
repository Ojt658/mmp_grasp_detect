#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <control_msgs/GripperCommandGoal.h>
#include <control_msgs/GripperCommandAction.h>

typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_control_client;

class GraspIt {
    public:
        bool move_to_pose(moveit::planning_interface::MoveGroupInterface&);
        void gripper(bool);
        bool default_pose(moveit::planning_interface::MoveGroupInterface&);
        bool retreat(moveit::planning_interface::MoveGroupInterface&);
};

void GraspIt::gripper(bool grasp) {
    float gripper_pose = (grasp) ? 0.00 : 0.09;
    gripper_control_client gripper_client("/gripper_controller/gripper_action", true);
    gripper_client.waitForServer(); //will wait for infinite time
    control_msgs::GripperCommandGoal gripper_goal;
    gripper_goal.command.max_effort = 20;
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
        transformStamped = tfBuffer.lookupTransform("base_link", "grasp", ros::Time(0));
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
    move_group.move();

    ros::Duration(1).sleep();

    pose.position.x += 0.18;
    pose.position.z -= 0.03;

    move_group.setPoseTarget(pose);
    move_group.move();
}

bool GraspIt::default_pose(moveit::planning_interface::MoveGroupInterface& move_group) {
    std::vector<double> default_pose = {1.480, 0.129, -3.134, 1.534, -1.449, -1.756, -0.321};
    move_group.setJointValueTarget(default_pose);
    return move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

bool GraspIt::retreat(moveit::planning_interface::MoveGroupInterface& move_group) {
    std::vector<double> retreat_pose = {1.240, -0.144, -1.433, 1.425, -0.172, 1.009, 1.894};
    move_group.setJointValueTarget(retreat_pose);
    return move_group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grasp_it");
    ros::NodeHandle nh;
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::AsyncSpinner spinner(1); 
    spinner.start();

    int state;
    bool done = false;

    GraspIt gi;
    gi.default_pose(move_group);
    while (ros::ok()) {
        ros::param::get("/grasp", state);
        if (state == 1) {
            gi.move_to_pose(move_group);
            ros::param::set("/grasp", 2);
            // else ros::param::set("/grasp", 3);
        } else if (state == 2) {
            gi.gripper(true);
            gi.retreat(move_group);
            ros::param::set("/loaded", 2);
        } else if (state == 3) {
            //default pose
            gi.default_pose(move_group);
            gi.gripper(false);
            ros::param::set("/grasp", 0);
            ros::param::set("/loaded", 0);
        }
    }
}
