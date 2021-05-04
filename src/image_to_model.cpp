#include <ros/ros.h>
#include "mmp_grasp_detect/GetGrasp.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class Image2Model {
    bool found;
    std::vector<double> grasp = {};

    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_; 
        image_transport::Subscriber image_sub_;
        ros::ServiceClient client = nh_.serviceClient<mmp_grasp_detect::GetGrasp>("get_grasp");
    
    public:
        Image2Model() : it_(nh_) {
            image_sub_ = it_.subscribe("/head_camera/rgb/image_raw",
                                1, &Image2Model::imageCb, this);

            found = false;
        }

        void imageCb(const sensor_msgs::ImageConstPtr&);
        void tfBroadcaster();
};

void Image2Model::imageCb(const sensor_msgs::ImageConstPtr& msg) {
    int loaded;
    ros::param::get("/loaded", loaded);

    if (loaded == 1 && !found) {
        mmp_grasp_detect::GetGrasp srv;
        sensor_msgs::Image img = *msg;
        srv.request.image = img;
        if (client.call(srv)) {
            ROS_INFO_STREAM("Grasp:");
            for (int i = 0; i < 6; i++) {
                grasp.push_back(srv.response.grasp[i]);
            }
            ROS_INFO_STREAM("X: " << grasp[0]);
            ROS_INFO_STREAM("Y: " << grasp[1]);
            ROS_INFO_STREAM("Z: " << grasp[2]);
            ROS_INFO_STREAM("ROLL: " << grasp[3]);
            ROS_INFO_STREAM("PITCH: " << grasp[4]);
            ROS_INFO_STREAM("YAW: " << grasp[5]);
            found = true;
            ros::param::set("/grasp", 1);
            tfBroadcaster();
        } else {
            ROS_INFO_STREAM("Error getting grasp from service");
        }
    } else if (loaded == 2) {
        found = false;
    }
}

void Image2Model::tfBroadcaster() {
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // Broadcast to the transform frame the pose of the target object
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "target_object";
    transformStamped.child_frame_id = "grasp";
    transformStamped.transform.translation.x = -grasp[0] - 0.22;
    transformStamped.transform.translation.y = grasp[1] - 0.12;
    transformStamped.transform.translation.z = grasp[2] + 0.12;

    tf2::Quaternion q;
    q.setRPY(grasp[3], grasp[4], grasp[5]); // roll pitch yaw of end effector
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    
    br.sendTransform(transformStamped);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_to_model");

    Image2Model im;

    ros::spin();
}