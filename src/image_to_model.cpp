/*
This ROS node utilises the GetGraspService to retrieve a grasp for the current object.
*/

#include <ros/ros.h>
#include "mmp_grasp_detect/GetGrasp.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>

class Image2Model {
    bool found, loaded;
    std::vector<double> grasp = {};

    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_; 
        image_transport::Subscriber image_sub_;
        ros::Subscriber loaded_sub_;
        ros::ServiceClient client = nh_.serviceClient<mmp_grasp_detect::GetGrasp>("get_grasp");
    
    public:
        Image2Model() : it_(nh_) {
            image_sub_ = it_.subscribe("/head_camera/rgb/image_raw",
                                1, &Image2Model::imageCb, this);

            loaded_sub_ = nh_.subscribe("/model_loaded", 1, &Image2Model::loadedCb, this);

            found = false;
            loaded = false;
        }

        void imageCb(const sensor_msgs::ImageConstPtr&);
        void loadedCb(const std_msgs::Bool&);
        void tfBroadcaster();

        void add_grasp(std::vector<double> grasp) { // Save the Grasp to the results file
            std::ofstream file;

            file.open("/home/ollie/mmp_ws/src/mmp_grasp_detect/results/results.csv", std::ios_base::app);
            file << "[";
            for (int g = 0; g < grasp.size(); g++) {
                if (g < grasp.size() - 1) {
                    file << grasp[g] << ", ";
                } else {
                    file << grasp[g];
                }
            }
            file << "], ";
        }
};

void Image2Model::loadedCb(const std_msgs::Bool& model_loaded) {
    // ROS_INFO("LOADED");
    int load_param;
    ros::param::get("/loaded", load_param);

    if (load_param == 0) {
        loaded = model_loaded.data;
    } else if (load_param == 2) {
        loaded = false;
    }
}

void Image2Model::imageCb(const sensor_msgs::ImageConstPtr& msg) {
    if (loaded && !found) { // This logic makes sure the service is only called once per object
        mmp_grasp_detect::GetGrasp srv;
        sensor_msgs::Image img = *msg;
        srv.request.image = img;
        if (client.call(srv)) {  // Call the grasp service to get the grasp prediction
            // ROS_INFO_STREAM("Grasp:");
            grasp.clear();
            for (int i = 0; i < 6; i++) {
                grasp.push_back(srv.response.grasp[i]);
            }
            // ROS_INFO_STREAM("X: " << grasp[0]);
            // ROS_INFO_STREAM("Y: " << grasp[1]);
            // ROS_INFO_STREAM("Z: " << grasp[2]);
            // ROS_INFO_STREAM("ROLL: " << grasp[3]);
            // ROS_INFO_STREAM("PITCH: " << grasp[4]);
            // ROS_INFO_STREAM("YAW: " << grasp[5]);
            found = true;
            tfBroadcaster();
            add_grasp(grasp);
            ros::param::set("/loaded", 1);
        } else {
            ROS_INFO_STREAM("Error getting grasp from service");
        }
    } else if (!loaded && found) {
        found = false;
    }
}

void Image2Model::tfBroadcaster() {
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // Broadcast to the transform frame the pose of the grasp
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "target_object";
    transformStamped.child_frame_id = "grasp";
    transformStamped.transform.translation.x = -grasp[0] - 0.2;
    transformStamped.transform.translation.y = grasp[1];
    transformStamped.transform.translation.z = grasp[2] + 0.03;

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