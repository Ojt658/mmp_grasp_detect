#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv_apps/MomentArrayStamped.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/Bool.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace Screen {
    int width = 400;
    int height = 400;
}

class ImageDepth {
    float x, y, depth;
    bool run;

    protected:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_; 
        image_transport::Subscriber image_sub_;
        ros::Subscriber momentSub_;
        ros::Subscriber commandSub_;
    
    public:
        ImageDepth() : it_(nh_) {
            run = false;
            x, y, depth = 0.0;

            image_sub_ = it_.subscribe("/head_camera/depth_registered/image_raw",
                                1, &ImageDepth::imageCb, this);

            momentSub_ = nh_.subscribe("/moment/moments", 1000,
                                &ImageDepth::momentCb, this);

            commandSub_ = nh_.subscribe("/get_model_pose", 1000,
                                        &ImageDepth::commandCb, this);

            
            // cv::namedWindow("Image window");
        }

        // ~DepthDetection() {
        //     cv::destroyWindow("Image window");
        // }

        void imageCb(const sensor_msgs::ImageConstPtr&);
        void momentCb(const opencv_apps::MomentArrayStamped&);
        void commandCb(const std_msgs::Bool&);


    private:
        void offsetCalc(float *offsets) { // Calculate the offsets for the transform
            double focal_length = 346.40918199449186; // look at camera info topic
            offsets[0] = - ((x - (Screen::width / 2)) * depth / focal_length);
            offsets[1] = - ((y - (Screen::height / 2)) * depth / focal_length);
        }

        void tfBroadcaster();

};

void ImageDepth::imageCb(const sensor_msgs::ImageConstPtr& msg) {
    if (run) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        depth = cv_ptr -> image.at<float>(y,x); // get the depth from the pixel
        ROS_INFO_STREAM(depth);
        tfBroadcaster();

        run = false;
    }
}

void ImageDepth::momentCb(const opencv_apps::MomentArrayStamped& msg) {
    if (msg.moments.size() > 0 && run) { // Found an object
        x = msg.moments[0].center.x;
        y = msg.moments[0].center.y;
        ROS_INFO_STREAM("X: " << x << "   Y: " << y);
    }
}

void ImageDepth::commandCb(const std_msgs::Bool& msg) {
    if (msg.data) {
        run = true;
    }
}

void ImageDepth::tfBroadcaster() {
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    float offsets[2];
    offsetCalc(offsets);

    // Broadcast to the transform frame the pose of the target object
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "head_camera_depth_frame";
    transformStamped.child_frame_id = "target_object";
    transformStamped.transform.translation.x = depth;
    transformStamped.transform.translation.y = offsets[0];
    transformStamped.transform.translation.z = offsets[1];

    tf2::Quaternion q;
    q.setRPY(0, -0.75, 0); // roll pitch yaw of end effector
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    
    br.sendTransform(transformStamped);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "depth_detection");

    ImageDepth dd;
    ros::Rate rate(5);
    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}