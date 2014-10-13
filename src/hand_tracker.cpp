#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class HandTracker {
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    image_transport::Subscriber _depth_sub;
    image_transport::Publisher _depth_pub;

public:
    HandTracker() :
    _it(_nh)
    {
        _depth_sub = _it.subscribe("/camera/depth/image_raw", 1, &HandTracker::imageCb, this);
        _depth_pub = _it.advertise("/hand_follower/hand_tracker_filtered", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& img)
    {
        ROS_INFO_ONCE("encoding: %s", img->encoding.c_str());

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, "32FC1");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat resMat;
        try {
            cv::threshold(cv_ptr->image, resMat, 1000, 1, cv::THRESH_BINARY_INV);
        } catch (cv::Exception& e) {
            ROS_ERROR("cv_threshold error: %s", e.what());
        }

        cv_bridge::CvImage res;
        res.header = img->header;
        res.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        res.image = resMat;

        _depth_pub.publish(res.toImageMsg());

    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hand_tracker");
  HandTracker ht;
  ros::spin();
  return 0;
}
