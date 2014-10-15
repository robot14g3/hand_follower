#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>

static ros::Publisher pcl_pub;

void imageCb(const sensor_msgs::PointCloud2ConstPtr& pcl_msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 tmp_pcl;
    pcl_conversions::toPCL(*pcl_msg, tmp_pcl);
    pcl::fromPCLPointCloud2(tmp_pcl, *input);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter(*output);

    ROS_INFO("input size:%d output size:%d", input->size(), output->size());

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*output, msg_out);
    pcl_pub.publish(msg_out);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hand_tracker");
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, imageCb);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/hand_tracker/pcl_filtered", 1);
    ros::spin();
    return 0;
}
