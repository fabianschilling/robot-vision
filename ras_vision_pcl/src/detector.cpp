#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>

ros::Publisher publisher;
ros::Subscriber subscriber;

void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud) {

    //sensor_msgs::PointCloud2 output = *input;
    publisher.publish(cloud);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "detector");
    ros::NodeHandle nh;

    subscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 1, cloudCallback);
    publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/pointcloud", 1);

    ros::spin();
}
