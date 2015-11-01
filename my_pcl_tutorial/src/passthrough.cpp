#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

ros::Publisher publisher;
ros::Subscriber subscriber;

void callback(const sensor_msgs::PointCloud2ConstPtr& input) {

    // Convert
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    pcl_conversions::toPCL(*input, *cloud);

    // Create filtering object
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(cloud_filtered);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    publisher.publish(output);

}

int main(int argc, char** argv) {

    std::cout << "Running..." << std::endl;

    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle handle;

    // Create a ROS subscriber for the input point cloud
    subscriber = handle.subscribe ("/camera/depth_registered/points", 1, callback);

    // Create a ROS publisher for the output point cloud
    publisher = handle.advertise<sensor_msgs::PointCloud2>("/pcl/passthrough", 1);

    // Spin
    ros::spin();
}
