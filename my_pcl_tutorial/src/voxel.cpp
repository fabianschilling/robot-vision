#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

ros::Publisher publisher;
ros::Subscriber subscriber;

void callback(const sensor_msgs::PointCloud2ConstPtr& input) {

    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*input, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(cloud_filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    // Publish the data
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
    publisher = handle.advertise<sensor_msgs::PointCloud2>("/pcl/voxel", 1);

    // Spin
    ros::spin();
}
