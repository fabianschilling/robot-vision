#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher publisher;
ros::Subscriber subscriber;

void callback(const sensor_msgs::PointCloud2ConstPtr& input) {

      // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg (*input, cloud);

      pcl::ModelCoefficients coefficients;
      pcl::PointIndices inliers;
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);

      seg.setInputCloud (cloud.makeShared ());
      seg.segment (inliers, coefficients);

      // Publish the model coefficients
      pcl_msgs::ModelCoefficients ros_coefficients;
      pcl_conversions::fromPCL(coefficients, ros_coefficients);
      publisher.publish (ros_coefficients);
}

int main(int argc, char** argv) {

    std::cout << "Running..." << std::endl;

    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle handle;

    // Create a ROS subscriber for the input point cloud
    subscriber = handle.subscribe ("/camera/depth/points", 1, callback);

    // Create a ROS publisher for the output point cloud
    publisher = handle.advertise<pcl_msgs::ModelCoefficients>("/pcl/coefficients", 1);

    // Spin
    ros::spin();
}
