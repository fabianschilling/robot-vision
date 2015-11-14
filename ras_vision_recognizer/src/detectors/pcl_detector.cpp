#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher publisher;
ros::Subscriber subscriber;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input) {

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (temp_cloud);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "pcl_detector");
    ros::NodeHandle nh;

    subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, cloudCallback);

    publisher = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

    ros::spin();
}
