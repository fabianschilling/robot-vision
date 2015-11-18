#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher cloudPublisher;
ros::Subscriber subscriber;

void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& inputCloud) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotatedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Rotation around z-axis
    /*Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    float theta = M_PI / 8; // 45°
    transformation(0, 0) = cos(theta);
    transformation(0, 1) = -sin(theta);
    transformation(1, 0) = sin(theta);
    transformation(1, 1) = cos(theta);*/

    // Rotation around x-axis
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    float theta = -M_PI / 8; // 45°
    transformation(1, 1) = cos(theta);
    transformation(1, 2) = -sin(theta);
    transformation(2, 1) = sin(theta);
    transformation(2, 2) = cos(theta);

    pcl::transformPointCloud(*inputCloud, *rotatedCloud, transformation);

    cloudPublisher.publish(rotatedCloud);
}

int main (int argc, char** argv) {

  ros::init (argc, argv, "voxel_grid");
  ros::NodeHandle nh;

  subscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 1, cloudCallback);
  cloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/voxel_grid", 1);

  ros::Rate r(10); // 10Hz

  while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
  }

}
