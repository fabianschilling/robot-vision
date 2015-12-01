#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

ros::Publisher publisher;
ros::Subscriber cloudSubscriber;

void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& inputCloud) {

    size_t pointsBefore = inputCloud->points.size();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> downsample;
    downsample.setInputCloud(inputCloud);
    //downsample.setLeafSize(0.1f, 0.1f, 0.1f);
    downsample.setLeafSize(0.01f, 0.01f, 0.01f);
    downsample.filter(*voxelCloud);

    size_t pointsAfter = voxelCloud->points.size();

    double pointsLeft = (100.0f * pointsAfter / pointsBefore);

    std::cout << "Downsampled: " << pointsBefore << " -> " << pointsAfter << " (" << pointsLeft << "% left)" << std::endl;

    publisher.publish(voxelCloud);
}

int main (int argc, char** argv) {

  ros::init (argc, argv, "voxel_grid");
  ros::NodeHandle nh;

  cloudSubscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 1, cloudCallback);
  publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/voxel_grid", 1);

  ros::Rate r(10); // 10Hz

  while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
  }

}
