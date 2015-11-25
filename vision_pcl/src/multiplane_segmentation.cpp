#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>

ros::Publisher publisher;
ros::Subscriber subscriber;

void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& inputCloud) {

    // Normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(inputCloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03); // 3cm
    ne.compute (*cloudNormals);

    // Segment planes


    //publisher.publish(voxelCloud);
}

int main (int argc, char** argv) {

  ros::init (argc, argv, "voxel_grid");
  ros::NodeHandle nh;

  subscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 1, cloudCallback);
  publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/segmentation", 1);

  ros::Rate r(10); // 10Hz

  while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
  }

}
