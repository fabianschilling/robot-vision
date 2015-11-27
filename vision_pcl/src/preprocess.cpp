// ROS
#include <ros/ros.h>

// Messages
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

// PCL Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

ros::Publisher publisher;
ros::Subscriber subscriber;

double const leafSize = 0.005;
double const minZ = 0.0; // 0m
double const maxZ = 1.0; // 1m

pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthroughZ(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PassThrough<pcl::PointXYZRGB> passthrough;
    passthrough.setInputCloud(input);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(minZ, maxZ); // 0-1m
    passthrough.filter(*output);

    return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> downsample;
    downsample.setInputCloud(input);
    downsample.setLeafSize(leafSize, leafSize, leafSize);
    downsample.filter(*output);

    return output;
}

double getNanRatio(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    double nanSum = 0.0;

    for(size_t i = 0; i < input->points.size(); i++) {
        if(isnan(input->points[i].x) || isnan(input->points[i].y) || isnan(input->points[i].z)) {
            nanSum += 1.0;
        }
    }

    return nanSum;

}


void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& inputCloud) {

    // Filter out everything farther than 1m
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPassthroughZ = passthroughZ(inputCloud);

    // Downsample cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDownsampled = downsample(cloudPassthroughZ);

    double nanRatio = getNanRatio(cloudDownsampled);

    std::cout << "Nan sum: " << nanRatio << std::endl;

    // Downsampling calculations

    /*double sizeBefore = (double) inputCloud->points.size();
    double sizeAfter = (double) cloudDownsampled->points.size();
    int left = (int) (100.0 * (sizeAfter / sizeBefore));
    std::cout << left << "% remaining after preprocessing." << std::endl;*/

    publisher.publish(cloudDownsampled);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "preprocess");
  ros::NodeHandle nh;

  subscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/depth_registered/points", 1, cloudCallback);
  publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/vision/preprocessed", 1);

  ros::spin();

  /*ros::Rate r(10); // 10Hz

  while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
  }*/

}
