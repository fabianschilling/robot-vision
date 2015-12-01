// ROS
#include <ros/ros.h>

// Messages
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <vision_msgs/Rect.h>
#include <vision_msgs/Point.h>
#include <vision_msgs/Points.h>
#include <vision_msgs/Histogram.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

// PCL Common
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

// PCL Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

// PCL KdTree
#include <pcl/kdtree/kdtree.h>

// PCL Sample Concensus
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

// PCL Segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// Eigen
#include <Eigen/Geometry>

ros::Publisher publisher;
ros::Subscriber cloudSubscriber;

static const double P[] = {-0.00324735, -0.88032, -0.474366, 0.290389};

// Intrinsic camera parameters
double const fx = 574.0527954101562;
double const fy = 574.0527954101562;
double const cx = 319.5;
double const cy = 239.5;

double const leafSize = 0.005;
double const minZ = 0.0; // 0m
double const maxZ = 1.0; // 1m
double const minY = -0.30; // 30cm (may want the wall information too!
//double const minY = -0.15; // 15cm
double const maxY = -0.01; // 1cm

double const minSaturation = 0.3;

Eigen::Matrix4f getTransformation() {

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    double theta = -std::acos(std::abs(P[1]) / (std::sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2])));
    transformation(1, 1) = std::cos(theta);
    transformation(1, 2) = -std::sin(theta);
    transformation(2, 1) = std::sin(theta);
    transformation(2, 2) = std::cos(theta);
    transformation(1, 3) = -P[3]; // 1 meter translation on y axis*/

    return transformation;
}

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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthroughY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PassThrough<pcl::PointXYZRGB> passthrough;
    passthrough.setInputCloud(input);
    passthrough.setFilterFieldName("y");
    passthrough.setFilterLimits(minY, maxY); // 1-15cm
    passthrough.filter(*output);

    return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, Eigen::Matrix4f transformation) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::transformPointCloud(*input, *output, transformation);

    return output;
}

void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& inputCloud) {

    // Filter out everything farther than 1m
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPassthroughZ = passthroughZ(inputCloud);

    // Downsample cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDownsampled = downsample(cloudPassthroughZ);

    // Transform (rotate & translate) so the ground is on the y-axis
    Eigen::Matrix4f transformation = getTransformation();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTransformed = transform(cloudDownsampled, transformation);

    // Filter out everything below 1cm (floor) and above debris (15cm)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPassthroughY = passthroughY(cloudTransformed);

    publisher.publish(cloudPassthroughY);
}

int main (int argc, char** argv) {

  ros::init (argc, argv, "voxel_grid");
  ros::NodeHandle nh;

  cloudSubscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/depth_registered/points", 1, cloudCallback);

  publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/vision/processed", 1);

  ros::Rate r(100); // 10Hz

  while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
  }

}
