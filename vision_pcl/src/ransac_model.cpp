#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Filters
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

// Sample Concensus
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

// Segmentation
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/common.h> // minMax3d
#include <pcl/kdtree/kdtree.h> // KdTree
#include <pcl/common/centroid.h>

ros::Publisher publisher;
ros::Subscriber subscriber;

const float fx = 574.0;
const float fy = 574.0;
const float cx = 319.5;
const float cy = 239.5;

void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& inputCloud) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> downsample;
    downsample.setInputCloud(inputCloud);
    downsample.setLeafSize(0.01f, 0.01f, 0.01f);
    downsample.filter(*cloudDownsampled);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZRGB>);
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_STICK);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(100);
    segmentation.setDistanceThreshold(0.02);

    int numPoints = (int) cloudDownsampled->points.size();

    // While still planes in the scene
    while (cloudDownsampled->points.size() > 0.1 * numPoints) {

        segmentation.setInputCloud(cloudDownsampled);
        segmentation.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            std::cout << "No planar model" << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        //extract.setKeepOrganized(true); // important for pixel conversion
        extract.setInputCloud(cloudDownsampled);
        extract.setIndices(inliers);
        extract.setNegative(false);

         // Get the points associated with the planar surface
        extract.filter(*cloudPlane);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloudFiltered);
        *cloudDownsampled = *cloudFiltered;
    }

    publisher.publish(cloudDownsampled);

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "detector");
    ros::NodeHandle nh;

    subscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 1, cloudCallback);
    publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/pointcloud", 1);


    ros::Rate r(1); //1Hz
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}
