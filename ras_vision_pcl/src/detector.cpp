// ROS
#include <ros/ros.h>

// Messages
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

ros::Publisher cloudPublisher;
ros::Subscriber subscriber;

// These need to be adjusted every time the plane changes
double const a = 0.00477479;
double const b = -0.878083;
double const c = -0.478483;
double const d = 0.256502;

// Intrinsic camera parameters
double const fx = 574.0;
double const fy = 574.0;
double const cx = 319.5;
double const cy = 239.5;

double const leafSize = 0.005;
double const minZ = 0.0; // 0m
double const maxZ = 1.0; // 1m
double const minY = -0.30; // 30cm (may want the wall information too!
//double const minY = -0.15; // 15cm
double const maxY = -0.01; // 1cm

double distanceToPlane(Eigen::Vector4f centroid) {

    double x = centroid[0];
    double y = centroid[1];
    double z = centroid[2];

    double distTop = std::abs(a * x + b * y + c * z + d);
    double distBot = std::sqrt(a * a + b * b + c * c);

    return distTop / distBot;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThroughZ(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input) {

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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr passThroughY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PassThrough<pcl::PointXYZRGB> passthrough;
    passthrough.setInputCloud(input);
    passthrough.setFilterFieldName("y");
    passthrough.setFilterLimits(minY, maxY); // 1-15cm
    passthrough.filter(*output);

    return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Rotation around x-axis
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    double theta = -std::acos(std::abs(b) / (std::sqrt(a * a + b * b + c * c)));
    transformation(1, 1) = std::cos(theta);
    transformation(1, 2) = -std::sin(theta);
    transformation(2, 1) = std::sin(theta);
    transformation(2, 2) = std::cos(theta);

    // Translation on y-axis
    transformation(1, 3) = -d; // 1 meter translation on y axis

    pcl::transformPointCloud(*input, *output, transformation);

    return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr untransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Rotation around x-axis
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    double theta = std::acos(std::abs(b) / (std::sqrt(a * a + b * b + c * c)));
    transformation(1, 1) = std::cos(theta);
    transformation(1, 2) = -std::sin(theta);
    transformation(2, 1) = std::sin(theta);
    transformation(2, 2) = std::cos(theta);

    // Translation on y-axis
    transformation(1, 3) = d; // 1 meter translation on y axis

    pcl::transformPointCloud(*input, *output, transformation);

    return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(100);
    segmentation.setDistanceThreshold(0.02);

    int numPoints = (int) input->points.size();

    // While still planes in the scene
    while (input->points.size() > 0.3 * numPoints) {

        segmentation.setInputCloud(input);
        segmentation.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            std::cout << "No planar model" << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        //extract.setKeepOrganized(true); // important for pixel conversion
        extract.setInputCloud(input);
        extract.setIndices(inliers);
        extract.setNegative(false);

         // Get the points associated with the planar surface
        extract.filter(*plane);

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*filtered);
        *input = *filtered;
    }

    return input;

}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> computeClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(input);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> extract;
    extract.setClusterTolerance(0.04); // 4cm
    extract.setMinClusterSize(50);
    extract.setMaxClusterSize(1000);
    extract.setSearchMethod(tree);
    extract.setInputCloud(input);
    extract.extract(clusterIndices);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clusters;

    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cluster->points.push_back(input->points[*pit]);
        }

        clusters.push_back(cluster);
    }

    return clusters;
}

void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& inputCloud) {

    // Filter out everything farther than 1m
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPassthroughZ = passThroughZ(inputCloud);

    // Downsample cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDownsampled = downsample(cloudPassthroughZ);

    // Transform (rotate & translate) so the ground is on the y-axis
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTransformed = transform(cloudDownsampled);

    // Filter out everything below 1cm (floor) and above debris (15cm)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPassthroughY = passThroughY(cloudTransformed);

    // Transform back so the camera center is on the y-axis
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudUntransformed = untransform(cloudPassthroughY);

    // Filter out planes
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered = filterPlanes(cloudPassthroughY);

    //std::cout << "Cloud size: " << cloudFiltered->points.size() << std::endl;

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = computeClusters(cloudPassthroughY);

    std::cout << "Clusters found: " << clusters.size() << std::endl;

    for (int i = 0; i < clusters.size(); ++i) {

        std::cout << "Cluster " << (i + 1) << std::endl;

        std::cout << "Size:"  << clusters[i]->points.size() << std::endl;

        Eigen::Vector4f minPoint, maxPoint;
        pcl::getMinMax3D(*clusters[i], minPoint, maxPoint);
        std::cout << "Min y: " << minPoint[1] << std::endl;
        std::cout << "Max y: " << maxPoint[1] << std::endl;

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*clusters[i], centroid);

        std::cout << "Centroid y: " << centroid[1] << std::endl;

        if (minPoint[1] > -0.07 && minPoint[1] < -0.03) {
            std::cout << "Object" << std::endl;
        } else if (minPoint[1] < -0.07 && minPoint[1] > -0.15) {
            std::cout << "Debris" << std::endl;
        } else {
            std::cout << "Wall or noise?" << std::endl;
        }

        //cloudPublisher.publish(clusters[i]);
    }

    cloudPublisher.publish(cloudPassthroughY);
}

int main (int argc, char** argv) {

  ros::init (argc, argv, "voxel_grid");
  ros::NodeHandle nh;

  subscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/depth_registered/points", 1, cloudCallback);
  cloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/voxel_grid", 1);

  ros::Rate r(1); // 10Hz

  while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
  }

}
