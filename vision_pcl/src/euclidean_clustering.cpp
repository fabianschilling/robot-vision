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
ros::Publisher pointPublisher;
ros::Subscriber cloudSubscriber;

const float fx = 574.0;
const float fy = 574.0;
const float cx = 319.5;
const float cy = 239.5;

const double a = 0.0099;
const double b = -0.8687;
const double c = -0.4952;
const double d = 0.2582;

double distanceToPlane(Eigen::Vector4f centroid) {
    double x = centroid[0];
    double y = centroid[1];
    double z = centroid[2];

    double distTop = std::abs(a * x + b * y + c * z + d);
    double distBot = std::sqrt(a * a + b * b + c * c);

    return distTop / distBot;
}

void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& inputCloud) {

    // Filter cloud based on z-value
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPassthrough(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> passthrough;
    passthrough.setInputCloud(inputCloud);
    passthrough.setFilterFieldName("z");
    passthrough.setFilterLimits(0.0, 1.0); // 0-1m
    passthrough.filter(*cloudPassthrough);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDownsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> downsample;
    downsample.setInputCloud(cloudPassthrough);
    downsample.setLeafSize(0.01f, 0.01f, 0.01f);
    downsample.filter(*cloudDownsampled);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZRGB>);
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
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

    if (cloudDownsampled->empty()) {
        return;
    }

    publisher.publish(cloudDownsampled);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloudDownsampled);

    std::vector<pcl::PointIndices> cloudIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> extract;
    extract.setClusterTolerance(0.04); // 4cm
    extract.setMinClusterSize(10);
    extract.setMaxClusterSize(200);
    extract.setSearchMethod(tree);
    extract.setInputCloud(cloudDownsampled);
    extract.extract(cloudIndices);

    if (!cloudIndices.empty()) {
        std::cout << "Clusters found: " << cloudIndices.size() << std::endl;
    }

    for (std::vector<pcl::PointIndices>::const_iterator it = cloudIndices.begin (); it != cloudIndices.end(); ++it) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            cluster->points.push_back(cloudDownsampled->points[*pit]);
        }

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);


        double distanceToGround = distanceToPlane(centroid);

        if (distanceToGround > 0.05) {
            continue;
        }

        std::cout << "Points: " << cluster->points.size() << std::endl;

        std::cout << "Distance to ground: " << distanceToPlane(centroid) << std::endl;

        float x = centroid[0];
        float y = centroid[1];
        float z = centroid[2];

        int px = (int) (fx * x/z + cx);
        int py = (int) (fy * y/z + cy);

        std::cout << "Centroid: (" << x << ", " << y << ", " << z << ")" << std::endl;
        std::cout << "Pixels: (" << px << ", " << py << ")" << std::endl;

        geometry_msgs::Point point;
        point.x = px;
        point.y = py;
        point.z = z;
        pointPublisher.publish(point);

    }

    /*//std::cout << "After: " << cloudPassthrough->size() << std::endl;
    Point min;
    Point max;
    pcl::getMinMax3D(*cloudPassthrough, min, max);

    std::cout << "Min: " << min << std::endl;
    std::cout << "Max: " << max << std::endl;*/

    /**/

    /*boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(inputCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (inputCloud, rgb, "sample cloud");*/

    /*// Remove statistical outliers
    PointCloud::Ptr cloudDenoised(new PointCloud);
    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud(cloudFiltered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloudDenoised);*/



}

int main(int argc, char** argv) {

    ros::init(argc, argv, "detector");
    ros::NodeHandle nh;

    cloudSubscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth_registered/points", 1, cloudCallback);
    publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/pointcloud", 1);
    pointPublisher = nh.advertise<geometry_msgs::Point>("/vision/object_rect", 1);


    ros::Rate r(1); //1Hz
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}
