// ROS
#include <ros/ros.h>

// Messages
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <vision_msgs/Rect.h>
#include <vision_msgs/Point.h>
#include <vision_msgs/Points.h>
#include <vision_msgs/Histogram.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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
#include <pcl/common/angles.h>

// PCL Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
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

// Global subscriber & publisher variables
ros::Publisher publisher;
ros::Publisher pointPublisher;
ros::Publisher processedPublisher;
ros::Publisher histogramPublisher;
ros::Publisher visualizationPublisher;
ros::Publisher planePublisher;
ros::Subscriber subscriber;

// Global PointCloud variables


// These need to be adjusted every time the plane changes
static const double P[] = {-0.00412679, -0.822544, -0.568684, 0.254984};

// Intrinsic camera parameters
static const double C[] = {574.0527954101562, 574.0527954101562, 319.5, 239.5};

double const leafSize = 0.005;
double const minZ = 0.0; // 0m
double const maxZ = 1.0; // 1m
double const minY = -0.30; // 30cm (may want the wall information too!
//double const minY = -0.15; // 15cm
double const maxY = -0.01; // 1cm

double const minSaturation = 0.4;

Eigen::Vector4f pclToRviz(Eigen::Vector4f pclPoint) {

    Eigen::Vector4f rvizPoint;

    //TODO: about 2.5 cm off
    rvizPoint[0] = pclPoint[2];
    rvizPoint[1] = -(pclPoint[0] + 0.025);
    rvizPoint[2] = -pclPoint[1];

    return rvizPoint;
}

visualization_msgs::Marker getmarkerForPlane(Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, pcl::ModelCoefficients::Ptr coeffs, int id) {

    visualization_msgs::Marker marker;

    // Depending on plane orientation, flip z values
    if ((coeffs->values[2] > 0 && coeffs->values[0] > 0) || (coeffs->values[2] < 0 && coeffs->values[0] < 0)) {
        std::swap(minPoint[2], maxPoint[2]);
    }

    Eigen::Vector4f rvizMinPoint = pclToRviz(minPoint);
    Eigen::Vector4f rvizMaxPoint = pclToRviz(maxPoint);

    marker.header.frame_id = "camera_depth_frame";
    marker.header.stamp = ros::Time::now();

    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.02; // 2cm
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.b = 0.0;
    marker.color.b = 0.0;

    marker.lifetime = ros::Duration(1.0);

    geometry_msgs::Point begin;
    begin.x = rvizMinPoint[0];
    begin.y = rvizMinPoint[1];
    begin.z = rvizMinPoint[2];
    marker.points.push_back(begin);

    geometry_msgs::Point end;
    end.x = rvizMaxPoint[0];
    end.y = rvizMaxPoint[1];
    end.z = rvizMaxPoint[2];
    marker.points.push_back(end);

    return marker;

}

visualization_msgs::Marker getMarkerForObject(Eigen::Vector4f point, int id) {

    visualization_msgs::Marker marker;

    Eigen::Vector4f rvizPoint = pclToRviz(point);

    marker.header.frame_id = "camera_depth_frame";
    marker.header.stamp = ros::Time::now();

    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = rvizPoint[0];
    marker.pose.position.y = rvizPoint[1];
    marker.pose.position.z = rvizPoint[2];
    marker.scale.x = 0.04; // 4cm
    marker.scale.y = 0.04; // 4cm
    marker.scale.z = 0.04; // 4cm
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(1.0);

    return marker;

}

vision_msgs::Point convertToPoint(Eigen::Vector3f pt) {

    vision_msgs::Point result;
    result.x = pt[0];
    result.y = pt[1];
    result.z = pt[2];

    return result;
}

void flattenPlaneToLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane) {

    for (size_t i = 0; i < plane->points.size(); ++i) {
        plane->points[i].y = 0.0;
    }
}

Eigen::Vector3f project(Eigen::Vector4f p3d) {

    Eigen::Vector3f p2d;

    p2d[0] = (C[0] * p3d[0]/p3d[2] + C[2]);
    p2d[1] = (C[1] * p3d[1]/p3d[2] + C[3]);
    p2d[2] = p3d[2];

    return p2d;
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

    //pcl::VoxelGrid<pcl::PointXYZRGB> downsample;
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> downsample;
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, Eigen::Matrix4f transformation) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::transformPointCloud(*input, *output, transformation);

    return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search(new pcl::search::KdTree<pcl::PointXYZRGB>);
    //search->setInputCloud(input);
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(100);
    segmentation.setDistanceThreshold(0.005); // 0.005 before
    segmentation.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
    segmentation.setEpsAngle(pcl::deg2rad(5.0));
    //segmentation.setRadiusLimits(0.0, 0.00001); // 0-5 cm
    //segmentation.setSamplesMaxDist(0.1, search);
    segmentation.setProbability(0.99);

    visualization_msgs::MarkerArray planeMarkers;

    int planeSize = 1500;
    int i = 0;
    // While still planes in the scene
    while (input->points.size() > planeSize) {

        segmentation.setInputCloud(input);
        segmentation.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0 || inliers->indices.size() < planeSize) {
            break;
        }

        std::cout << "Iteration " << (i + 1) << ": Removing plane of size: " << inliers->indices.size() << std::endl;

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        //extract.setKeepOrganized(true); // important for pixel conversion
        extract.setInputCloud(input);
        extract.setIndices(inliers);
        extract.setNegative(false);

         // Get the points associated with the planar surface
        extract.filter(*plane);

        // Flatten plane into a line to get rid of height
        flattenPlaneToLine(plane);

        Eigen::Vector4f minPoint, maxPoint;
        pcl::getMinMax3D(*plane, minPoint, maxPoint);

        visualization_msgs::Marker planeMarker = getmarkerForPlane(minPoint, maxPoint, coefficients, i);
        planeMarkers.markers.push_back(planeMarker);
        i++;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*filtered);
        *input = *filtered;


    }

    //std::cout << "Publishing " << planeMarkers.markers.size() << " plane markers" << std::endl;

    planePublisher.publish(planeMarkers);

    return input;

}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> computeClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(input);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> extract;
    extract.setClusterTolerance(0.04); // 2cm
    extract.setMinClusterSize(20);
    extract.setMaxClusterSize(400);
    extract.setSearchMethod(tree);
    extract.setInputCloud(input);
    extract.extract(clusterIndices);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clusters;

    for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>(*input, it->indices));

        clusters.push_back(cluster);
    }

    return clusters;
}

pcl::PointXYZHSV getCloudAverage(pcl::PointCloud<pcl::PointXYZHSV>::Ptr input) {

    pcl::PointXYZHSV avg;
    avg.x = 0; avg.y = 0; avg.z = 0; avg.h = 0; avg.s = 0; avg.v = 0;

    for(size_t i = 0; i < input->points.size(); i++) {
        if(!isnan(input->points[i].x) && !isnan(input->points[i].y) && !isnan(input->points[i].z)) {
            avg.x += input->points[i].x;
            avg.y += input->points[i].y;
            avg.z += input->points[i].z;
            avg.h += input->points[i].h;
            avg.s += input->points[i].s;
            avg.v += input->points[i].v;
        }
    }

    avg.x /= input->points.size();
    avg.y /= input->points.size();
    avg.z /= input->points.size();
    avg.h /= input->points.size();
    avg.s /= input->points.size();
    avg.v /= input->points.size();

    return avg;
}

double getMaxSaturation(pcl::PointCloud<pcl::PointXYZHSV>::Ptr input) {

    double maxSaturation = 0.0;
    double saturation = 0.0;
    for (size_t i = 0; i < input->points.size(); ++i) {
        saturation = input->points[i].s;
        if (saturation > maxSaturation) {
            maxSaturation = saturation;
        }
    }

    return maxSaturation;
}

Eigen::Matrix4f getTransformation() {

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    double theta = -std::acos(std::abs(P[1]) / (std::sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2])));
    transformation(1, 1) = std::cos(theta);
    transformation(1, 2) = -std::sin(theta);
    transformation(2, 1) = std::sin(theta);
    transformation(2, 2) = std::cos(theta);

    transformation(1, 3) = -P[3]; // translation on y axis
    //transformation(0, 3) = 0.05;  // translation on x axis

    return transformation;
}

vision_msgs::Histogram computeHistogram(pcl::PointCloud<pcl::PointXYZHSV>::Ptr input) {

    vision_msgs::Histogram result;

    std::vector<double> histogram(360);
    size_t size = input->points.size();

    for (size_t i = 0; i < size; ++i) {

        if (input->points[i].s > minSaturation && !isnan(input->points[i].x) && !isnan(input->points[i].y) && !isnan(input->points[i].z)) {
            int index = input->points[i].h;
            histogram[index] += 1.0;
        }
    }

    for (size_t i = 0; i < histogram.size(); ++i) {
        histogram[i] /= (double) size;
    }

    result.histogram = histogram;

    return result;
}

double getNanRatio(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input) {

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPassthroughZ = passThroughZ(inputCloud);

    // Downsample cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDownsampled = downsample(cloudPassthroughZ);

    // Transform (rotate & translate) so the ground is on the y-axis
    Eigen::Matrix4f transformation = getTransformation();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTransformed = transform(cloudDownsampled, transformation);

    // Filter out everything below 1cm (floor) and above debris (15cm)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPassthroughY = passThroughY(cloudTransformed);
    processedPublisher.publish(cloudPassthroughY);

    // Filter out planes and publish
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered = filterPlanes(cloudPassthroughY);
    publisher.publish(cloudFiltered);

    if (cloudFiltered->points.empty()) {
        return;
    }

    // Detect clusters
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = computeClusters(cloudFiltered);

    visualization_msgs::MarkerArray objectMarkers;

    for (int i = 0; i < clusters.size(); ++i) {

        std::cout << "Cluster " << (i + 1) << " size: " << clusters[i]->points.size() << std::endl;

        // Get cluster extreme points in y direction
        Eigen::Vector4f minPoint, maxPoint;
        pcl::getMinMax3D(*clusters[i], minPoint, maxPoint);
        double minY = -maxPoint[1];
        double maxY = -minPoint[1];

        std::cout << "(minY = " << minY << ", maxY = " << maxY << ")" << std::endl;

        // Conforms to object position?
        if (minY > 0.01 && minY < 0.02 && maxY > 0.02 && maxY < 0.055) {

            pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvCluster(new pcl::PointCloud<pcl::PointXYZHSV>);

            pcl::PointCloudXYZRGBtoXYZHSV(*clusters[i], *hsvCluster);

            double maxSaturation = getMaxSaturation(hsvCluster);

            std::cout << "maxSaturation: " << maxSaturation << std::endl;

            if (maxSaturation > minSaturation) {

                // Reverse the transformation to get the pixel coordinates
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr untransformed = transform(clusters[i], transformation.inverse());

                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*untransformed, centroid);

                Eigen::Vector4f vizCentroid;
                pcl::compute3DCentroid(*clusters[i], vizCentroid);

                visualization_msgs::Marker objectMarker = getMarkerForObject(vizCentroid, i);
                objectMarkers.markers.push_back(objectMarker);
                //visualizationPublisher.publish(objectMarker);

                Eigen::Vector3f p2d = project(centroid);

                vision_msgs::Point point = convertToPoint(p2d);

                pointPublisher.publish(point);

                vision_msgs::Histogram histogram = computeHistogram(hsvCluster);

                histogramPublisher.publish(histogram);

            }
        }
    }

    visualizationPublisher.publish(objectMarkers);
}

int main (int argc, char** argv) {

  ros::init (argc, argv, "detector");
  ros::NodeHandle nh;

  subscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/depth_registered/points", 1, cloudCallback);

  publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/vision/filtered", 1);
  processedPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZHSV> >("/vision/processed", 1);
  pointPublisher = nh.advertise<vision_msgs::Point>("/vision/object_centroid", 1);
  histogramPublisher = nh.advertise<vision_msgs::Histogram>("/vision/histogram", 1);
  visualizationPublisher = nh.advertise<visualization_msgs::MarkerArray>("/vision/object_markers", 1);
  planePublisher = nh.advertise<visualization_msgs::MarkerArray>("/vision/plane_markers", 1);

  //ros::spin();

  ros::Rate r(10); // 10Hz

  while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
  }

}
