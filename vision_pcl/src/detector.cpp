// ROS & General
#include <ros/ros.h>
#include <vector>
#include <string>

// Messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vision_msgs/Detection.h>
#include <vision_msgs/Wall.h>

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
#include <pcl/filters/statistical_outlier_removal.h>

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

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Global subscriber & publisher variables
ros::Subscriber cloudSubscriber;
ros::Subscriber colorSubscriber;
ros::Publisher publisher;
ros::Publisher processedPublisher;
ros::Publisher visualizationPublisher;
ros::Publisher planePublisher;
ros::Publisher detectionPublisher;
ros::Publisher imagePublisher;
ros::Publisher wallPublisher;

// Global variables
cv::Mat colorImage;

// These need to be adjusted every time the plane changes
static const double P[] = {-0.00122599, -0.834404, -0.551152, 0.253691};

// Intrinsic camera parameters
static const double C[] = {574.0527954101562, 574.0527954101562, 319.5, 239.5};

double const leafSize = 0.005;
double const minZ = 0.0; // 0m
double const maxZ = 1.0; // 1m
double const minY = -0.30; // 30cm (may want the wall information too!
//double const minY = -0.15; // 15cm
double const maxY = -0.01; // 1cm

double const minPCLSaturation = 0.3;
double const minCVSaturation = 70.0;


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

    marker.lifetime = ros::Duration(0.5);

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
    marker.lifetime = ros::Duration(0.5);

    return marker;

}

vision_msgs::Histogram computeHistogram(pcl::PointCloud<pcl::PointXYZHSV>::Ptr input) {

    vision_msgs::Histogram result;

    std::vector<double> histogram(360);
    size_t size = input->points.size();

    for (size_t i = 0; i < size; ++i) {
            int index = input->points[i].h;
            histogram[index] += 1.0;
    }

    for (size_t i = 0; i < histogram.size(); ++i) {
        histogram[i] /= (double) size;
    }

    result.histogram = histogram;

    return result;
}

void flattenPlaneToLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane) {

    for (size_t i = 0; i < plane->points.size(); ++i) {
        plane->points[i].y = 0.0;
    }
}

double minDistanceToLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr line) {

    double minDistance = 1.0;
    for (size_t i = 0; i < line->points.size(); ++i) {
        pcl::PointXYZRGB point = line->points[i];
        if (point.x >= -0.05 && point.x <= 0.05 && point.z < minDistance) {
            minDistance = point.z;
        }
    }

    return minDistance;
}

Eigen::Vector3f project(Eigen::Vector4f p3d) {

    Eigen::Vector3f p2d;

    p2d[0] = (C[0] * p3d[0]/p3d[2] + C[2]);
    p2d[1] = (C[1] * p3d[1]/p3d[2] + C[3]);
    p2d[2] = p3d[2];

    return p2d;
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

    //pcl::VoxelGrid<pcl::PointXYZRGB> downsample;
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> downsample;
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPlanes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search(new pcl::search::KdTree<pcl::PointXYZRGB>);
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE); // remove planes perpendicular to ground
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(1000);
    segmentation.setDistanceThreshold(0.005);
    segmentation.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));
    segmentation.setEpsAngle(pcl::deg2rad(5.0)); // plane angle can be 5 degrees off
    segmentation.setProbability(0.99);

    visualization_msgs::MarkerArray planeMarkers;

    std::vector<double> distances;

    int planeSize = 400;
    int i = 0;

    // While still planes in the scene
    //while (input->points.size() > planeSize) {
    while (true) {

        // Search only in a radius of 1 cm
        search->setInputCloud(input);
        segmentation.setSamplesMaxDist(0.01, search);

        segmentation.setInputCloud(input);
        segmentation.segment(*inliers, *coefficients);

        if (inliers->indices.empty() || inliers->indices.size() < planeSize) {
            break;
        }

        //std::cout << "Iteration " << (i + 1) << ": Removing plane of size: " << inliers->indices.size() << std::endl;

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(input);
        extract.setIndices(inliers);
        extract.setNegative(false);

         // Get the points associated with the planar surface
        extract.filter(*plane);

        Eigen::Vector4f minXPoint, maxXPoint;
        pcl::getMinMax3D(*plane, minXPoint, maxXPoint);

        // Flatten plane into a line to get rid of height
        flattenPlaneToLine(plane);

        double distance = minDistanceToLine(plane);

        distances.push_back(distance);

        Eigen::Vector4f minPoint, maxPoint;
        pcl::getMinMax3D(*plane, minPoint, maxPoint);

        //std::cout << "Plane " << (i + 1) << " height: " << std::abs(minXPoint[1]) << std::endl;

        visualization_msgs::Marker planeMarker = getmarkerForPlane(minPoint, maxPoint, coefficients, i);
        planeMarkers.markers.push_back(planeMarker);
        i++;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*filtered);
        *input = *filtered;


    }

    //std::cout << "Publishing " << planeMarkers.markers.size() << " plane markers" << std::endl;

    // Get minimum distance to all walls in front
    double minDistance = 1.0;
    if (!distances.empty()) {
        std::vector<double>::iterator pos = std::min_element(distances.begin(), distances.end());
        minDistance = *pos;
    }

    vision_msgs::Wall wall;
    wall.distance = minDistance;
    wallPublisher.publish(wall);

    std::cout << "Front wall: " << minDistance << std::endl;

    planePublisher.publish(planeMarkers);

    return input;

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(input);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*output);

    return output;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(input);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> extract;
    extract.setClusterTolerance(0.04); // 4cm
    extract.setMinClusterSize(30);
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

vision_msgs::Detection getDetection(Eigen::Vector4f centroid, vision_msgs::Histogram histogram) {

    vision_msgs::Detection detection;
    vision_msgs::Centroid ctr;
    ctr.x = centroid[0];
    ctr.y = centroid[1];
    ctr.z = centroid[2];

    std::cout << ctr << std::endl;

    Eigen::Vector3f p2d = project(centroid);
    detection.centroid = ctr;
    vision_msgs::Box box;
    box.size = int(38.5 / p2d[2]);
    box.x = int(p2d[0] - (box.size / 2.0));
    box.y = int(p2d[1] - (box.size / 2.0));

    detection.box = box;

    detection.histogram = histogram;
    //detectionPublisher.publish(detection);

    return detection;
}

double getMeanSaturation(vision_msgs::Detection detection) {

    // Get the bounding box of the object
    vision_msgs::Box box = detection.box; 
    cv::Rect roi = cv::Rect(box.x, box.y, box.size, box.size);
    cv::Mat m = colorImage;

    // Check if bounding box is correct
    if (0 > roi.x || 0 > roi.width || roi.x + roi.width > m.cols || 0 > roi.y || 0 > roi.height || roi.y + roi.height > m.rows) {
        return 0.0;
    }

    // Crop out the object from the image, if not possible (invalid bbox) return false positive

    cv::Mat objectImage = colorImage(roi);

    // Convert to HSV color space
    cv::Mat hsvImage;
    cv::cvtColor(objectImage, hsvImage, cv::COLOR_BGR2HSV);

    // Split H, S and V
    std::vector<cv::Mat> hsvSplit;
    cv::split(hsvImage, hsvSplit);
    cv::Mat saturation = hsvSplit[1];

    double meanSaturation = cv::mean(saturation).val[0];

    //std::cout << "Saturation: " << meanSaturation << std::endl;

    if (meanSaturation > minCVSaturation) {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", objectImage).toImageMsg();
        imagePublisher.publish(msg);
    }

    return meanSaturation;

    /*if (meanSaturation < 70) {
        return true; // Is false positive
    } else {

        return false;
    }*/

    // It is a false positive if mean saturation is under 30
    //return (meanSaturation < 90);

    //std::cout << meanSaturation.val[0] << std::endl;

    //cv::imshow("detection", saturation);

    //cv::waitKey(1);
}

void colorCallback(const sensor_msgs::Image::ConstPtr& message) {

    colorImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8)->image;
}

void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& inputCloud) {

    // Filter out everything farther than 1m
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPassthroughZ = passthroughZ(inputCloud);
    if (cloudPassthroughZ->points.empty()) { return; }

    // Downsample cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudDownsampled = downsample(cloudPassthroughZ);
    if (cloudDownsampled->points.empty()) { return; }

    // Transform (rotate & translate) so the ground is on the y-axis
    Eigen::Matrix4f transformation = getTransformation();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudTransformed = transform(cloudDownsampled, transformation);
    if (cloudTransformed->points.empty()) { return; }

    // Filter out everything below 1cm (floor) and above debris (15cm)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPassthroughY = passthroughY(cloudTransformed);
    processedPublisher.publish(cloudPassthroughY);

    if (cloudPassthroughY->points.empty()) { return;}

    // Filter out planes and publish
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered = filterPlanes(cloudPassthroughY);

    if (cloudFiltered->points.empty()) { return;}

    // Remove outliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOutliers = removeOutliers(cloudFiltered);
    publisher.publish(cloudOutliers);

    // Detect clusters
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = extractClusters(cloudOutliers);

    visualization_msgs::MarkerArray objectMarkers;

    int detections = 0;

    for (int i = 0; i < clusters.size(); ++i) {

        //std::cout << "Cluster " << (i + 1) << " size: " << clusters[i]->points.size() << std::endl;

        // Get cluster extreme points in y direction
        Eigen::Vector4f minPoint, maxPoint;
        pcl::getMinMax3D(*clusters[i], minPoint, maxPoint);
        double minY = -maxPoint[1];
        double maxY = -minPoint[1];

        //std::cout << "(minY = " << minY << ", maxY = " << maxY << ")" << std::endl;

        // Conforms to object position?
        if (minY > 0.01 && minY < 0.02 && maxY > 0.02 && maxY < 0.055) {

            pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvCluster(new pcl::PointCloud<pcl::PointXYZHSV>);

            pcl::PointCloudXYZRGBtoXYZHSV(*clusters[i], *hsvCluster);

            double maxSaturation = getMaxSaturation(hsvCluster);

            //std::cout << "maxSaturation: " << maxSaturation << std::endl;

            // Check if we have enough saturation to be an object
            if (maxSaturation > minPCLSaturation) {

                detections++; // Increase number of detections

                // Reverse the transformation to get the pixel coordinates
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr untransformed = transform(clusters[i], transformation.inverse());

                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*untransformed, centroid);

                Eigen::Vector4f vizCentroid;
                pcl::compute3DCentroid(*clusters[i], vizCentroid);

                visualization_msgs::Marker objectMarker = getMarkerForObject(vizCentroid, i);
                vision_msgs::Histogram histogram = computeHistogram(hsvCluster);
                vision_msgs::Detection detection = getDetection(centroid, histogram);

                double meanCVSaturation = getMeanSaturation(detection);

                // Check if we REALLY have no wall part or anything
                if (meanCVSaturation > minCVSaturation) {
                    detectionPublisher.publish(detection);
                    objectMarkers.markers.push_back(objectMarker);
                    std::cout << "Detection: (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ") @ " << int(meanCVSaturation) << std::endl;
                }
            }
        }
    }

    visualizationPublisher.publish(objectMarkers);
}


int main (int argc, char** argv) {

  ros::init (argc, argv, "detector");
  ros::NodeHandle nh;

  cloudSubscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/depth_registered/points", 1, cloudCallback);
  colorSubscriber = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1, colorCallback);

  publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/vision/filtered", 1);
  processedPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZHSV> >("/vision/processed", 1);
  visualizationPublisher = nh.advertise<visualization_msgs::MarkerArray>("/vision/object_markers", 1);
  planePublisher = nh.advertise<visualization_msgs::MarkerArray>("/vision/plane_markers", 1);
  detectionPublisher = nh.advertise<vision_msgs::Detection>("/vision/detection", 1);
  imagePublisher = nh.advertise<sensor_msgs::Image>("/vision/detection_image", 1);
  wallPublisher = nh.advertise<vision_msgs::Wall>("vision/wall", 1);

  //ros::spin();

  ros::Rate r(5); // 10Hz

  while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
  }

}
