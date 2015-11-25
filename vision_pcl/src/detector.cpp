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

ros::Publisher cloudPublisher;
ros::Publisher pointPublisher;
ros::Publisher processedPublisher;
ros::Publisher histogramPublisher;
ros::Subscriber subscriber;

// These need to be adjusted every time the plane changes
double const a = 0.018485;
double const b = -0.846875;
double const c = -0.531485;
double const d = 0.257969;

static const double P[] = {-0.00974888, -0.877952, -0.478626, 0.255177};

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



vision_msgs::Point convert(Eigen::Vector3f pt) {

    vision_msgs::Point result;
    result.x = pt[0];
    result.y = pt[1];
    result.z = pt[2];

    return result;
}

Eigen::Vector3f project(Eigen::Vector4f p3d) {

    Eigen::Vector3f p2d;

    p2d[0] = (fx * p3d[0]/p3d[2] + cx);
    p2d[1] = (fy * p3d[1]/p3d[2] + cy);
    p2d[2] = p3d[2];

    return p2d;
}

vision_msgs::Points computeExtremePoints(Eigen::Vector4f min, Eigen::Vector4f max, Eigen::Vector4f ctr) {

    vision_msgs::Points result;

    // Calculate extreme values
    Eigen::Vector4f ul;
    ul[0] = min[0];
    ul[1] = min[1];
    ul[2] = ctr[2];

    Eigen::Vector4f ur;
    ur[0] = max[0];
    ur[1] = min[1];
    ur[2] = ctr[2];

    Eigen::Vector4f ll;
    ll[0] = min[0];
    ll[1] = max[1];
    ll[2] = ctr[2];

    Eigen::Vector4f lr;
    lr[0] = max[0];
    lr[1] = max[1];
    lr[2] = ctr[2];

    // Project extrema onto plane

    result.points.push_back(convert(project(ul)));
    result.points.push_back(convert(project(ur)));
    result.points.push_back(convert(project(ll)));
    result.points.push_back(convert(project(lr)));

    return result;
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
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(100);
    segmentation.setDistanceThreshold(0.02);

    //int numPoints = (int) input->points.size();
    int i = 1;
    // While still planes in the scene
    while (input->points.size() > 1000) {

        segmentation.setInputCloud(input);
        segmentation.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            //std::cout << "No planar model" << std::endl;
            break;
        } else if (inliers->indices.size() < 1000) {
            //std::cout << "Planar model too small" << std::endl;
            break;
        }

        //std::cout << "Iteration " << i << ": Removing plane of size: " << inliers->indices.size() << std::endl;

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

        i++;
    }

    return input;

}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> computeClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(input);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> extract;
    extract.setClusterTolerance(0.02); // 2cm
    extract.setMinClusterSize(20);
    extract.setMaxClusterSize(1000);
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

    // Transform back so the camera center is on the y-axis
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudUntransformed = untransform(cloudPassthroughY);

    // Filter out planes
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered = filterPlanes(cloudPassthroughY);

    //std::cout << "Cloud size: " << cloudFiltered->points.size() << std::endl;

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = computeClusters(cloudFiltered);

    std::cout << "Clusters found: " << clusters.size() << std::endl;

    for (int i = 0; i < clusters.size(); ++i) {

        //std::cout << "Cluster " << (i + 1) << std::endl;

        //std::cout << "Size:"  << clusters[i]->points.size() << std::endl;

        Eigen::Vector4f minPoint, maxPoint;
        pcl::getMinMax3D(*clusters[i], minPoint, maxPoint);
        //std::cout << "Min y: " << minPoint[1] << std::endl;
        //std::cout << "Max y: " << maxPoint[1] << std::endl;

        double width = std::abs(std::abs(minPoint[0]) - std::abs(maxPoint[0]));
        double height = std::abs(std::abs(minPoint[1]) - std::abs(maxPoint[1]));

        //std::cout << "Width: " << width << std::endl;
        //std::cout << "Height: " << height << std::endl;

        //Eigen::Vector4f centroid;
        //pcl::compute3DCentroid(*clusters[i], centroid);

        //std::cout << "Centroid y: " << centroid[1] << std::endl;

        if (minPoint[1] > -0.07 && minPoint[1] < -0.01) {

            pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsvCluster(new pcl::PointCloud<pcl::PointXYZHSV>);

            pcl::PointCloudXYZRGBtoXYZHSV(*clusters[i], *hsvCluster);

            //computeSaturation(hsvCluster);

            pcl::PointXYZHSV avg = getCloudAverage(hsvCluster);

            //std::cout << "(" << avg.h << ", " << avg.s << ", " << avg.v << ")" << std::endl;

            if (avg.s > minSaturation) {

                std::cout << "Object" << std::endl;

                // Reverse the transformation to get the pixel coordinates
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr untransformed = transform(clusters[i], transformation.inverse());

                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*untransformed, centroid);

                Eigen::Vector3f p2d = project(centroid);

                //std::cout << "Centroid: (" << x << ", " << y << ", " << z << ")" << std::endl;
                //std::cout << "Pixels: (" << px << ", " << py << ")" << std::endl;

                vision_msgs::Point point = convert(p2d);

                pointPublisher.publish(point);

                vision_msgs::Histogram histogram = computeHistogram(hsvCluster);

                histogramPublisher.publish(histogram);


                /*geometry_msgs::Point point;
                point.x = p2d[0];
                point.y = p2d[1];
                point.z = p2d[2];
                pointPublisher.publish(point);*/

                //clusterPublisher.publish(hsvCluster);

            }


        } else if (minPoint[1] < -0.07 && minPoint[1] > -0.15) {
            //std::cout << "Debris" << std::endl;
        } else {
            //std::cout << "Wall or noise?" << std::endl;
        }

        //cloudPublisher.publish(clusters[i]);
    }

    cloudPublisher.publish(cloudFiltered);
}

int main (int argc, char** argv) {

  ros::init (argc, argv, "voxel_grid");
  ros::NodeHandle nh;

  subscriber = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/camera/depth_registered/points", 1, cloudCallback);

  cloudPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/vision/filtered", 1);
  processedPublisher = nh.advertise<pcl::PointCloud<pcl::PointXYZHSV> >("/vision/processed", 1);
  pointPublisher = nh.advertise<vision_msgs::Point>("/vision/object_centroid", 1);
  histogramPublisher = nh.advertise<vision_msgs::Histogram>("vision/histogram", 1);

  ros::Rate r(100); // 10Hz

  while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
  }

}
