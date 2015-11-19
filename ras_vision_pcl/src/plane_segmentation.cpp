#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub;

std::vector<double> as;
std::vector<double> bs;
std::vector<double> cs;
std::vector<double> ds;


double mean(std::vector<double>& vec) {

    double sum = 0.0;

    for (int i = 0; i < vec.size(); ++i) {
        sum += vec[i];
    }

    return sum /  (double) vec.size();
}

void
cloudCallback (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud.makeShared ());
    seg.segment (inliers, coefficients);

    as.push_back(coefficients.values[0]);
    bs.push_back(coefficients.values[1]);
    cs.push_back(coefficients.values[2]);
    ds.push_back(coefficients.values[3]);

    double a = coefficients.values[0];
    double b = coefficients.values[1];
    double c = coefficients.values[2];
    double d = coefficients.values[3];

    std::cout << "Num points: " << as.size() << std::endl;

    std::cout << mean(as) << " " << mean(bs) << " " << mean(cs) << " " << mean(ds) <<  std::endl;

    // Publish the model coefficients
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(coefficients, ros_coefficients);
    pub.publish (ros_coefficients);
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "example");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloudCallback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl_msgs::ModelCoefficients> ("/coefficients", 1);

    // Spin
    ros::spin ();
}
