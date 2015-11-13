// C++
#include <iostream>
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// ROS messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ras_vision_recognizer/Rect.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Constants
static const cv::Scalar color_black(0, 0, 0);
static const std::string IMAGE_WINDOW = "image";

// Global variables
ros::Subscriber imageSubscriber;
ros::Subscriber objectSubsriber;
ros::Publisher publisher;
cv::Mat image;

void colorCallback(const sensor_msgs::Image::ConstPtr& message) {

    cv_bridge::CvImagePtr cv_image;

    try {
        cv_image = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    image = cv_image->image;

}

void objectCallback(const ras_vision_recognizer::Rect::ConstPtr& message) {

    if (image.empty()) {
        return;
    }

    // Crop object ROI out of image
    cv::Rect rect;
    rect.x = message->x;
    rect.y = message->y;
    rect.width = message->width;
    rect.height = message->height;
    cv::Mat objectImage = image(rect);

    // Convert to HSV color space
    cv::Mat hsvImage;
    cv::cvtColor(objectImage, hsvImage, cv::COLOR_RGB2HSV);

    // Compute histogram
    cv::MatND hist;
    //cv::calcHist()

    cv::imshow(IMAGE_WINDOW, hsvImage);

    cv::waitKey(1);

    //std::cout << "object callback" << std::endl;
}

int main(int argc, char ** argv) {

    ros::init(argc, argv, "color_recognizer");

    ros::NodeHandle nh;

    cv::namedWindow(IMAGE_WINDOW, cv::WINDOW_NORMAL);

    imageSubscriber = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1, colorCallback);
    objectSubsriber = nh.subscribe<ras_vision_recognizer::Rect>("/vision/object_rect", 1, objectCallback);

    //publisher = nh.advertise<ras_vision_recognizer::Rect>("/vision/object_rect", 1);

    ros::spin();

    return 0;
}
