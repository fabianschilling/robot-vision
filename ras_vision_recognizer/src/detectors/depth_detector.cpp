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
#include <opencv2/photo/photo.hpp> // inpaint

// Constants
static const double MIN_RANGE = 0.0;
static const double MAX_RANGE = 1.0;
static const int PADX = 70;
static const int PADY = 20;
static const int EROSION = 20;
static const int DILATION = 1;
static const int MIN_AREA = 3000;
static const int MAX_AREA = 11000;
static const double MIN_ASPECT = 0.8;
static const double MAX_ASPECT = 1.2;
static const cv::Scalar color_black(0, 0, 0);
static const std::string DEPTH_WINDOW = "depth";
static const std::string THRESH_WINDOW = "thresh";

// Global variables
ros::Subscriber subscriber;
ros::Publisher publisher;

// Trackbar variables
int dilation = 0;
int erosion = 25;
int blockSize = 5;

std::vector<cv::Rect> filter_contours(std::vector<std::vector<cv::Point>>& contours) {

    std::vector<cv::Rect> rects;

    for (std::vector<cv::Point> contour: contours) {

        double area = std::fabs(cv::contourArea(cv::Mat(contour)));

        if (area > MIN_AREA && area < MAX_AREA) {

            cv::Rect rect = cv::boundingRect(contour);

            double aspect = (double) rect.height / (double) rect.width;

            if (aspect > MIN_ASPECT && aspect < MAX_ASPECT) {

                rects.push_back(rect);
            }
        }
    }

    return rects;
}

bool compare_contour_areas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2) {
    return std::fabs(cv::contourArea(cv::Mat(contour1))) < std::fabs(cv::contourArea(cv::Mat(contour2)));
}

void depth_callback(const sensor_msgs::Image::ConstPtr& message) {

    cv_bridge::CvImagePtr cv_image;

    try {
        cv_image = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Get mask of NaN values
    cv::Mat nan_mask = cv::Mat(cv_image->image != cv_image->image);

    // Map range from 0 to 255 based on MIN_RANGE and MAX_RANGE
    cv::Mat image(cv_image->image.rows, cv_image->image.cols, CV_8UC1);
    for (int i = 0; i < cv_image->image.rows; i++) {
        float* di = cv_image->image.ptr<float>(i);
        char* ii = image.ptr<char>(i);
        for (int j = 0; j < cv_image->image.cols; j++) {
            ii[j] = (char) (255 * ((di[j] - MIN_RANGE) / (MAX_RANGE - MIN_RANGE)));
        }
    }

    // Extract ROI without noise
    cv::Rect roi = cv::Rect(PADX, PADY, image.cols - (2 * PADX), image.rows - (2 * PADY));
    cv::Mat roi_image = image(roi);
    cv::Mat roi_nan_mask = nan_mask(roi);
    cv::rectangle(image, roi, color_black);
    //cv::imshow("nan", roi_nan_mask);

    // Inpainting
    cv::Mat roi_inpainted;
    cv::inpaint(roi_image, roi_nan_mask, roi_inpainted, -1, cv::INPAINT_TELEA);
    //cv::imshow("inpainted", roi_inpainted);

    // Adaptive Threshold (based on inpaint)
    cv::Mat thresh;
    int block_size = 3;
    double param1 = 2.0;
    cv::adaptiveThreshold(roi_inpainted, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, blockSize, param1);
    //cv::imshow("thresh", thresh);

    // Erosion and dilation
    if (erosion > 0) {
        cv::Mat erosion_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erosion, erosion));
        cv::erode(thresh, thresh, erosion_kernel);
    }

    if (dilation > 0) {
        cv::Mat dilation_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilation, dilation));
        cv::dilate(thresh, thresh, dilation_kernel);
    }

    // Find and draw contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(thresh, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    cv::drawContours(roi_inpainted, contours, -1, color_black);

    std::vector<cv::Rect> rects = filter_contours(contours);

    //std::cout << rects.size() << std::endl;

    for (cv::Rect rect: rects) {

        ras_vision_recognizer::Rect message;
        message.x = rect.x + PADX;
        message.y = rect.y + PADY;
        message.width = rect.width;
        message.height = rect.height;
        publisher.publish(message);

        // Draw a rectangle around the object
        cv::rectangle(roi_inpainted, rect, color_black);
    }

//    // Check if contours found
//    if (contours.size() > 1) {

//        // Sort contours by contour area
//        std::sort(contours.begin(), contours.end(), compare_contour_areas);

//        std::vector<cv::Point> object_contour = contours[contours.size() - 2];

//        cv::Rect rect = cv::boundingRect(cv::Mat(object_contour));

//        int size = rect.area();
//        double aspect = (double) rect.width / rect.height;

//        // Check if size and aspect ratio qualifies for object
//        if (size > MIN_AREA && size < MAX_AREA && aspect > MIN_ASPECT && aspect < MAX_ASPECT) {

//            // Draw a rectangle around the object
//            cv::rectangle(roi_inpainted, rect, color_black);

//            // Put some info text on the screen
//            std::ostringstream text;
//            text << "Object size: " << size << " px";
//            cv::putText(roi_inpainted, text.str(), cv::Point(rect.x + rect.width, rect.y), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, color_black, 1);

//            ras_vision_recognizer::Rect message;
//            message.x = rect.x;
//            message.y = rect.y;
//            message.width = rect.width;
//            message.height = rect.height;

//            publisher.publish(message);
//        }
//    }

    cv::imshow(DEPTH_WINDOW, roi_inpainted);
    cv::imshow(THRESH_WINDOW, thresh);

    if (int key = cv::waitKey(1) != -1) {
        std::cout << key << " pressed." << std::endl;
    }

}

int main(int argc, char ** argv) {

    ros::init(argc, argv, "depth_detector");

    ros::NodeHandle nh;

    cv::namedWindow(DEPTH_WINDOW, cv::WINDOW_NORMAL);
    cv::namedWindow(THRESH_WINDOW, cv::WINDOW_NORMAL);

    //cv::createTrackbar("dilation", THRESH_WINDOW, &dilation, 100);
    cv::createTrackbar("erosion", THRESH_WINDOW, &erosion, 100);
    //cv::createTrackbar("block size", THRESH_WINDOW, &blockSize, 30);


    subscriber = nh.subscribe<sensor_msgs::Image>("/camera/depth/image", 1, depth_callback);

    publisher = nh.advertise<ras_vision_recognizer::Rect>("/vision/object_rect", 1);

    ros::spin();

    return 0;
}
