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
static const double MAX_RANGE = 5.5;
static const int PADX = 70;
static const int PADY = 10;
static const int EROSION = 20;
static const int DILATION = 1;
static const int MIN_AREA = 2000;
static const int MAX_AREA = 11000;
static const double MIN_ASPECT = 0.75;
static const double MAX_ASPECT = 1.25;
static const cv::Scalar colorBlack(0, 0, 0);
static const cv::Scalar colorWhite(255, 255, 255);

// Window constants
static const std::string DEPTH_WIN = "depth";
static const std::string THRESH_WIN = "thresh";
static const std::string RANGE_WIN = "range";
static const std::string ROI_WIN = "roi";
static const std::string EDGE_WIN = "edge";

// Global variables
ros::Subscriber subscriber;
ros::Publisher publisher;

// Trackbar variables
int dilation = 0;
int erosion = 20;
int blockSize = 3;
int minRange = 20; //cm
int maxRange = 100; //cm
int uRoi = 10; //px
int dRoi = 10; //px
int lRoi = 50; //px
int rRoi = 60; //px
int lower = 100;
int upper = 200;


std::vector<cv::Rect> filterContours(std::vector<std::vector<cv::Point>>& contours) {

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

std::vector<std::vector<cv::Point>> filterNanContours(std::vector<std::vector<cv::Point>>& nanContours) {

    std::vector<std::vector<cv::Point>> filteredContours;
    for (std::vector<cv::Point> contour: nanContours) {
        if (std::fabs(cv::contourArea(cv::Mat(contour))) > 200) {
            filteredContours.push_back(contour);
        }
    }

    return filteredContours;

}

bool compareContourAreas(std::vector<cv::Point> contour1, std::vector<cv::Point> contour2) {
    return std::fabs(cv::contourArea(cv::Mat(contour1))) < std::fabs(cv::contourArea(cv::Mat(contour2)));
}

void depthCallback(const sensor_msgs::Image::ConstPtr& message) {

    cv_bridge::CvImagePtr cvImage;

    try {
        cvImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //cv::imshow("original", cv_image->image);

    // Get mask of NaN values
    cv::Mat nanMask = cv::Mat(cvImage->image != cvImage->image);

    // Map range from 0 to 255 based on MIN_RANGE and MAX_RANGE
    double minRangeD = minRange / 100.0;
    double maxRangeD = maxRange / 100.0;
    cv::Mat image(cvImage->image.rows, cvImage->image.cols, CV_8UC1);
    for (int i = 0; i < cvImage->image.rows; i++) {
        float* di = cvImage->image.ptr<float>(i);
        char* ii = image.ptr<char>(i);
        for (int j = 0; j < cvImage->image.cols; j++) {
            if (di[j] > maxRangeD) {
                ii[j] = (char) 0;
            } else {
                ii[j] = (char) (255 * ((di[j] - minRangeD) / (maxRangeD - minRangeD)));
            }
        }
    }

    std::vector<std::vector<cv::Point> > nanContours;
    cv::imshow("NAN before", nanMask);
    cv::findContours(nanMask, nanContours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> filtered = filterNanContours(nanContours);
    cv::drawContours(nanMask, filtered, -1, colorBlack, -1);

    cv::imshow("NAN after", nanMask);
    cv::imshow(RANGE_WIN, image);

    // Extract ROI without noise
    cv::Rect roi = cv::Rect(lRoi, uRoi, image.cols - (lRoi + rRoi), image.rows - (uRoi + dRoi));
    cv::Mat roiImage = image(roi);
    cv::Mat roiNanMask = nanMask(roi);
    cv::rectangle(image, roi, colorBlack);
    //cv::imshow("nan", roi_nan_mask);

    cv::imshow(ROI_WIN, roiImage);

    // Inpainting
    cv::Mat roiInpainted;
    cv::inpaint(roiImage, roiNanMask, roiInpainted, -1, cv::INPAINT_TELEA);
    cv::blur(roiInpainted, roiInpainted, cv::Size(3, 3));
    cv::imshow("inpainted", roiInpainted);

    // Adaptive Threshold (based on inpaint)
    cv::Mat thresh;
    double param1 = 2.0;
    cv::adaptiveThreshold(roiInpainted, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, blockSize, param1);
    //cv::Canny(roiInpainted, thresh, lower, upper);
    cv::imshow(EDGE_WIN, thresh);

    // Erosion and dilation
    if (erosion > 0) {
        cv::Mat erosionKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erosion, erosion));
        cv::erode(thresh, thresh, erosionKernel);
    }

    if (dilation > 0) {
        cv::Mat dilationKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilation, dilation));
        cv::dilate(thresh, thresh, dilationKernel);
    }

    // Find and draw contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(thresh, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    cv::drawContours(roiInpainted, contours, -1, colorBlack);

    std::vector<cv::Rect> rects = filterContours(contours);

    //std::cout << rects.size() << std::endl;

    for (cv::Rect rect: rects) {

        ras_vision_recognizer::Rect message;
        message.x = rect.x + lRoi;
        message.y = rect.y + uRoi;
        message.width = rect.width;
        message.height = rect.height;
        publisher.publish(message);

        // Draw a rectangle around the object
        cv::rectangle(roiInpainted, rect, colorBlack);
    }

    cv::imshow(DEPTH_WIN, roiInpainted);
    //cv::imshow(THRESH_WINDOW, thresh);

    if (int key = cv::waitKey(1) != -1) {
        std::cout << key << " pressed." << std::endl;
    }

}

int main(int argc, char ** argv) {

    ros::init(argc, argv, "depth_detector");

    ros::NodeHandle nh;

    cv::namedWindow(RANGE_WIN, cv::WINDOW_NORMAL);
    cv::createTrackbar("min", RANGE_WIN, &minRange, 50);
    cv::createTrackbar("max", RANGE_WIN, &maxRange, 900);

    cv::namedWindow(ROI_WIN, cv::WINDOW_NORMAL);
    cv::createTrackbar("up", ROI_WIN, &uRoi, 100);
    cv::createTrackbar("down", ROI_WIN, &dRoi, 100);
    cv::createTrackbar("left", ROI_WIN, &lRoi, 100);
    cv::createTrackbar("right", ROI_WIN, &rRoi, 100);

    cv::namedWindow(DEPTH_WIN, cv::WINDOW_NORMAL);
    //cv::namedWindow(THRESH_WINDOW, cv::WINDOW_NORMAL);

    cv::namedWindow(EDGE_WIN, cv::WINDOW_NORMAL);
    cv::createTrackbar("lower", EDGE_WIN, &lower, 500);
    cv::createTrackbar("upper", EDGE_WIN, &upper, 500);

    //cv::createTrackbar("dilation", THRESH_WINDOW, &dilation, 100);
    //cv::createTrackbar("erosion", THRESH_WINDOW, &erosion, 100);
    //cv::createTrackbar("block size", THRESH_WINDOW, &blockSize, 30);


    subscriber = nh.subscribe<sensor_msgs::Image>("/camera/depth/image", 1, depthCallback);

    publisher = nh.advertise<ras_vision_recognizer::Rect>("/vision/object_rect", 1);

    ros::spin();

    return 0;
}
