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
#include <opencv2/features2d/features2d.hpp> // blobs

// Constants
static const double MIN_RANGE = 0.0;
static const double MAX_RANGE = 5.5;
static const int PADX = 70;
static const int PADY = 10;
static const int EROSION = 20;
static const int DILATION = 1;
static const int MIN_AREA = 500;
static const int MAX_AREA = 6000;
static const int MIN_DEBRIS_AREA = 7000;
static const int MAX_DEBRIS_AREA = 50000;
static const double MIN_ASPECT = 0.6;
static const double MAX_ASPECT = 1.4;
static const cv::Scalar colorBlack(0, 0, 0);
static const cv::Scalar colorWhite(255, 255, 255);
static const cv::Scalar colorRed(0, 0, 255);
static const cv::Scalar colorGreen(0, 255, 0);
static const cv::Scalar colorBlue(255, 0, 0);

// Window constants
static const std::string WIN_DEPTH = "depth";
static const std::string WIN_THRESH = "thresh";
static const std::string WIN_RANGE = "range";
static const std::string WIN_ROI = "roi";
static const std::string WIN_EDGE = "edge";
static const std::string WIN_NOISE = "no noise";
static const std::string WIN_OPEN = "opening";
static const std::string WIN_DETECT = "detection";

// Global variables
ros::Subscriber depthSubscriber;
ros::Subscriber colorSubscriber;
ros::Publisher publisher;

// Trackbar variables
int minRange = 20; //cm
int maxRange = 150; //cm
int uRoi = 110; //px
int dRoi = 15; //px
int lRoi = 50; //px
int rRoi = 65; //px
int lower = 25;
int upper = 50;
int kSize = 3;
int iterations = 10;
int thresh = 3;
int minSaturation = 130;

// Test
bool enableSubtraction = false;
cv::Mat backgroundImage;
cv::Mat colorImage;
static const std::string FILENAME = "/home/fabian/catkin_ws/src/ras_vision/background.jpg";

// Visualization
bool visualization = false;

cv::Rect convertRect(cv::Rect rect) {
    return cv::Rect(rect.x + lRoi, rect.y + uRoi, rect.width, rect.height);
}

cv::Mat normalize(cv::Mat image) {
    double minRangeD = minRange / 100.0;
    double maxRangeD = maxRange / 100.0;
    cv::Mat normalized(image.rows, image.cols, CV_8UC1);
    for (int i = 0; i < image.rows; i++) {
        float* di = image.ptr<float>(i);
        char* ii = normalized.ptr<char>(i);
        for (int j = 0; j < image.cols; j++) {
            if (di[j] > maxRangeD) {
                ii[j] = (char) 0;
            } else {
                ii[j] = (char) (255 * ((di[j] - minRangeD) / (maxRangeD - minRangeD)));
            }
        }
    }
    return normalized;
}

std::vector<std::vector<cv::Point> > filterContours(std::vector<std::vector<cv::Point> > contours) {

    std::vector<std::vector<cv::Point>> filtered;

    for (std::vector<cv::Point> contour: contours) {

        double area = cv::contourArea(contour);

        if (area > MIN_AREA && area < MAX_AREA) {

            cv::Rect rect = cv::boundingRect(contour);

            double aspectRatio = (double) rect.width / rect.height;

            if (aspectRatio > MIN_ASPECT && aspectRatio < MAX_ASPECT) {

                filtered.push_back(contour);
            }
        }
    }

    return filtered;
}

void colorCallback(const sensor_msgs::Image::ConstPtr& message) {

    colorImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8)->image;
}

void depthCallback(const sensor_msgs::Image::ConstPtr& message) {

    // Convert from ROS to OpenCV image
    cv::Mat depthImage = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::TYPE_32FC1)->image;

    // Crop ROI and visualize
    cv::Rect roi = cv::Rect(lRoi, uRoi, depthImage.cols - (lRoi + rRoi), depthImage.rows - (uRoi + dRoi));
    cv::Mat roiImage = depthImage(roi);
    cv::rectangle(depthImage, roi, colorWhite);
    if (visualization) cv::imshow(WIN_ROI, depthImage);

    // Get mask of NaN values
    cv::Mat nanMask = cv::Mat(roiImage != roiImage);

    // Normalize image
    cv::Mat rangeImage = normalize(roiImage);
    if (visualization) cv::imshow(WIN_RANGE, rangeImage);

    if (enableSubtraction) {

        // Background subtraction
        cv::Mat foregroundImage;
        cv::subtract(backgroundImage, rangeImage, foregroundImage);

        // Thresholding
        cv::Mat threshold;
        cv::threshold(foregroundImage, threshold, thresh, 255, cv::THRESH_BINARY);
        if (visualization) cv::imshow(WIN_THRESH, threshold);

        // Morphological opening (remove noise)
        cv::Mat opening;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kSize, kSize));
        cv::morphologyEx(threshold, opening, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), iterations);
        if (visualization) cv::imshow(WIN_OPEN, opening);

        // Find object, debris and wall contours
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(opening, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        for (std::vector<cv::Point> contour: contours) {

            double area = cv::contourArea(contour);

            // Possible object
            if (area > MIN_AREA && area < MAX_AREA) {

                cv::Rect rect = cv::boundingRect(contour);

                double aspectRatio = (double) rect.width / rect.height;

                if (aspectRatio > MIN_ASPECT && aspectRatio < MAX_ASPECT) {

                    cv::Rect realRect = convertRect(rect);

                    cv::Mat objectRoi = colorImage(realRect);

                    cv::Mat hsvImage;
                    cv::cvtColor(objectRoi, hsvImage, cv::COLOR_BGR2HSV);

                    //cv::imshow("possible object", hsvImage);

                    std::vector<cv::Mat> hsvPlanes;

                    cv::split(hsvImage, hsvPlanes);

                    //cv::Scalar saturation = cv::mean(hsvPlanes[1]);

                    double minSat;
                    double maxSat;
                    cv::minMaxLoc(hsvPlanes[1], &minSat, &maxSat);

                    if (maxSat > minSaturation) {

                        cv::rectangle(colorImage, realRect, colorGreen);

                    }
                }
            // Possible debris
            } else if (area > MIN_DEBRIS_AREA && area < MAX_DEBRIS_AREA) {

                cv::Rect debrisRect = cv::boundingRect(contour);

                if (debrisRect.x != 1 && debrisRect.y != 1) {
                    cv::Rect realRect = convertRect(debrisRect);
                    cv::rectangle(colorImage, realRect, colorRed);
                }
            // Wall
            } else {

            }
        }
    }

    cv::imshow(WIN_DETECT, colorImage);

    int key = cv::waitKey(60);

    if (key == 10) {
        std::cout << "Background image saved." << std::endl;
        enableSubtraction = true;
        cv::Mat inpaintImage;
        cv::inpaint(rangeImage, nanMask, inpaintImage, -1, cv::INPAINT_TELEA);
        backgroundImage = inpaintImage;
        cv::imwrite("/home/fabian/catkin_ws/src/ras_vision/background.jpg", backgroundImage);
    }

}

int main(int argc, char ** argv) {

    ros::init(argc, argv, "depth_detector");

    ros::NodeHandle nh;

    backgroundImage = cv::imread(FILENAME, CV_LOAD_IMAGE_GRAYSCALE);

    if (!backgroundImage.data) {
        std::cout << "Failed to load background image." << std::endl;
    } else {
        std::cout << "Background image loaded." << std::endl;
        enableSubtraction = true;
    }

    cv::namedWindow(WIN_ROI, cv::WINDOW_NORMAL);
    cv::createTrackbar("up", WIN_ROI, &uRoi, 200);
    cv::createTrackbar("down", WIN_ROI, &dRoi, 200);
    cv::createTrackbar("left", WIN_ROI, &lRoi, 200);
    cv::createTrackbar("right", WIN_ROI, &rRoi, 200);

    cv::namedWindow(WIN_RANGE, cv::WINDOW_NORMAL);
    cv::createTrackbar("min", WIN_RANGE, &minRange, 50);
    cv::createTrackbar("max", WIN_RANGE, &maxRange, 900);

    cv::namedWindow(WIN_OPEN, cv::WINDOW_NORMAL);
    cv::createTrackbar("kernel", WIN_OPEN, &kSize, 20);
    cv::createTrackbar("iternations", WIN_OPEN, &iterations, 20);

    cv::namedWindow(WIN_THRESH, cv::WINDOW_NORMAL);
    cv::createTrackbar("thresh", WIN_THRESH, &thresh, 20);

    cv::namedWindow(WIN_DETECT, cv::WINDOW_NORMAL);
    cv::createTrackbar("min saturation", WIN_DETECT, &minSaturation, 255);

    depthSubscriber = nh.subscribe<sensor_msgs::Image>("/camera/depth/image", 1, depthCallback);
    colorSubscriber = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 1, colorCallback);

    publisher = nh.advertise<ras_vision_recognizer::Rect>("/vision/object_rect", 1);

    ros::spin();

    return 0;
}