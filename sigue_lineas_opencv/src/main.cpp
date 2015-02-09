#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/highgui.h>
#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include <sstream>
#include <sys/time.h>
#include <raspicam_cv.h>

using namespace cv;
using namespace std;

void Threshold();
void roi(int, int, int, int, int, int,int, int);

Point find_center_line(Point);
Point find_center_curve(Point);
Mat linea, curva;
cv::Mat camera;

int main(int argc, char **argv) {

	raspicam::RaspiCam_Cv Camera;
    Point centro_l;
    Point centro_c;

    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 );

    //ROS Initialization
    ros::init(argc, argv, "test_opencv");
    ros::NodeHandle n;
    ros::Publisher servo_msg = n.advertise<std_msgs::UInt16>("servo_msg", 10);
    ros::Publisher motor_msg = n.advertise<std_msgs::UInt16>("motor_msg", 10);
    ros::Rate loop_rate(10);
    std_msgs::UInt16 servo;
    std_msgs::UInt16 motor;

    if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}
    
    Camera.grab();
    Camera.retrieve ( camera);
    
    int rows = camera.rows;
    int cols = camera.cols;

    centro_l = Point(cols/2, rows-50);
    centro_c = Point(cols/2, 65);

    while (ros::ok()) {
        Camera.grab();
		Camera.retrieve ( camera);
        cvWaitKey(1);        

        roi(0, rows-100, cols, 100, 0, 0, cols, 130); // Region Of Interest

        Threshold();	// Apply treshold

        centro_l = find_center_line(centro_l); // Find center of line
        centro_c = find_center_curve(centro_c); // Find center of curve

        servo.data = centro_l.x;
        motor.data = centro_c.x;

        // Publish message for ROS communication
        servo_msg.publish(servo);
        motor_msg.publish(motor);

        ros::spinOnce();
        loop_rate.sleep();
    }

    Camera.release();
    return 0;
}


void roi(int x1, int y1, int h1, int w1, int x2, int y2, int h2, int w2) {
    linea = camera(Rect(x1, y1, h1, w1));
    curva = camera(Rect(x2, y2, h2, w2));
}

void Threshold() {
    int threshold_value = 100;
    int threshold_type = 1;
    int const max_BINARY_value = 255;

    // Convert image to grayscale
    cvtColor(linea, linea, CV_RGB2GRAY);
    cvtColor(curva, curva, CV_RGB2GRAY);

    // Apply binary threshold
    threshold(linea, linea, threshold_value, max_BINARY_value, threshold_type);
    threshold(curva, curva, threshold_value, max_BINARY_value, threshold_type);

}

Point find_center_line(Point centro_l) {
    vector<vector<Point> > contours;
    vector<Point> large_contour;
    vector<Point> convex_hull;
    vector<Vec4i> hierarchy;
    Point centro;
    Moments mo;
    RNG rng(12345);
    Scalar color;
    int max_area = 0;
    int area;

    // Remove noise
    erode(linea, linea, Mat());
    dilate(linea, linea, Mat());
    erode(linea, linea, Mat());
    dilate(linea, linea, Mat());

    // Find contours
    findContours(linea, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    // Find max area contours
    for (unsigned int i = 0; i < contours.size(); ++i) {
        area = (int)contourArea(contours[i]);
        if (area > max_area) {
            large_contour = contours[i];
            max_area = area;
        }
    }

    if (max_area == 0) {
        centro = centro_l;
    }
    else {
        // Simplify large contours
        approxPolyDP(Mat(large_contour), large_contour, 5, true);

        // Convex hull
        convexHull(large_contour, convex_hull);

        // Center of gravity
        mo = moments(convex_hull);
        centro = Point(mo.m10 / mo.m00, mo.m01 / mo.m00);
        if (centro.x < 0) {
            centro = centro_l;
        }
    }

    color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    circle(linea, centro, 4, color, -1, 8, 0);
    for (int i = 0; i< contours.size(); i++) {
        drawContours(linea, contours, i, color, 2, 8, hierarchy, 0, Point());
    }

    //imshow("test", linea);
    //cvWaitKey(1);

    return centro;
}

Point find_center_curve(Point centro_c) {
    vector<vector<Point> > contours;
    vector<Point> large_contour;
    vector<Point> convex_hull;
    vector<Vec4i> hierarchy;
    Point centro;
    Moments mo;
    RNG rng(12345);
    Scalar color;
    int area;
    int max_area = 0;

    // Remove noise
    erode(curva, curva, Mat());
    dilate(curva, curva, Mat());
    erode(curva, curva, Mat());
    dilate(curva, curva, Mat());

    // Find contours
    findContours(curva, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    // Find max area contours
    for (unsigned int i = 0; i < contours.size(); ++i) {
        area = (int)contourArea(contours[i]);
        if (area > max_area) {
            large_contour = contours[i];
            max_area = area;
        }
    }

    if (max_area == 0) {
        centro = centro_c;
    }
    else {
        // Simplify large contours
        approxPolyDP(Mat(large_contour), large_contour, 5, true);

        // Convex hull
        convexHull(large_contour, convex_hull, false);

        // Center of gravity
        mo = moments(convex_hull);
        centro = Point(mo.m10 / mo.m00, mo.m01 / mo.m00);

        if (centro.x < 0) {
            centro = centro_c;
        }
    }
    color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    circle(curva, centro, 4, color, -1, 8, 0);
    for (int i = 0; i< contours.size(); i++) {
        drawContours(curva, contours, i, color, 2, 8, hierarchy, 0, Point());
    }

    return centro;
}

