#ifndef LADYBUG2_H
#define LADYBUG2_H


// std includes
#include <iostream>
#include <string>
#include <fstream>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse_yml.h>

// Eigen includes
#include <Eigen/Eigen>

// opencv includes
#include <opencv2/opencv.hpp>
#include <highgui.h>

// Ladybug includes
#include <ladybug2/rectificationMaps.h>


#define NUM_CAMERAS 6           /*!< Total number of cameras */
#define NUM_OMNI_CAMERAS 5      /*!< Number of cameras in the omnidirectional ring */
#define IMAGE_WIDTH 768         /*!< Image width of each individual image */
#define IMAGE_HEIGHT 1024       /*!< Image height of each individual image */

class Ladybug2
{
    public:
        
        /** Default constructor */
        Ladybug2();

        /** Constructor with arguments 
         * @param ros::NodeHandle ROS node */
        Ladybug2(ros::NodeHandle node);

        /** Destructor */
        ~Ladybug2();

        /** Split Ladybug image.
         * @param cv::Mat Ladybug image, with all individual images stacked
         * @param std::string color code - COLOR or MONO
         * @return std::vector<cv::Mat> vector with the splitted images */
        std::vector<cv::Mat> splitLadybug(cv::Mat ladybugImage, std::string colorCode);

        /** Get the neighbouring left and right camera indices of a specified camera. Only the 5 ring-arranged cameras are considered.
         * @param int index of the camera 
         * @param int& output index of the left neighbouring camera
         * @param int& output index of the right neighbouring camera
         * @return void */
        void getLeftRightCameras(int cam, int &camLeft, int &camRight);

        /** Undistort images according to rectification maps.
         * @param std::vector<cv::Mat> vector with distorted images
         * @return std::vector<cv::Mat> vector with undistorted (rectified) images */
        std::vector<cv::Mat> undistort(std::vector<cv::Mat> images);

        /** Rectify images with manually obtained calibration.
         * @param std::vector<cv::Mat> vector with distorted images
         * @return std::vector<cv::Mat> vector with rectified images */
        std::vector<cv::Mat> rectifyManually(std::vector<cv::Mat> images);

        /** Convert vector with CameraInfo messages.
         * @param void
         * @return std::vector<sensor_msgs::CameraInfo> vector with CameraInfo messages */
        std::vector<sensor_msgs::CameraInfo> getCameraInfoMsg();

        /** Get intrinsic parameters from manually obtained .yaml files.
         * @param void
         * @return void */
        void getIntrinsics();

        /** Get extrinsic parameters.
         * @param void
         * @return void */
        void getExtrinsics();

        std::vector<image_geometry::PinholeCameraModel> intrinsics_;	/*!< Vector with intrinsic calibration data of each camera */
        std::vector<sensor_msgs::CameraInfo> cameraInfos_;	            /*!< Vector with CameraInfo messages of each camera */
        std::vector<Eigen::Matrix4f> extrinsics_;		                /*!< Vector with extrinsic parameters of the cameras */

    private:

        int cameraPositions_[6] = {5, 4, 3, 2, 1, 0};                   /*!< Conventional camera order =/= order of the stacked images in the full Ladybug image */

        std::vector<std::string> param_intrinsicsPaths_;                /*!< Paths to the files with intrinsic parameters */    
        std::vector<std::vector<double>> param_extrinsicsData_;         /*!< Vector with extrinsics data of each camera */  
};

#endif