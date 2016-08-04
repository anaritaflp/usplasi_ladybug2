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

        /** Rectify images with calibration data.
         * @param std::vector<cv::Mat> vector with distorted images
         * @return std::vector<cv::Mat> vector with rectified images */
        std::vector<cv::Mat> rectify(std::vector<cv::Mat> images);

        /** Get intrinsic parameters from manually obtained .yaml files.
         * @param ros::NodeHandle ROS node for reading parameters
         * @return void */
        void getIntrinsics(ros::NodeHandle node);

        /** Get extrinsic parameters.
         * @param ros::NodeHandle ROS node for reading parameters
         * @return void */
        void getExtrinsics(ros::NodeHandle node);

        /** Convert transform in local (camera) coordinates to global (ladybug) coordinates.
		 * @param Eigen::Matrix4f transform in local coordinates
		 * @param int index of the previous camera
		 * @param int index of the current camera
		 * @return Eigen::Matrix4f transform in global coordinates */
		Eigen::Matrix4f cam2LadybugRef(Eigen::Matrix4f TLocal, int camNoPrev, int camNoCurr);

		/** Convert transform in global (ladybug) coordinates to local (camera) coordinates.
		 * @param Eigen::Matrix4f transform in global coordinates
		 * @param int index of the previous camera
		 * @param int index of the current camera
		 * @return Eigen::Matrix4f transform in local coordinates */
		Eigen::Matrix4f Ladybug2CamRef(Eigen::Matrix4f TGlobal, int camNoPrev, int camNoCurr);

        std::vector<image_geometry::PinholeCameraModel> intrinsics_;	/*!< Real intrinsic parameters: image_geometry::PinholeCameraModel */
        std::vector<sensor_msgs::CameraInfo> cameraInfos_;	            /*!< Real intrinsic parameters: sensor_msgs::CameraInfo */
        std::vector<Eigen::Matrix3f> cameraMatrices_;	           		/*!< Real intrinsic parameters: camera matrices (K) */

        std::vector<Eigen::Matrix4f> extrinsics_;		                /*!< Real extrinsic parameters: tranformation relatively to global (Ladybug) coordinates */

    private:

        int cameraPositions_[6] = {5, 4, 3, 2, 1, 0};                   /*!< Conventional camera order =/= order of the stacked images in the full Ladybug image */

        bool param_simulation_;                                         /*!< Parameter telling if Ladybug is being used for simulated (true) or real (false) data */  
};

#endif
