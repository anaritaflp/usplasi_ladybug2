
// std includes
#include <iostream>
#include <fstream>
#include <string>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <highgui.h>

// project includes
#include <ladybug2/Ladybug2.h>

std::string paramCameraTopic;                       /*!< Parameter: Ladybug image topic */
std::string paramYamlPath;                          /*!< Parameter: Path to calibration files */
bool paramResizeHalf;                               /*!< Parameter: Resize images to half if true, keep original dimensions otherwise */
bool paramColor;                                    /*!< Parameter: Color if true, gray scale otherwise */

Ladybug2 lb2;                                       /*!< Ladybug2 object */

std::vector<sensor_msgs::CameraInfo> camInfos;      /*!< Vector with camera info of each camera */
image_transport::Publisher pub_cams[NUM_CAMERAS];   /*!< Vector with image publishers of each individual camera */
ros::Publisher pub_info[NUM_CAMERAS];               /*!< Vector with camera info publishers of each individual camera */

/** Ladybug image callback.
 * @param const sensor_msgs::Image::ConstPtr& Ladybug image message
 * @return void */
void ladybugCallback(const sensor_msgs::Image::ConstPtr &msg);
