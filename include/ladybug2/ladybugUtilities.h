#ifndef SPLIT_LADYBUG_H
#define SPLIT_LADYBUG_H

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <highgui.h>

#define IMAGE_HEIGHT 	1024                            /*!< Individual image height */
#define IMAGE_WIDTH	    768                             /*!< Individual image width */
#define NUM_CAMERAS     6                               /*!< Total number of cameras */

#define COLOR 		"bgr8"                              /*!< Color code */
#define MONO		"mono8"                             /*!< Gray scale code */

const int CAMERA_POSITIONS[6] = {5, 4, 3, 2, 1, 0};     /*!< Conventional camera order =/= order of the stacked images in the full Ladybug image */

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

/** Get previous and current camera associated to a given vector position.
 * @param int vector position
 * @param int& output previous camera index
 * @param int& output current camera index
 * @return bool true if valid vector position, false otherwise */
bool getPrevAndCurrCamIndex(int position, int &camPrev, int &camCurr);

/** Get vector position given a previous and current camera index.
 * @param int previous camera index
 * @param int current camera index
 * @param int& output vector index
 * @return bool true if valid camera indices, false otherwise */
bool getVectorPosition(int camPrev, int camCurr, int &position);

#endif
