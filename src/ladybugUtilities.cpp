#include <ladybug2/ladybugUtilities.h>

/** Split Ladybug image.
 * @param cv::Mat Ladybug image, with all individual images stacked
 * @param std::string color code - COLOR or MONO
 * @return std::vector<cv::Mat> vector with the splitted images */
std::vector<cv::Mat> splitLadybug(cv::Mat ladybugImage, std::string colorCode)
{
    // rotate
    cv::Mat rotImage;
	cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point2f(3*IMAGE_WIDTH, 3*IMAGE_WIDTH), -90, 1);
	warpAffine(ladybugImage, rotImage, rot_mat, cv::Size(NUM_CAMERAS*IMAGE_WIDTH,IMAGE_HEIGHT));

    // convert to specified color code
    cv::cvtColor(rotImage, rotImage, CV_BayerRG2BGR);
    if(colorCode == MONO)
    {
        cv::cvtColor(rotImage, rotImage, CV_BGR2GRAY);
    }

    // split image and store in vector
    std::vector<cv::Mat> splitImages_unsorted;
    for(int i=0; i<NUM_CAMERAS; i++)
    {
        int offset = i * IMAGE_WIDTH;
        cv::Mat im = rotImage.rowRange(0, IMAGE_HEIGHT).colRange(offset, offset+IMAGE_WIDTH);
        splitImages_unsorted.push_back(im);
    }

    // order vector
    std::vector<cv::Mat> splitImages;
    for(int i=0; i<NUM_CAMERAS; i++)
    {
        splitImages.push_back(splitImages_unsorted[CAMERA_POSITIONS[i]]);
    }
    
    return splitImages;
}

/** Get the neighbouring left and right camera indices of a specified camera. Only the 5 ring-arranged cameras are considered.
 * @param int index of the camera 
 * @param int& output index of the left neighbouring camera
 * @param int& output index of the right neighbouring camera
 * @return void */
void getLeftRightCameras(int cam, int &camLeft, int &camRight)
{
    // negative camera index: assume it's camera 0 (first camera in the ring)
    if(cam < 0)
    {
        cam = 0;
    }

    // too large camera index: assume it's camera 4 (last camera in the ring)
    if(cam > NUM_CAMERAS-2)
    {
        cam = 4;
    }

    camLeft = cam - 1;
    camRight = cam + 1;
    
    if(camLeft == -1)
    {
        camLeft = NUM_CAMERAS - 2;
    }
    if(camRight == NUM_CAMERAS - 1)
    {
        camRight = 0;
    }
}

/** Get previous and current camera associated to a given vector position.
 * @param int vector position
 * @param int& output previous camera index
 * @param int& output current camera index
 * @return bool true if valid vector position, false otherwise */
 bool getPrevAndCurrCamIndex(int position, int &camPrev, int &camCurr)
 {
     if(position < 0 || position >= 15)
     {
         return false;
     }

     camPrev = position/3;
     if(position%3 == 0)
     {
         camCurr = camPrev;
     }
     else
     {
         int camLeft, camRight;
         getLeftRightCameras(camPrev, camLeft, camRight);
         if(position%3 == 1)
         {
             camCurr = camRight;
         }
         else if(position%3 == 2)
         {
             camCurr = camLeft;
         }
     }
     return true;
 }
 
 /** Get vector position given a previous and current camera index.
 * @param int previous camera index
 * @param int current camera index
 * @param int& output vector index
 * @return bool true if valid camera indices, false otherwise */
bool getVectorPosition(int camPrev, int camCurr, int &position)
{
	if(camPrev < 0 || camPrev > 4)
	{
		return false;
	}
	position = 3 * camPrev;
	
	if(camCurr == camPrev)
	{
		return true;
	}
	
	int camLeft, camRight;
	getLeftRightCameras(camPrev, camLeft, camRight);
	if(camCurr == camRight)
	{
		position += 1;
		return true;
	}
	if(camCurr == camLeft)
	{
		position += 2;
		return true;
	}
	return false;
}
