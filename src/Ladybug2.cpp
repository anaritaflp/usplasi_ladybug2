
#include <ladybug2/Ladybug2.h>

/** Default constructor */
Ladybug2::Ladybug2()
{
    
}

/** Constructor with arguments 
 * @param ros::NodeHandle ROS node */
Ladybug2::Ladybug2(ros::NodeHandle node)
{
    // get vector with intrinsics data
    getIntrinsics(node);

    // get vector with extrinsic transformations   
    getExtrinsics(node);
}

/** Destructor */
Ladybug2::~Ladybug2()
{
    
}

/** Split Ladybug image.
 * @param cv::Mat Ladybug image, with all individual images stacked
 * @param std::string color code - "bgr8" or "mono8"
 * @return std::vector<cv::Mat> vector with the splitted images */
std::vector<cv::Mat> Ladybug2::splitLadybug(cv::Mat ladybugImage, std::string colorCode)
{
    // rotate
    cv::Mat rotImage;
	cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point2f(3*IMAGE_WIDTH, 3*IMAGE_WIDTH), -90, 1);
	warpAffine(ladybugImage, rotImage, rot_mat, cv::Size(NUM_CAMERAS*IMAGE_WIDTH, IMAGE_HEIGHT));

    // convert to specified color code
    cv::cvtColor(rotImage, rotImage, CV_BayerRG2BGR);
    if(colorCode == "mono8")
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
        splitImages.push_back(splitImages_unsorted[cameraPositions_[i]]);
    }
    
    return splitImages;
}

/** Get the neighbouring left and right camera indices of a specified camera. Only the 5 ring-arranged cameras are considered.
 * @param int index of the camera 
 * @param int& output index of the left neighbouring camera
 * @param int& output index of the right neighbouring camera
 * @return void */
void Ladybug2::getLeftRightCameras(int cam, int &camLeft, int &camRight)
{
    // negative camera index: assume it's camera 0 (first camera in the ring)
    if(cam < 0)
    {
        cam = 0;
    }

    // too large camera index: assume it's camera 4 (last camera in the ring)
    if(cam > NUM_OMNI_CAMERAS)
    {
        cam = 4;
    }

    camLeft = cam - 1;
    camRight = cam + 1;
    
    if(camLeft == -1)
    {
        camLeft = NUM_OMNI_CAMERAS - 1;
    }
    if(camRight == NUM_OMNI_CAMERAS)
    {
        camRight = 0;
    }
}

/** Rectify images with calibration data.
 * @param std::vector<cv::Mat> vector with distorted images
 * @return std::vector<cv::Mat> vector with rectified images */
std::vector<cv::Mat> Ladybug2::rectify(std::vector<cv::Mat> images)
{
    std::vector<cv::Mat> imagesRect;
    for(int i=0; i<NUM_CAMERAS; i++)
    {
        cv::Mat imRect;
        intrinsics_[i].rectifyImage(images[i], imRect);
        imagesRect.push_back(imRect);
    }
    return imagesRect;
}

/** Get intrinsic parameters from manually obtained .yaml files.
 * @param ros::NodeHandle ROS node for reading parameters
 * @return void */
void Ladybug2::getIntrinsics(ros::NodeHandle node)
{
    intrinsics_.clear();
    cameraInfos_.clear();
    cameraMatrices_.clear();

    // check whether simulated or read data is being used
    node.param<bool>("simulation", param_simulation_, false);

    // simulated data: fill intrinsic calibration structures with datasheet values
    if(param_simulation_)
    {
        // get path of yaml calibration file
        std::string intrinsicsPathIdeal;
        node.param<std::string>("calib_file_cam_ideal", intrinsicsPathIdeal, "");
        
        // open calibration yaml file
        std::ifstream fin(intrinsicsPathIdeal);  
           
        // parse yaml file to get cameraInfo
        std::string camName;
        sensor_msgs::CameraInfo camInfo;
        bool ret = camera_calibration_parsers::readCalibrationYml(fin, camName, camInfo);
        if(ret == false)
        {
            ROS_ERROR("Invalid calibration file! Exiting...");
            ros::shutdown();
        }

        // get pinhole camera model out of cameraInfo
        image_geometry::PinholeCameraModel camModel;
        camModel.fromCameraInfo(camInfo); 

        // get camera matrix
        Eigen::Matrix3f K;
        K << camModel.fx(), 0.0, camModel.cx(), 0.0, camModel.fy(), camModel.cy(), 0.0, 0.0, 1.0;

        for(int i=0; i<NUM_CAMERAS; i++)
        {
            cameraInfos_.push_back(camInfo);
            intrinsics_.push_back(camModel); 
            cameraMatrices_.push_back(K); 
        }
    }
    // real data: fill intrinsic calibration structure with ladybug's calibration data
    else
    {
        for(int i=0; i<NUM_CAMERAS; i++)
        {

            // get path of yaml calibration file
            std::string intrinsicsPath;
            char param_calibPath[50];
            sprintf(param_calibPath, "calib_file_cam_%d", i);
            node.param<std::string>(std::string(param_calibPath), intrinsicsPath, "");

            // open calibration yaml file
            std::ifstream fin(intrinsicsPath);  
            
            // parse yaml file to get cameraInfo
            std::string camName;
            sensor_msgs::CameraInfo camInfo;
            bool ret = camera_calibration_parsers::readCalibrationYml(fin, camName, camInfo);
            if(ret == false)
            {
                ROS_ERROR("Invalid calibration file! Exiting...");
                ros::shutdown();
            }
            cameraInfos_.push_back(camInfo);
            
            // get pinhole camera model out of cameraInfo
            image_geometry::PinholeCameraModel camModel;
            camModel.fromCameraInfo(camInfo);        
            intrinsics_.push_back(camModel);  
            
            // get camera matrix
            Eigen::Matrix3f K;
            K << camModel.fx(), 0.0, camModel.cx(), 0.0, camModel.fy(), camModel.cy(), 0.0, 0.0, 1.0;
            cameraMatrices_.push_back(K); 
        }
    }
}

/** Get extrinsic parameters.
 * @param ros::NodeHandle ROS node for reading parameters
 * @return void */
void Ladybug2::getExtrinsics(ros::NodeHandle node)
{
    extrinsics_.clear();
	
	Eigen::Matrix4f rotMat;
	rotMat << 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    for(int i=0; i<NUM_CAMERAS; i++)
    {
        // get vector with extrinsics data
        std::vector<double> extrinsicsData;
        char extParamName[50];
        sprintf(extParamName, "cam%d_ext", i);
        node.getParam(std::string(extParamName), extrinsicsData);

        // get rotation matrix
        double rx = extrinsicsData[0];
        double ry = extrinsicsData[1];
        double rz = extrinsicsData[2];
        Eigen::Matrix3f Rx, Ry, Rz;
        Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
        Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
        Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;    
        Eigen::Matrix3f R = Rz * Ry * Rx;

        // get translation vector
        Eigen::Vector3f t;
        t << extrinsicsData[3], extrinsicsData[4], extrinsicsData[5];
        
        // get transform
        Eigen::Matrix4f ext;
        ext << R(0, 0), R(0, 1), R(0, 2), t(0), R(1, 0), R(1, 1), R(1, 2), t(1), R(2, 0), R(2, 1), R(2, 2), t(2), 0, 0, 0, 1;
		
		ext = ext * rotMat;

        extrinsics_.push_back(ext);
    }
}

/** Convert transform in local (camera) coordinates to global (ladybug) coordinates.
 * @param Eigen::Matrix4f transform in local coordinates
 * @param int index of the previous camera
 * @param int index of the current camera
 * @return Eigen::Matrix4f transform in global coordinates */
Eigen::Matrix4f Ladybug2::cam2LadybugRef(Eigen::Matrix4f TLocal, int camNoPrev, int camNoCurr)
{
    Eigen::Matrix4f TGlobal = extrinsics_[camNoPrev] * TLocal.inverse() * extrinsics_[camNoCurr].inverse();
    return TGlobal;
}

/** Convert transform in global (ladybug) coordinates to local (camera) coordinates.
 * @param Eigen::Matrix4f transform in global coordinates
 * @param int index of the previous camera
 * @param int index of the current camera
 * @return Eigen::Matrix4f transform in local coordinates */
Eigen::Matrix4f Ladybug2::Ladybug2CamRef(Eigen::Matrix4f TGlobal, int camNoPrev, int camNoCurr)
{
    Eigen::Matrix4f TLocalInv = extrinsics_[camNoPrev].inverse() * TGlobal * extrinsics_[camNoCurr];
    Eigen::Matrix4f TLocal = TLocalInv.inverse();
    return TLocal;
}
