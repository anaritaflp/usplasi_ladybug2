#include "../include/ladybug2/publishSplitLadybug.h"

/** Main function.
 * @param int number of arguments
 * @param char** vector with arguments (strings)
 * @return int */
int main(int argc, char **argv)
{
    // initialize ROS
    ros::init(argc, argv, "split_ladybug_node");
    ros::NodeHandle n;    
      
    // private node for reading parameters
    ros::NodeHandle privateNode("~");
    privateNode.param("color", paramColor, bool(false));
    privateNode.param("camera_topic", paramCameraTopic, std::string(""));
    privateNode.param("yaml_path", paramYamlPath, std::string(""));

    image_transport::ImageTransport it(n); 

    // for each camera...
    for(int i=0; i<NUM_CAMERAS; i++)
    {
        // build camera info topic (except the header, which is always changing)        
        char yamlFilename[20];
        sprintf(yamlFilename, "cam%d.yaml", CAMERA_POSITIONS[i]);
        std::string yamlFilename_str(yamlFilename);
        camInfos[i] = getCameraInfo(paramYamlPath + yamlFilename_str);

        // initialize camera info publishers
        char infoTopic[100];
        sprintf(infoTopic, "/ladybug/%d/camera_info", CAMERA_POSITIONS[i]);
        std::string infoTopic_str(infoTopic);        
        pub_info[i] = n.advertise<sensor_msgs::CameraInfo>(infoTopic_str, 1);

        // initialize image publishers
        char camTopic[100];
        sprintf(camTopic, "/ladybug/%d/image_raw", i);
        std::string camTopic_str(camTopic);        
        pub_cams[i] = it.advertise(camTopic_str, 1);        
    }

    // subscribe to Ladybug image topic
    image_transport::Subscriber sub = it.subscribe(paramCameraTopic, 1, ladybugCallback);
    
    ros::spin();
    return 0;
}

/** Ladybug image callback.
 * @param const sensor_msgs::Image::ConstPtr& Ladybug image message
 * @return void */
void ladybugCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    // convert image message to cv::Mat
    cv::Mat_<uint8_t> full_raw_img = cv::Mat_<uint8_t>(NUM_CAMERAS*IMAGE_WIDTH, IMAGE_HEIGHT, const_cast<uint8_t*>(&msg->data[0]), msg->step);

    std::string colorCode;
    if(paramColor)
    {
        colorCode = COLOR;
    }
    else
    {
        colorCode = MONO;
    }

    // split Ladybug image
    std::vector<cv::Mat> splitImages = splitLadybug(full_raw_img, colorCode);	

    // for each camera...
    for(int i=0; i<NUM_CAMERAS; i++)
    {
        cv::Mat im = splitImages[i];

        // resize image to half if specified
        if(paramResizeHalf)
        {
            cv::resize(im, im, cv::Size(IMAGE_WIDTH/2, IMAGE_HEIGHT/2));
        }        

        // update header of both image and camera info messages
        std_msgs::Header header(msg->header);
        char frameId[50];
        sprintf(frameId, "ladybug_cam_%d", i);
        header.frame_id = std::string(frameId);
        camInfos[i].header = header;

        // convert cv::Mat to image message
        sensor_msgs::ImagePtr camMsg = cv_bridge::CvImage(header, colorCode, im).toImageMsg();

        // publish image and camera info
         pub_cams[i].publish(camMsg);
         pub_info[i].publish(camInfos[i]);
    }
}

/** Build camera info message from a calibration file (needs to run only once, since calibration data does not change).
 * @param std::string path of the calibration file
 * @return sensor_msgs::CameraInfo camera info message */
sensor_msgs::CameraInfo getCameraInfo(std::string yamlPath)
{
     sensor_msgs::CameraInfo camInfoMsg;    

     // read calibration file
     std::ifstream yaml(yamlPath); 
     std::string word;
     while(yaml.good())
     {
        yaml >> word;

        // assign read values to camera info members    
        if(word == "image_width:")
        {
            yaml >> word;
            camInfoMsg.width = atoi(word.c_str());
        }
        else if(word == "image_height:")
        {
            yaml >> word;
            camInfoMsg.height = atoi(word.c_str());
        }
        else if(word == "camera_matrix:")
        {
            for(int i=0; i<5; i++)
            {
                yaml >> word;
            }
            for(int i=0; i<9; i++)
            {
                yaml >> word;                
                word.erase(word.size()-1, 1);
                if(i == 0)
                {
                    word.erase(0, 1);
                }
                camInfoMsg.K[i] = atof(word.c_str());
            }            
        }
        else if(word == "distortion_model:")
        {
            yaml >> word;
            camInfoMsg.distortion_model = word;
        }
        else if(word == "distortion_coefficients:")
        {
            for(int i=0; i<5; i++)
            {
                yaml >> word;
            }
            for(int i=0; i<5; i++)
            {
                yaml >> word;                
                word.erase(word.size()-1, 1);
                if(i == 0)
                {
                    word.erase(0, 1);
                }
                camInfoMsg.D.push_back(atof(word.c_str()));
            }            
        }
        else if(word == "rectification_matrix:")
        {
            for(int i=0; i<5; i++)
            {
                yaml >> word;
            }
            for(int i=0; i<9; i++)
            {
                yaml >> word;                
                word.erase(word.size()-1, 1);
                if(i == 0)
                {
                    word.erase(0, 1);
                }
                camInfoMsg.R[i] = atof(word.c_str());
            }            
        }
        else if(word == "projection_matrix:")
        {
            for(int i=0; i<5; i++)
            {
                yaml >> word;
            }
            for(int i=0; i<12; i++)
            {
                yaml >> word;                
                word.erase(word.size()-1, 1);
                if(i == 0)
                {
                    word.erase(0, 1);
                }
                camInfoMsg.P[i] = atof(word.c_str());
            }            
        }                
    }

    // assign default values to attributes that are not specified in the calibration file
    camInfoMsg.binning_x = 0;
    camInfoMsg.binning_y = 0;
    camInfoMsg.roi.width = camInfoMsg.width;
    camInfoMsg.roi.height = camInfoMsg.height;
    camInfoMsg.roi.x_offset = 0;
    camInfoMsg.roi.y_offset = 0;
    camInfoMsg.roi.do_rectify = true;

    // update parameters for half-resized image
    if(paramResizeHalf)
    {
        camInfoMsg.width *= 0.5;
        camInfoMsg.height *= 0.5;
        camInfoMsg.roi.width *= 0.5;
        camInfoMsg.roi.height *= 0.5;
        camInfoMsg.K[0] *= 0.5;
        camInfoMsg.K[2] *= 0.5;
        camInfoMsg.K[4] *= 0.5;
        camInfoMsg.K[5] *= 0.5;
        camInfoMsg.P[0] *= 0.5;
        camInfoMsg.P[2] *= 0.5;
        camInfoMsg.P[5] *= 0.5;
        camInfoMsg.P[6] *= 0.5;
    }

    return camInfoMsg;
}