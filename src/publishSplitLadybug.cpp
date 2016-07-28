
// Ladybug includes
#include <ladybug2/publishSplitLadybug.h>

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

    image_transport::ImageTransport it(n); 
    
    lb2 = Ladybug2(privateNode);
    
    camInfos = lb2.cameraInfos_;
    
    // for each camera...
    for(int i=0; i<NUM_CAMERAS; i++)
    {
        // initialize camera info publishers
        char infoTopic[100];
        sprintf(infoTopic, "/ladybug/%d/camera_info", i);
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
        colorCode = "bgr8";
    }
    else
    {
        colorCode = "mono8";
    }

    // split Ladybug image
    std::vector<cv::Mat> splitImages = lb2.splitLadybug(full_raw_img, colorCode);	

    std::vector<cv::Mat> imagesRect = lb2.rectify(splitImages);

    // for each camera...
    for(int i=0; i<NUM_CAMERAS; i++)
    {
        cv::Mat im = imagesRect[i];

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
