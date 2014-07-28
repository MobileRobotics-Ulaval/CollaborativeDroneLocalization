/*
 * dot_finder_node.cpp
 *
 * Created on: Jul 29, 2013
 *      Author: Karl Schwabe
 *  Edited on:  May 14 2014
 *      Author: Philippe Bbin
 */

/** \file mutual_camera_localizator_node.cpp
 * \brief File containing the main function of the package
 *
 * This file is responsible for the flow of the program.
 *
 */
using namespace std;
#include "dot_finder/dot_finder_node.h"

namespace dot_finder
{

/**
 * Constructor of the Dot finder Node class
 *
 */
DotFinder::DotFinder()
{
  // Generate the name of the publishing and subscribing topics
  ros::param::get("~topic", this->topic);

  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  dynamic_reconfigure::Server<dot_finder::DotFinderConfig>::CallbackType dynamicReconfigCallback;
  dynamicReconfigCallback = boost::bind(&DotFinder::dynamicParametersCallback, this, _1, _2);
  dynamicReconfigServer.setCallback(dynamicReconfigCallback);

  // Initialize subscribers
  ROS_INFO("Subscribing to %s /ardrone/image_raw", this->topic.c_str());
  this->createSubscribers();
  this->createPublishers();
}

void DotFinder::createPublishers(){
    image_transport::ImageTransport image_transport(this->nodeHandle);
    this->pubImage = image_transport.advertise(topic + "/ardrone/image_with_detections", 1);
    this->pubDotHypothesis = this->nodeHandle.advertise<dot_finder::DuoDot>(topic  + "/dots", 1);
}

void DotFinder::createSubscribers(){
   this->subImage = this->nodeHandle.subscribe(this->topic + "/ardrone/image_raw", 1, &DotFinder::imageCallback, this);
   this->subCameraInfo = this->nodeHandle.subscribe(this->topic + "/ardrone/camera_info", 1, &DotFinder::cameraInfoCallback, this);
}


DotFinder::~DotFinder(){}

/**
 * The callback function that retrieves the camera calibration information
 *
 * \param msg the ROS message containing the camera calibration information
 *
 */
void DotFinder::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
  if (!this->haveCameraInfo)
  {
    this->camInfo = *msg;

    // Calibrated camera
    Eigen::Matrix<double, 3, 4> camera_matrix;
    camera_matrix(0, 0) = this->camInfo.P[0];
    camera_matrix(0, 2) = this->camInfo.P[2];
    camera_matrix(1, 1) = this->camInfo.P[5];
    camera_matrix(1, 2) = this->camInfo.P[6];
    camera_matrix(2, 2) = 1.0;
    
    this->cameraMatrixK = cv::Mat(3, 3, CV_64F);
    this->cameraMatrixP = cv::Mat(3, 4, CV_64F);

    this->cameraMatrixK.at<double>(0, 0) = this->camInfo.K[0];
    this->cameraMatrixK.at<double>(0, 1) = this->camInfo.K[1];
    this->cameraMatrixK.at<double>(0, 2) = this->camInfo.K[2];
    this->cameraMatrixK.at<double>(1, 0) = this->camInfo.K[3];
    this->cameraMatrixK.at<double>(1, 1) = this->camInfo.K[4];
    this->cameraMatrixK.at<double>(1, 2) = this->camInfo.K[5];
    this->cameraMatrixK.at<double>(2, 0) = this->camInfo.K[6];
    this->cameraMatrixK.at<double>(2, 1) = this->camInfo.K[7];
    this->cameraMatrixK.at<double>(2, 2) = this->camInfo.K[8];
    this->cameraDistortionCoeffs = this->camInfo.D;
    this->cameraMatrixP.at<double>(0, 0) = this->camInfo.P[0];
    this->cameraMatrixP.at<double>(0, 1) = this->camInfo.P[1];
    this->cameraMatrixP.at<double>(0, 2) = this->camInfo.P[2];
    this->cameraMatrixP.at<double>(0, 3) = this->camInfo.P[3];
    this->cameraMatrixP.at<double>(1, 0) = this->camInfo.P[4];
    this->cameraMatrixP.at<double>(1, 1) = this->camInfo.P[5];
    this->cameraMatrixP.at<double>(1, 2) = this->camInfo.P[6];
    this->cameraMatrixP.at<double>(1, 3) = this->camInfo.P[7];
    this->cameraMatrixP.at<double>(2, 0) = this->camInfo.P[8];
    this->cameraMatrixP.at<double>(2, 1) = this->camInfo.P[9];
    this->cameraMatrixP.at<double>(2, 2) = this->camInfo.P[10];
    this->cameraMatrixP.at<double>(2, 3) = this->camInfo.P[11];

   this->cameraProjectionMatrix = camera_matrix;
   this->haveCameraInfo = true;
   ROS_INFO("Camera calibration information obtained.");
  }

}

/**
 * The callback function that is executed every time an image is received. It runs the main logic of the program.
 *
 * \param image_msg the ROS message containing the image to be processed
 */
void DotFinder::imageCallback(const sensor_msgs::Image::ConstPtr& image_msg)
{

  // Check whether already received the camera calibration data
  if (!this->haveCameraInfo){
    ROS_WARN("No camera info yet...");
    return;
  }

  // Import the image from ROS message to OpenCV mat
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR16);// MONO
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  this->publishDetectedLed(cv_ptr->image);

  this->publishVisualizationImage(image_msg);
}


/**
 * Detect, filter and publish markers.
 */
void DotFinder::publishDetectedLed(cv::Mat &image){
  //regionOfInterest = cv::Rect(0, 0, image.cols, image.rows);

    // Do detection of LEDs in image
  std::vector< std::vector<cv::Point2f> > dots_hypothesis_undistorted;

  // Convert image to gray
  this->marqueurDetector.LedFilteringArDrone(image,
              this->trioDistorted, this->dotsHypothesisDistorted, dots_hypothesis_undistorted,
              this->cameraMatrixK, this->cameraDistortionCoeffs, this->cameraMatrixP, this->regionOfInterest);

  if(dots_hypothesis_undistorted.size() == 0){
    ROS_WARN("No LED detected");
    return;
  }

  // Publication of the position of the possible position in the image
  dot_finder::DuoDot duoDot_msg_to_publish;
  geometry_msgs::Pose2D position_in_image;
  for (int i = 0; i < dots_hypothesis_undistorted.size(); i++){
    //Left dot
    position_in_image.x = dots_hypothesis_undistorted[i][0].x;
    position_in_image.y = dots_hypothesis_undistorted[i][0].y;
    duoDot_msg_to_publish.leftDot.push_back(position_in_image);

    //Right dot
    position_in_image.x = dots_hypothesis_undistorted[i][1].x;
    position_in_image.y = dots_hypothesis_undistorted[i][1].y;
    duoDot_msg_to_publish.rightDot.push_back(position_in_image);
  }

  duoDot_msg_to_publish.topicName = this->topic;
  // We also publish the focal length
  duoDot_msg_to_publish.fx = this->cameraMatrixK.at<double>(0, 0);
  duoDot_msg_to_publish.fy = this->cameraMatrixK.at<double>(1, 1);

  duoDot_msg_to_publish.px = this->cameraMatrixK.at<double>(0, 2);
  duoDot_msg_to_publish.py = this->cameraMatrixK.at<double>(1, 2);

  this->pubDotHypothesis.publish(duoDot_msg_to_publish);
  
}

/**
 * The visualization creator and publisher.
 */
void DotFinder::publishVisualizationImage(const sensor_msgs::Image::ConstPtr& image_msg){
  cv::Mat image;
  cv::cvtColor(this->marqueurDetector.getVisualisationImg() , image, CV_GRAY2BGR);
  if(this->infoToggle)
       Visualization::createVisualizationImage(image, this->dotsHypothesisDistorted, trioDistorted, regionOfInterest, duoToggle, trioToggle);
  // Convert to color image for visualisation
  // Publish image for visualization
  cv_bridge::CvImage visualized_image_msg;
  visualized_image_msg.header = image_msg->header;
  visualized_image_msg.encoding = sensor_msgs::image_encodings::BGR8; //BGR8
  visualized_image_msg.image = image;

  this->pubImage.publish(visualized_image_msg.toImageMsg());
}

/**
 * The dynamic reconfigure callback function. This function updates the variable within the program whenever they are changed using dynamic reconfigure.
 */
void DotFinder::dynamicParametersCallback(dot_finder::DotFinderConfig &config, uint32_t level){
  // Visualization Parameters
  // TODO but them into visualization object
  this->infoToggle = config.infoToggle;
  this->duoToggle  = config.duoToggle;
  this->trioToggle = config.trioToggle;

  // ROI Parameters
  if(config.toggleROI){
     regionOfInterest = cv::Rect(config.xROI, config.yROI, config.wROI, config.hROI);
  }
  else{
     regionOfInterest = cv::Rect(0, 0, 640, 360);
  }

  // Detector Parameters
  this->marqueurDetector.setDetectorParameter(config.min_radius,
                                             config.morph_type,
                                             config.dilation_size,
                                             config.erosion_size,
                                             config.maxAngle * M_PI/180.0,
                                             config.maxAngleBetweenTrio * M_PI/180.0,
                                             config.maxNormOnDist,
                                             config.maskToggle);

  // Color detection parameter
  this->marqueurDetector.setOrangeParameter(config.OrangeHueHigh, config.OrangeSatHigh, config.OrangeValueHigh,
                                            config.OrangeHueLow,  config.OrangeSatLow,  config.OrangeValueLow);
  this->marqueurDetector.setBlueParameter(config.BlueHueHigh, config.BlueSatHigh, config.BlueValueHigh,
                                          config.BlueHueLow,  config.BlueSatLow,  config.BlueValueLow);

  ROS_INFO("Parameters changed");
}

} // namespace dot_finder

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dot_finder", ros::init_options::AnonymousName);

  dot_finder::DotFinder dot_finder_node;
  ros::spin();
  return 0;
}
// lol
