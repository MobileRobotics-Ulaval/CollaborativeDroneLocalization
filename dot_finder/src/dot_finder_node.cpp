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
DotFinder::DotFinder():trainingToggle(false)
{
  // Generate the name of the publishing and subscribing topics
  ros::param::get("~topic", this->topic);
  string training;
  ros::param::get("~training", training);
  if(training == "on"){
      trainingToggle = true;
  }

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
   this->subImage = this->nodeHandle.subscribe(this->topic + "/ardrone/image_raw", 150, &DotFinder::imageCallback, this);
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
    cv::Mat cameraMatrixP = cv::Mat(3, 4, CV_64F);

    this->cameraMatrixK.at<double>(0, 0) = this->camInfo.K[0];
    this->cameraMatrixK.at<double>(0, 1) = this->camInfo.K[1];
    this->cameraMatrixK.at<double>(0, 2) = this->camInfo.K[2];
    this->cameraMatrixK.at<double>(1, 0) = this->camInfo.K[3];
    this->cameraMatrixK.at<double>(1, 1) = this->camInfo.K[4];
    this->cameraMatrixK.at<double>(1, 2) = this->camInfo.K[5];
    this->cameraMatrixK.at<double>(2, 0) = this->camInfo.K[6];
    this->cameraMatrixK.at<double>(2, 1) = this->camInfo.K[7];
    this->cameraMatrixK.at<double>(2, 2) = this->camInfo.K[8];
    std::vector<double> cameraDistortionCoeffs = this->camInfo.D;
    cameraMatrixP.at<double>(0, 0) = this->camInfo.P[0];
    cameraMatrixP.at<double>(0, 1) = this->camInfo.P[1];
    cameraMatrixP.at<double>(0, 2) = this->camInfo.P[2];
    cameraMatrixP.at<double>(0, 3) = this->camInfo.P[3];
    cameraMatrixP.at<double>(1, 0) = this->camInfo.P[4];
    cameraMatrixP.at<double>(1, 1) = this->camInfo.P[5];
    cameraMatrixP.at<double>(1, 2) = this->camInfo.P[6];
    cameraMatrixP.at<double>(1, 3) = this->camInfo.P[7];
    cameraMatrixP.at<double>(2, 0) = this->camInfo.P[8];
    cameraMatrixP.at<double>(2, 1) = this->camInfo.P[9];
    cameraMatrixP.at<double>(2, 2) = this->camInfo.P[10];
    cameraMatrixP.at<double>(2, 3) = this->camInfo.P[11];

   this->cameraProjectionMatrix = camera_matrix;
   this->haveCameraInfo = true;
   ROS_INFO("Camera calibration information obtained.");

   this->markerDetector.setCameraParameter(cameraMatrixK,cameraMatrixP, cameraDistortionCoeffs);
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

  if(!this->trainingToggle/* && false*/){
      this->publishDetectedDot(cv_ptr->image);
      this->publishVisualizationImage(image_msg);
  }
  else{
      this->saveDetectedData(cv_ptr->image);
      this->publishVisualizationImage(image_msg);

      vector<int> trio_positive = this->getHumanInputTrueFinding();
      this->markerDetector.saveToCSV(trio_positive);
  }
}



/**
 * Detect, filter and publish markers.
 */
void DotFinder::publishDetectedDot(cv::Mat &image){
  std::vector< std::vector<cv::Point2f> > dots_hypothesis_undistorted;
  this->markerDetector.dotFilteringArDrone(image,
                                           this->trioDistorted,
                                           this->dotsHypothesisDistorted,
                                           dots_hypothesis_undistorted,
                                           this->regionOfInterest);

  if(dots_hypothesis_undistorted.size() == 0){
    ROS_WARN("No marker detected");
  }

  dot_finder::DuoDot msg = this->generateDotHypothesisMessage(dots_hypothesis_undistorted);
  this->pubDotHypothesis.publish(msg);
  
}

dot_finder::DuoDot DotFinder::generateDotHypothesisMessage(std::vector< std::vector<cv::Point2f> > pDots){
    dot_finder::DuoDot duoDot_msg_to_publish;
    geometry_msgs::Pose2D position_in_image;
    for (int i = 0; i < pDots.size(); i++){
      //Left dot
      position_in_image.x = pDots[i][0].x;
      position_in_image.y = pDots[i][0].y;
      duoDot_msg_to_publish.leftDot.push_back(position_in_image);

      //Right dot
      position_in_image.x = pDots[i][1].x;
      position_in_image.y = pDots[i][1].y;
      duoDot_msg_to_publish.rightDot.push_back(position_in_image);

      // distort
      position_in_image.x = this->dotsHypothesisDistorted[i][0].x; position_in_image.y = this->dotsHypothesisDistorted[i][0].y;
      duoDot_msg_to_publish.leftDistortDot.push_back(position_in_image);

      position_in_image.x = this->dotsHypothesisDistorted[i][1].x; position_in_image.y = this->dotsHypothesisDistorted[i][1].y;
      duoDot_msg_to_publish.rightDistortDot.push_back(position_in_image);
    }

    duoDot_msg_to_publish.header.stamp = ros::Time::now();
    return duoDot_msg_to_publish;
}

/**
 * The visualization creator and publisher.
 */
void DotFinder::publishVisualizationImage(const sensor_msgs::Image::ConstPtr& image_msg){
  cv::Mat image;
  cv::cvtColor(this->markerDetector.getVisualisationImg() , image, CV_GRAY2BGR);
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


vector<int> DotFinder::getHumanInputTrueFinding(){
    vector<int> trio_positive;
    string str;
    int id;
    bool flag = true;

    while(flag && ros::ok()){
        cout << "Please enter trio id ('n' to stop, 'c' to clear): ";
        getline (cin, str);
        stringstream stream(str);
        if(stream >> id){
            trio_positive.push_back(id);
            cout << id << " added." << endl;
        }
        else{
            if(str == "n"){
                cout << "Finish positive input for this image" << endl << endl;
                flag = false;
            }
            else if(str == "c"){
                cout << "Clear previous statements" << endl;
                trio_positive.clear();
            }
            else{
                cout << "Invalide input!" << endl;
            }
        }
    }

    return trio_positive;
}

void DotFinder::saveDetectedData(cv::Mat &image){
    this->markerDetector.trainingDataAcquiring(image,
                                               this->trioDistorted);
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
  //this->trainingToggle = config.trainingToggle;

  // ROI Parameters
  if(config.toggleROI){
     regionOfInterest = cv::Rect(config.xROI, config.yROI, config.wROI, config.hROI);
  }
  else{
     regionOfInterest = cv::Rect(0, 0, 640, 360);
  }

  // Detector Parameters
  this->markerDetector.setDetectorParameter(config.dilation_size,
                                            config.erosion_size,
                                            config.maxAngle * M_PI/180.0);

  // Color detection parameter
  this->markerDetector.setOrangeParameter(config.OrangeHueHigh, config.OrangeSatHigh, config.OrangeValueHigh,
                                          config.OrangeHueLow,  config.OrangeSatLow,  config.OrangeValueLow);

  ROS_INFO("Parameters changed");
}

} // namespace dot_finder

int main(int argc, char* argv[])
{
  //ros::init(argc, argv, "dot_finder", ros::init_options::AnonymousName);
  ros::init(argc, argv, "dot_finder");

  dot_finder::DotFinder dot_finder_node;
  ros::spin();
  return 0;
}
// lol
