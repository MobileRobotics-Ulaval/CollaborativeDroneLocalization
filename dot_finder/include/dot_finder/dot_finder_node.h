/*
 * dot_finder_node.h
 *

 *  Edited on:  4 June 2014
 *      Author: Philippe Bbin
 */

/** \file dot_finder_node.h
 * \brief File containing the definition of the Mutual Camera Localizator Node class
 *
 */

#ifndef DOT_FINDER_NODE_H_
#define DOT_FINDER_NODE_H_

#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#include <dynamic_reconfigure/server.h>
#include <dot_finder/DotFinderConfig.h>
#include <dot_finder/dot_detector.h>
#include <dot_finder/visualization.h>

#include <dot_finder/DuoDot.h>

namespace dot_finder
{
class DotFinder{
public:
  DotFinder();
  ~DotFinder();
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg);

  void dynamicParametersCallback(dot_finder::DotFinderConfig &config, uint32_t level);


private:
  void createPublishers();
  void createSubscribers();

  void publishDetectedLed(cv::Mat &image);
  void publishVisualizationImage(const sensor_msgs::Image::ConstPtr& image_msg);

  string topic; //!< Camera node to subscribe
  ros::NodeHandle nodeHandle; //!< The ROS node handler

  DotDetector marqueurDetector;

  image_transport::Publisher pubImage; //!< The ROS image publisher that publishes the visualisation image
  ros::Publisher pubDotHypothesis; //!< The ROS image publisher that publishes the visualisation image

  ros::Subscriber subImage; //!< The ROS subscriber to the raw camera image
  ros::Subscriber subCameraInfo; //!< The ROS subscriber to the camera info

  dynamic_reconfigure::Server<dot_finder::DotFinderConfig> dynamicReconfigServer; //!< The dynamic reconfigure server
  dynamic_reconfigure::Server<dot_finder::DotFinderConfig>::CallbackType dynamicReconfigCallback; //!< The dynamic reconfigure callback type

  //geometry_msgs::PoseWithCovarianceStamped predicted_pose_; //!< The ROS message variable for the estimated pose and covariance of the object

  bool haveCameraInfo; //!< The boolean variable that indicates whether the camera calibration parameters have been obtained from the camera
  sensor_msgs::CameraInfo camInfo; //!< Variable to store the camera calibration parameters

  cv::Rect regionOfInterest;
  cv::Mat cameraMatrixK; //!< Variable to store the camera matrix as an OpenCV matrix
  cv::Mat cameraMatrixP; //!< Variable to store the projection matrix (as an OpenCV matrix) that projects points onto the rectified image plane.
  std::vector<double> cameraDistortionCoeffs; //!< Variable to store the camera distortion parameters
  std::vector< std::vector<cv::Point2f> > trioDistorted; //!< Trio dot with distortion (for visualisation)
  std::vector< std::vector<cv::Point2f> > dotsHypothesisDistorted; //!< Dual marquer with distortion (for visualisation)
  Eigen::Matrix<double, 3, 4> cameraProjectionMatrix; //!< Stores the camera calibration matrix. This is the 3x4 projection matrix that projects the world points to the image coordinates stored in #image_points_.


  bool infoToggle;
  bool duoToggle;
  bool trioToggle;

};

} // dot_finder namespace

#endif /* DOT_FINDER_NODE_H_ */
