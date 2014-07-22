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
private:
  string m_topic; //!< Camera node to subscribe
  ros::NodeHandle m_nodeHandler; //!< The ROS node handler

  DotDetector marqueurDetector;

  image_transport::Publisher m_image_pub; //!< The ROS image publisher that publishes the visualisation image
  ros::Publisher m_dot_hypothesis_pub; //!< The ROS image publisher that publishes the visualisation image

  ros::Subscriber m_image_sub; //!< The ROS subscriber to the raw camera image 
  ros::Subscriber m_camera_info_sub; //!< The ROS subscriber to the camera info

  dynamic_reconfigure::Server<dot_finder::DotFinderConfig> m_dr_server; //!< The dynamic reconfigure server
  dynamic_reconfigure::Server<dot_finder::DotFinderConfig>::CallbackType m_cb; //!< The dynamic reconfigure callback type

  //geometry_msgs::PoseWithCovarianceStamped predicted_pose_; //!< The ROS message variable for the estimated pose and covariance of the object

  bool m_have_camera_info; //!< The boolean variable that indicates whether the camera calibration parameters have been obtained from the camera
  sensor_msgs::CameraInfo m_cam_info; //!< Variable to store the camera calibration parameters

  cv::Rect m_region_of_interest;
  cv::Mat m_camera_matrix_K; //!< Variable to store the camera matrix as an OpenCV matrix
  cv::Mat m_camera_matrix_P; //!< Variable to store the projection matrix (as an OpenCV matrix) that projects points onto the rectified image plane.
  std::vector<double> m_camera_distortion_coeffs; //!< Variable to store the camera distortion parameters
  std::vector< std::vector<cv::Point2f> > m_trio_distorted; //!< Trio dot with distortion (for visualisation)
  std::vector< std::vector<cv::Point2f> > m_dots_hypothesis_distorted; //!< Dual marquer with distortion (for visualisation)
  Eigen::Matrix<double, 3, 4> m_camera_projection_matrix; //!< Stores the camera calibration matrix. This is the 3x4 projection matrix that projects the world points to the image coordinates stored in #image_points_.

  double m_highH, m_highS, m_highV;
  double m_lowH, m_lowS, m_lowV;

  bool m_maskToggle;
  bool m_infoToggle;
  bool m_duoToggle;
  bool m_trioToggle;

  double m_dilation_size, m_erosion_size;
  int  m_min_radius, m_morph_type;

  double m_max_norm_on_dist;
  double m_max_angle;
  double m_max_angle_duo;
  double m_gaussian_sigma;
  double m_min_blob_area;
  double m_max_blob_area;
  double m_max_circular_distortion;
  double m_ratio_ellipse_min;
  double m_ratio_ellipse_max;
  double m_pos_ratio;
  double m_pos_ratio_tolerance;
  double m_acos_tolerance;
  double m_hor_line_angle;
  double m_radius_ratio_tolerance;
  double m_ratio_int_tolerance;

public:

  DotFinder();
  ~DotFinder();
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg);

  void publishDetectedLed(cv::Mat image);

  void publishVisualizationImage(cv::Mat &image, const sensor_msgs::Image::ConstPtr& image_msg);

  void dynamicParametersCallback(dot_finder::DotFinderConfig &config, uint32_t level);
};

} // dot_finder namespace

#endif /* DOT_FINDER_NODE_H_ */
