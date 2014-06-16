/*
 * visualization.cpp
 *
 *  Created on: Mar 21, 2014
 *      Author: Matthias Faessler
 */

/**
 * \file visualization.cpp
 * \brief File containing the function definitions for all visualization tasks
 *
 */

#include "dot_finder/visualization.h"
#include <stdio.h>
#include "ros/ros.h"

namespace dot_finder
{

// Function that projects the RGB orientation vectors back onto the image
void Visualization::projectOrientationVectorsOnImage(cv::Mat &image, const std::vector<cv::Point3f> points_to_project,
                                                     const cv::Mat camera_matrix_K,
                                                     const std::vector<double> camera_distortion_coeffs)
{

  std::vector<cv::Point2f> projected_points;

  // 0 rotation
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);

  // 0 translation
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

  projectPoints(points_to_project, rvec, tvec, camera_matrix_K, camera_distortion_coeffs, projected_points);

  cv::line(image, projected_points[0], projected_points[1], CV_RGB(255, 0, 0), 2);
  cv::line(image, projected_points[0], projected_points[2], CV_RGB(0, 255, 0), 2);
  cv::line(image, projected_points[0], projected_points[3], CV_RGB(0, 0, 255), 2);

}

void Visualization::createVisualizationImage(cv::Mat &image, std::vector< std::vector<cv::Point2f> > dots_hypothesis_distorted,
                                             cv::Rect region_of_interest)
{
  int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  double fontScale = 0.7;
  int thickness = 3;
  CvScalar color;

  std::vector<cv::Point2f> hypothesis;
  for(int index = 0; index < dots_hypothesis_distorted.size(); index++){
    switch(index % 4){
      case 0:
        color = CV_RGB(0, 255, 0);
        break;
      case 1:
        color = CV_RGB(0, 255, 255);
        break;
      case 2:
        color = CV_RGB(255, 255, 255);
        break;
      case 3:
        color = CV_RGB(128, 255, 128);
        break;
      default:
        color = CV_RGB(255, 255, 0);
        break;
    }

    hypothesis = dots_hypothesis_distorted[index];

    // Draw a line between each trio of dot
    //for (int i = 1; i < hypothesis.size(); i++){
    //  cv::line(image, hypothesis[i - 1], hypothesis[i], color, 2);
   // }

    // Draw a circle around each dot
    //Orange dot
    cv::circle(image, hypothesis[0], 2, CV_RGB(255, 0, 0), 2);
    cv::circle(image, hypothesis[1], 2, CV_RGB(255, 0, 0), 2);
    //Blue dot
    cv::circle(image, hypothesis[2], 2, CV_RGB(0, 150, 255), 2);

    // Radius circle
    //cv::circle(image, hypothesis[0], hypothesis[3].x, CV_RGB(0, 255, 0), 1);


    std::stringstream ss;
    ss << hypothesis[3].y;
    cv::putText(image, ss.str(), hypothesis[0], fontFace, fontScale, color);
  }

  // Draw region of interest
  cv::rectangle(image, region_of_interest, CV_RGB(0, 0, 255), 2);

}

} // namespace
