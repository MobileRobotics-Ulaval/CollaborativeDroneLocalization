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


void Visualization::createVisualizationImage(cv::Mat &image, std::vector< std::vector<cv::Point2f> > dots_hypothesis_distorted, std::vector< std::vector<cv::Point2f> > trio_distorted,
                                             cv::Rect region_of_interest, const bool markerDuo, const bool trio)
{
    if(trio)
        drawTrio(image, trio_distorted);
    if(markerDuo)
        drawMarkerPair(image, dots_hypothesis_distorted);

    // Draw region of interest
    cv::rectangle(image, region_of_interest, CV_RGB(0, 0, 255), 2);
}

void Visualization::drawTrio(cv::Mat &image, std::vector< std::vector<cv::Point2f> > trio_distorted){
  int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  double fontScale = 0.7;
  int thickness = 3;
  CvScalar color;

  std::vector<cv::Point2f> hypothesis;
  for(int index = 0; index < trio_distorted.size(); index++){
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

    hypothesis = trio_distorted[index];

    // Draw a line between each trio of dot
    //for (int i = 1; i < hypothesis.size(); i++){
    //  cv::line(image, hypothesis[i - 1], hypothesis[i], color, 2);
   // }

    // Draw a circle around each dot
    //Orange dot
    cv::circle(image, hypothesis[0], 2, CV_RGB(255, 0, 0), 2);
    cv::circle(image, hypothesis[1], 2, CV_RGB(255, 0, 0), 2);
    //Blue dot
    //cv::circle(image, hypothesis[2], 2, CV_RGB(0, 150, 255), 2);
    std::stringstream ss;
    ss << hypothesis[3].y;
    cv::putText(image, ss.str(), hypothesis[0], fontFace, fontScale, color);



  }
}

void Visualization::drawMarkerPair(cv::Mat &image, std::vector< std::vector<cv::Point2f> > dots_hypothesis_distorted){
   std::vector<cv::Point2f> hypothesis;
  // ==== Draw the pairs of marker ====
  for(int index = 0; index < dots_hypothesis_distorted.size(); index++){
        hypothesis = dots_hypothesis_distorted[index];
        cv::circle(image, hypothesis[0], 2, CV_RGB(173, 33, 96), 2);
        cv::circle(image, hypothesis[1], 2, CV_RGB(126,189, 36), 2);

        cv::line(image, hypothesis[0], hypothesis[1], CV_RGB(255, 0, 0), 1);



  }

}

} // namespace
