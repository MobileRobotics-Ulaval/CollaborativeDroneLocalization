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
  //int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
  int fontFace = cv::FONT_HERSHEY_PLAIN;
  //int fontFace = cv::InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1, 1, shear=0, thickness=2, lineType=8)
  double fontScale = 1.0;
  int thickness = 3;
  CvScalar color;
  cv::Point2d textPos;

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

    // Draw a circle around each dot
    //Orange dot
    cv::circle(image, hypothesis[0], 2, color, 2);
    cv::circle(image, hypothesis[0], 1, color, 2);//CV_RGB(255, 0, 0)

    std::stringstream ss;
    ss << hypothesis[2].x;
    //cv::putText(image, ss.str(), hypothesis[0], fontFace, fontScale, color);
    textPos = (hypothesis[0] - hypothesis[1])*0.5 + hypothesis[1];

    cv::putText(image, ss.str(),textPos, fontFace, fontScale, color);



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
