/*
 * visualization.h
 *
 *  Created on: Mar 21, 2014
 *      Author: Matthias Faessler
 */

/**
 * \file visualization.h
 * \brief File containing the includes and the function prototypes for the visualization.cpp file.
 *
 */

#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <sstream>

namespace dot_finder
{
typedef Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> List2DPoints; //!< A dynamic column vector containing Vector2D elements. \see Vector2d

class Visualization
{
public:

  static void createVisualizationImage(cv::Mat &image, std::vector< std::vector<cv::Point2f> > dots_hypothesis_distorted, std::vector< std::vector<cv::Point2f> > trio_distorted,
                                             cv::Rect region_of_interest, const bool trio, const bool markerDuo);
  static void drawMarkerPair(cv::Mat &image, std::vector< std::vector<cv::Point2f> > dots_hypothesis_distorted);
  static void drawTrio(cv::Mat &image, std::vector< std::vector<cv::Point2f> > trio_distorted);

};

} // namespace

#endif /* VISUALIZATION_H_ */
