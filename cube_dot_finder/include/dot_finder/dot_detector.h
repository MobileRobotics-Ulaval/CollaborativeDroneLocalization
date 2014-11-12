/*
 * dot_detector.h
 *
 *  Created on: Jun 15, 2014
 *      Author: Philippe Babin
        based on the work of karl Schwabe
 */

/**
 * \file do_detector.h
 * \brief File containing the includes and the function prototypes for the dot_detector.cpp file.
 *
 */

#ifndef DotDetector_H_
#define DotDetector_H_

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <algorithm>

namespace dot_finder
{

typedef Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> List2DPoints;

class DotDetector
{
public:
  /**
   * Detects the duo of markers in the image and returns their positions in the rectified image and also draws circles around the detected marker in an output image.
   *
   * \param image the image in which the LEDs are to be detected
   * \param pixel_positions (output) the vector containing the pixel positions of the centre of the detected blobs
   * \param distorted_detection_centers (output) centers of the LED detection in the distorted image (only used for visualization)
   * \param camera_matrix_K the camera calibration matrix (without distortion correction) \f$K = \left[ \begin{array}{ccc}
   *                         f_x & 0 & c_x \\
   *                         0 & f_y & c_y \\
   *                         0 & 0 & 1 \end{array} \right] \f$
   * \param camera_distortion_coeffs the vector containing the 4, 5 or 8 distortion coefficients of the camera \f$ (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]]) \f$
   * \param camera_matrix_P the 4x4 camera projection matrix that maps real world points to the positions on the undistorted image \f$P = \left[ \begin{array}{cccc}
   *                         f_x & 0 & c_x & 0 \\
   *                         0 & f_y & c_y & 0 \\
   *                         0 & 0 & 1 & 0 \end{array} \right] \f$
   * \param ROI the region of interest in the image to which the image search will be confined
   *
   */
  void dotFilteringCube(const cv::Mat &image,
                           std::vector< std::vector<cv::Point2f> > & trio_distorted,
                           std::vector< std::vector<cv::Point2f> > & dot_hypothesis_distorted, std::vector< std::vector<cv::Point2f> > & dot_hypothesis_undistorted,
                           cv::Rect ROI);

  void setDetectorParameter(const double pThreshold,
                            const double pDilatation, const double pErosion,
                            const double pMax_angle, const double pMinRadius);
  void setCameraParameter(const cv::Mat pCameraMatrixK,
                          const cv::Mat pCameraMatrixP,
                          const std::vector<double> pCameraDistortionCoeffs);

  cv::Mat getVisualisationImg();
  void setRatioDotOnCameraPlan(double pRatioDotOnCameraPlan);

/**
  * Detect duo of marker with almost no filtering, so it can later be convert to a Mathlab compatible file format.
  */
  void trainingDataAcquiring(const cv::Mat &image,
                             std::vector< std::vector<cv::Point2f> > & trio_distorted);
  void saveToCSV(std::vector<int> trioPositive);
private:
  /**
   * Training private Method
  */
  std::string generateDataLine(uint64_t time,
                               std::vector<int> contoursId);
  bool isOutOfRangeDot(int id);


  /**
   * Filtering private method
  */
  void doThresholding(cv::Mat & image);
  void resizeRegionOfInterest(const int colsImg, const int rowsImg, cv::Rect & ROI);

  void colorThresholdingDilateErode(cv::Mat &image);
  void extractContourAndFeatureFromImage(const cv::Mat &image, cv::Rect ROI);
  std::vector<std::vector<cv::Point2f> > extractPairFromContour(std::vector< std::vector<cv::Point2f> > & trio_distorted);
  std::vector<std::vector<cv::Point2f> > paringBlobPair();
  std::vector< std::vector<cv::Point2f> > removeCameraDistortion(std::vector< std::vector<cv::Point2f> > & distortedPoints);


  cv::Mat visualisationImg;
  int threshold;
  double dilatation, erosion, max_angle, min_radius;
  double ratioDotOnCameraPlan; // Percentage of the distance between two blobs where the dot on the camera plan is

  cv::Mat cameraMatrixK, cameraMatrixP;
  std::vector<double> cameraDistortionCoeffs;

  std::vector<std::map <std::string, double> > contoursFeatures;
  std::vector<cv::Point2f> contoursPosition;
  std::vector< std::vector<int> > pairStack;

};

inline cv::Mat DotDetector::getVisualisationImg(){
    return this->visualisationImg;
}

} // namespace

#endif /* DotDetector_H_ */
