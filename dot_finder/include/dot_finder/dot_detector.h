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

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <algorithm>

namespace dot_finder
{

typedef Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> List2DPoints;

class DotDetector
{
public:
    cv::Mat getVisualisationImg();
    void setOrangeParameter(const int pHighH, const int pHighS, const int pHighV,
                            const int pLowH,  const int pLowS,  const int pLowV);
    void setBlueParameter(const int pHighH, const int pHighS, const int pHighV,
                          const int pLowH,  const int pLowS,  const int pLowV);
    void setDetectorParameter(const int min_radius, const int morph_type,
                              const double dilatation, const double erosion, const double max_angle,
                              const double max_angle_duo, const double max_norm_on_dist,
                              const bool maskToggle);
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
  void LedFilteringArDrone(const cv::Mat &image,
                            std::vector< std::vector<cv::Point2f> > & trio_distorted,
                            std::vector< std::vector<cv::Point2f> > & dot_hypothesis_distorted, std::vector< std::vector<cv::Point2f> > & dot_hypothesis_undistorted,
                           const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                           const cv::Mat &camera_matrix_P, cv::Rect ROI);
private:
  void colorThresholding(cv::Mat & image,
                         const int pHighH, const int pHighS, const int pHighV,
                         const int pLowH,  const int pLowS,  const int pLowV);
  void redHueThresholding(cv::Mat & pImage,
                          const int pHighH, const int pHighS, const int pHighV,
                          const int pLowH,  const int pLowS,  const int pLowV);
  void resizeRegionOfInterest(const int colsImg, const int rowsImg, cv::Rect & ROI);


  cv::Mat visualisationImg;
  double highHOrange, highSOrange, highVOrange, lowHOrange, lowSOrange, lowVOrange;
  double highHBlue, highSBlue, highVBlue, lowHBlue, lowSBlue, lowVBlue;
  int min_radius, morph_type;
  double dilatation, erosion, max_angle, max_angle_duo, max_norm_on_dist;
  bool maskToggle;

};

inline cv::Mat DotDetector::getVisualisationImg(){
    return this->visualisationImg;
}

} // namespace

#endif /* DotDetector_H_ */
