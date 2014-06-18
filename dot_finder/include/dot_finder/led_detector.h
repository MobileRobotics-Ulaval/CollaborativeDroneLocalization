/*
 * led_detector.h
 *
 *  Created on: Jul 29, 2013
 *      Author: karl Schwabe
 */

/**
 * \file led_detector.h
 * \brief File containing the includes and the function prototypes for the led_detector.cpp file.
 *
 */

#ifndef LEDDETECTOR_H_
#define LEDDETECTOR_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <algorithm>

namespace dot_finder
{

typedef Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> List2DPoints;

class LEDDetector
{
private:
    double m_highHOrange, m_highSOrange, m_highVOrange, m_lowHOrange, m_lowSOrange, m_lowVOrange;
    double m_highHBlue, m_highSBlue, m_highVBlue, m_lowHBlue, m_lowSBlue, m_lowVBlue;
public:
    cv::Mat m_visualisationImg;

    void setOrangeParameter(const int pHighH, const int pHighS, const int pHighV,
                            const int pLowH,  const int pLowS,  const int pLowV);
    void setBlueParameter(const int pHighH, const int pHighS, const int pHighV,
                          const int pLowH,  const int pLowS,  const int pLowV);
    void colorThresholding(cv::Mat & image,
                                        const int pHighH, const int pHighS, const int pHighV,
                                        const int pLowH,  const int pLowS,  const int pLowV);
    double distanceFromLineToPoint(const cv::Point2f p, const cv::Point2f lineA, const cv::Point2f lineB);
  /**
   * Detects the LEDs in the image and returns their positions in the rectified image and also draws circles around the detected LEDs in an output image.
   *
   * The image is thresholded where everything smaller than the threshold is set to zero. I.e.,
   * \f[
   \mathbf{I}^\prime (u, v) = \begin{cases} \mathbf{I}(u, v), & \mbox{if } \mathbf{I}(u, v) > \mbox{threshold}, \\ 0, & \mbox{otherwise}. \end{cases}
   * \f]
   *
   *
   * Thereafter the image is Gaussian smoothed. Contours are extracted and their areas are determined. Blobs with
   * too large, or too small an area are ignored. Blobs that are not sufficiently round are also ignored.
   *
   * The centre of the extracted blobs is calculated using the method of moments. The centre \f$(\hat{u}, \hat{v})\f$ is calculated as \n
   * \f{eqnarray*}{
   *       \hat{u} & = & M_{10} / M_{00}, \\
   *       \hat{v} & = & M_{01} / M_{00}.
   * \f}
   * where the image moments are defined as
   * \f[
   *      M_{pq} = \sum_u \sum_v u^p v^q I^\prime (u,v)
   * \f]
   *
   *
   * \param image the image in which the LEDs are to be detected
   * \param ROI the region of interest in the image to which the image search will be confined
   * \param threshold_value the threshold value for the LED detection
   * \param gaussian_sigma standard deviation \f$\sigma\f$ of the Gaussian that is to be applied to the thresholded image
   * \param min_blob_area the minimum blob area to be detected (in pixels squared)
   * \param max_blob_area the maximum blob area to be detected (in pixels squared)
   * \param max_width_height_distortion the maximum distortion ratio of the width of the blob to the height of the blob. Calculated as \f$1-\frac{\mbox{width}}{\mbox{height}}\f$
   * \param max_circular_distortion the maximum distortion circular distortion ratio, calculated as \f$ 1- \frac{\mbox{area}}{(\mbox{width}/2)^2} \f$ or \f$ 1- \frac{\mbox{area}}{(\mbox{height}/2)^2} \f$
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
   *
   */
  void LedFilteringArDrone(const cv::Mat &image, const int &min_radius, const int &morph_type, const double &dilatation, const double &erosion, const double &max_angle, const double &max_angle_duo,
                           std::vector< std::vector<cv::Point2f> > & dot_hypothesis_distorted, std::vector< std::vector<cv::Point2f> > & dot_hypothesis_undistorted,
                           const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                           const cv::Mat &camera_matrix_P, bool debug);
  /**
   * Calculates the region of interest (ROI) in the distorted image in which the points lie.
   *
   * The pixel positions of the points in the undistorted image are bounded with a box. The corner points of the rectangle are
   * undistorted to produce a new region of interest in the distorted image. This region of interest is returned.
   *
   * \param pixel_positions position of the points in the undistorted image
   * \param image_size size of the image
   * \param border_size size of the boarder around the bounding box of points
   * \param camera_matrix_K camera matrix
   * \param camera_distortion_coeffs the distortion coefficients of the camera
   * \param camera_matrix_P projection matrix that takes points from the world and projects them onto the rectified image plane
   *
   * \return the rectangular region of interest to be processed in the image
   *
   */
  static cv::Rect determineROI(List2DPoints pixel_positions, cv::Size image_size, const int border_size,
                               const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                               const cv::Mat &camera_matrix_P);

};

} // namespace

#endif /* LEDDETECTOR_H_ */
