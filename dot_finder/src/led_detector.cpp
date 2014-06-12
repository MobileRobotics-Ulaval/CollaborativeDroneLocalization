/*
 * led_detector.cpp
 *
 * Created on: July 29, 2013
 * Author: Karl Schwabe
 */

/**
 * \file led_detector.cpp
 * \brief File containing the function definitions required for detecting LEDs and visualising their detections and the pose of the tracked object.
 *
 */

#include "ros/ros.h"
#include "dot_finder/led_detector.h"

namespace dot_finder
{

typedef Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> List2DPoints; //!< A dynamic column vector containing Vector2D elements. \see Vector2d

void LEDDetector::LedFilteringArDrone(const cv::Mat &gaussian_image, const double &min_blob_area, const double &max_blob_area, const double &max_circular_distortion,
               const double &radius_ratio_tolerance, const double &intensity_ratio_tolerance, const double &max_deviation_horizontal,
               const double &min_ratio_ellipse, const double &max_ratio_ellipse,
               const double &distance_ratio, const double &distance_ratio_tolerance,
               const double &acos_tolerance,
                           std::vector< std::vector<cv::Point2f> > & dot_hypothesis_distorted, std::vector< std::vector<cv::Point2f> > & dot_hypothesis_undistorted,
                           const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                           const cv::Mat &camera_matrix_P, int OutputFlag) {
  cv::Rect ROI= cv::Rect(0,0,gaussian_image.cols,gaussian_image.rows);

    // Find all contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(gaussian_image.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // Vector for containing the detected points that will be undistorted later
    // std::vector<cv::Point2f> distorted_points;
    int DataIndex = 1; // For matlab-compatible output, when chasing the parameters.
  
    std::vector<cv::Point2f> KeptContoursPosition;
    std::vector<cv::Point2f> detection_centers;
    std::vector<double> KeptRadius, KeptAvgIntensity;

    // Identify the blobs in the image
    for (unsigned i = 0; i < contours.size(); i++)
    {
    double area = cv::contourArea(contours[i]); // Blob area
    cv::Rect rect = cv::boundingRect(contours[i]); // Bounding box
    double radius = (rect.width + rect.height) / 4; // Average radius

    cv::Moments mu;
    mu = cv::moments(contours[i], false);
    cv::Point2f mc;
    mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00) + cv::Point2f(ROI.x, ROI.y);

    if (area > 0.01) {
      double width_height_distortion = std::abs(1 - std::min((double)rect.width / (double)rect.height, (double)rect.height / (double)rect.width));
      double circular_distortion1 = std::abs(1 - (area / (CV_PI * pow(rect.width / 2, 2.0))));
      double circular_distortion2 = std::abs(1 - (area / (CV_PI * pow(rect.height / 2, 2.0))));

      cv::RotatedRect minEllipse;
      double RatioEllipse = 1.0;
      if (contours[i].size()>4) {
        minEllipse = cv::fitEllipse(cv::Mat(contours[i]));
        RatioEllipse = float(minEllipse.boundingRect().width+1.0)/float(minEllipse.boundingRect().height+1.0);  // the 0.5 is to increase immunity to small circles.
      }
      int x, y;
      double total_intensity=0.0,avg_intensity;
      for (x = rect.x; x<rect.x+rect.width; x++) {
        for (y = rect.y; y<rect.y+rect.height; y++) {
          cv::Scalar intensity = gaussian_image.at<uchar>(y, x);
          total_intensity+= float(intensity.val[0]);
        }
      }
      avg_intensity = total_intensity/area;

      if (OutputFlag == 1) {
        // We want to output some data for further analysis in matlab.
        printf("%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %8.2f, %6.2f, %.2f, %d\n", mc.x, mc.y, area, radius, width_height_distortion,circular_distortion1,circular_distortion2,total_intensity,avg_intensity,RatioEllipse,DataIndex);
        DataIndex++;
      }
  
      // We will prune the blobs set based on appearance. These were found using matlab.
      if ((area < max_blob_area) && (circular_distortion1 < max_circular_distortion) && (circular_distortion2 < max_circular_distortion)
            && (RatioEllipse < max_ratio_ellipse) && (RatioEllipse > min_ratio_ellipse )) {
        // These will be used further down the filtering pipeline
        // Ideally we would sort in order of intensity, and do a n choose k on a sliding window of that ranked intensity.
        KeptContoursPosition.push_back(mc);
        KeptRadius.push_back(radius);
        KeptAvgIntensity.push_back(avg_intensity);
      }
    }
    }

  // Filtering step #2: doing all the permutations to find the one with the best characteristics.
  // We are basically looking at:
  //   -Three blobs in line (line angle tolerance)
  //   -With a 3-to-1 ratio between the shortest distance to the longest distance
  //   -And similar radius
  int nBlob = KeptContoursPosition.size();
  cv::Point vec1, vec2, vec3, shortArm, longArm;
  double norm1, norm2, norm3, cosArms, horizontal;
  int index;
  double RatioRadii, RatioDistance, RatioIntensity, minDist, maxDist, maxIntensity=0.0;
  int l1, l2, l3;
  std::vector<int> trioDot(3);
  std::vector< std::vector<int> > BestCombo;
  for (l1 = 0; l1<(nBlob-2);l1++) {
      for (l2 = (l1+1); l2<(nBlob-1); l2++) {
      for (l3 = (l2+1); l3<nBlob; l3++) {
        // This is n-choose-k permutations

        // Test 1: Radius ratio tolerance
        // Let's start with the computations that take the least amount of time
        // the 2.0 factor is to avoid the problematic case when the LEDS are very small
        RatioRadii = (std::min(std::min(KeptRadius[l1],KeptRadius[l2]),KeptRadius[l3])+2.5) / 
                   (std::max(std::max(KeptRadius[l1],KeptRadius[l2]),KeptRadius[l3])+2.5);
        if (std::abs(RatioRadii-1.0)>radius_ratio_tolerance) continue;

        // Ok now we have no choice. We have to compute the 3 vectors representing the 3-choose-2 combination of leds
        vec1 = KeptContoursPosition[l1]-KeptContoursPosition[l2];
        vec2 = KeptContoursPosition[l2]-KeptContoursPosition[l3];
        vec3 = KeptContoursPosition[l3]-KeptContoursPosition[l1];
        norm1 = cv::norm(vec1);
        norm2 = cv::norm(vec2);
        norm3 = cv::norm(vec3);

        // Test 2: Led intensity ratio
        RatioIntensity = std::min(KeptAvgIntensity[l1],std::min(KeptAvgIntensity[l2],KeptAvgIntensity[l3])) /
                       std::max(KeptAvgIntensity[l1],std::max(KeptAvgIntensity[l2],KeptAvgIntensity[l3]));
        
        if (std::abs(RatioIntensity-1.0)>intensity_ratio_tolerance) continue;

        // Test 3: Led Distance tolerance
        minDist = std::min(std::min(norm1,norm2),norm3);
        maxDist = std::max(std::max(norm1,norm2),norm3);
        RatioDistance = maxDist/minDist;
        if (std::abs(RatioDistance-distance_ratio)>distance_ratio_tolerance) continue;

        // Now we have to find the actual shortest arm
        if (minDist == norm1) shortArm = vec1;
        else if (minDist == norm2) shortArm = vec2;
        else shortArm = vec3;

        // Now we have to find the actual longest arm
        if (maxDist == norm1) longArm = vec1;
        else if (maxDist == norm2) longArm = vec2;
        else longArm = vec3;

        // Test 4: tolerance on the horizontal
        horizontal = longArm.dot(cv::Point(0,1))/(maxDist);
        if (std::abs(horizontal)>max_deviation_horizontal) continue;
            

        // Test 5: tolerance on the angle between the arms
        cosArms = std::abs(shortArm.dot(longArm)/(minDist*maxDist));
        //double cosArms2 = std::abs(shortArm.dot(longArm)/(cv::norm(shortArm)*cv::norm(longArm)));
        //printf("combo %d %d %d has passed first 3 tests, with cos=%.2f, %.2f!\n",l1, l2, l3,cosArms, cosArms2);
        double val = std::abs(std::abs(cosArms)-1.0);
        if (val>acos_tolerance) continue;
        

        // Test 6: we keep the one with the highest average intensity
        /*
        double sumIntensity = KeptAvgIntensity[l1]+KeptAvgIntensity[l2]+KeptAvgIntensity[l3];
        if (sumIntensity>maxIntensity) {
          maxIntensity = sumIntensity;
          FoundLEDs = true;
          BestCombo[0] = l1;
          BestCombo[1] = l2;
          BestCombo[2] = l3;
        }*/
        trioDot[0] = l1;
        trioDot[1] = l2;
        trioDot[2] = l3;
        BestCombo.push_back(trioDot);

      }
    }
  }

  dot_hypothesis_distorted.clear();
  dot_hypothesis_undistorted.clear();
  for(int id = 0; id < BestCombo.size(); id++){
    detection_centers.clear();
    // We then push the best results
    for (index = 0; index < 3; index++) {
      detection_centers.push_back(KeptContoursPosition[BestCombo[id][index]]);
    }

    
    // Order the dot from left to right
    cv::Point2f buf;
    for(int i = 0; i < 2; i++){
      if(detection_centers[0].x > detection_centers[1].x){
        buf = detection_centers[0];
        detection_centers[0] = detection_centers[1];
        detection_centers[1] = buf;
      }
      if(detection_centers[1].x > detection_centers[2].x){
        buf = detection_centers[1];
        detection_centers[1] = detection_centers[2];
        detection_centers[2] = buf;
      }
    }
    dot_hypothesis_distorted.push_back(detection_centers);

    // Undistort the points
    std::vector<cv::Point2f> undistorted_points;
    cv::undistortPoints(detection_centers, undistorted_points, camera_matrix_K, camera_distortion_coeffs, cv::noArray(),
                        camera_matrix_P);
    dot_hypothesis_undistorted.push_back(undistorted_points);

  } 
}


cv::Rect LEDDetector::determineROI(List2DPoints pixel_positions, cv::Size image_size, const int border_size,
                                   const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                                   const cv::Mat &camera_matrix_P)
{
  double x_min = INFINITY;
  double x_max = 0;
  double y_min = INFINITY;
  double y_max = 0;

  for (unsigned i = 0; i < pixel_positions.size(); ++i)
  {
    if (pixel_positions(i)(0) < x_min)
    {
      x_min = pixel_positions(i)(0);
    }
    if (pixel_positions(i)(0) > x_max)
    {
      x_max = pixel_positions(i)(0);
    }
    if (pixel_positions(i)(1) < y_min)
    {
      y_min = pixel_positions(i)(1);
    }
    if (pixel_positions(i)(1) > y_max)
    {
      y_max = pixel_positions(i)(1);
    }
  }

  std::vector<cv::Point2f> undistorted_points;

  undistorted_points.push_back(cv::Point2f(x_min, y_min));
  undistorted_points.push_back(cv::Point2f(x_max, y_max));

  std::vector<cv::Point2f> distorted_points;

  // Distort the points
  //distortPoints(undistorted_points, distorted_points, camera_matrix_K, camera_distortion_coeffs, camera_matrix_P);

  double x_min_dist = distorted_points[0].x;
  double y_min_dist = distorted_points[0].y;
  double x_max_dist = distorted_points[1].x;
  double y_max_dist = distorted_points[1].y;

  double x0 = std::max(0.0, std::min((double)image_size.width, x_min_dist - border_size));
  double x1 = std::max(0.0, std::min((double)image_size.width, x_max_dist + border_size));
  double y0 = std::max(0.0, std::min((double)image_size.height, y_min_dist - border_size));
  double y1 = std::max(0.0, std::min((double)image_size.height, y_max_dist + border_size));

  cv::Rect region_of_interest;

  // if region of interest is too small, use entire image
  // (this happens, e.g., if prediction is outside of the image)
  if (x1 - x0 < 1 || y1 - y0 < 1)
  {
    region_of_interest = cv::Rect(0, 0, image_size.width, image_size.height);
  }
  else
  {
    region_of_interest.x = x0;
    region_of_interest.y = y0;
    region_of_interest.width = x1 - x0;
    region_of_interest.height = y1 - y0;
  }

  return region_of_interest;
}

} // namespace
