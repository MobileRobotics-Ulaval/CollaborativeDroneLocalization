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

using namespace std;
namespace dot_finder
{

typedef Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> List2DPoints; //!< A dynamic column vector containing Vector2D elements. \see Vector2d

void LEDDetector::setOrangeParameter(const int pHighH, const int pHighS, const int pHighV,
                                     const int pLowH,  const int pLowS,  const int pLowV){
    m_highHOrange = pHighH;
    m_highSOrange = pHighS;
    m_highVOrange = pHighV;
    m_lowHOrange  = pLowH;
    m_lowSOrange  = pLowS;
    m_lowVOrange  = pLowV;
}
void LEDDetector::setBlueParameter(const int pHighH, const int pHighS, const int pHighV,
                                   const int pLowH,  const int pLowS,  const int pLowV){
    m_highHBlue = pHighH;
    m_highSBlue = pHighS;
    m_highVBlue = pHighV;
    m_lowHBlue  = pLowH;
    m_lowSBlue  = pLowS;
    m_lowVBlue  = pLowV;
}

void LEDDetector::colorThresholding(cv::Mat & pImage,
                                    const int pHighH, const int pHighS, const int pHighV,
                                    const int pLowH,  const int pLowS,  const int pLowV){
    cv::Mat imgHSV;
    cv::cvtColor(pImage , imgHSV, CV_BGR2HSV); // Convert to color image for visualisation


    cv::Mat imgThresholded;

    if(pLowH > pHighH){// If the interval is in the around zero, we split the lower and higher part of the interval than we fusion them
        cv::Mat lowImgThresholded;
        cv::Mat highImgThresholded;
        cv::inRange(imgHSV, cv::Scalar(pLowH, pLowS, pLowV), cv::Scalar(179, pHighS, pHighV), lowImgThresholded); //Lower part threshold

        cv::inRange(imgHSV, cv::Scalar(0, pLowS, pLowV), cv::Scalar(pHighH, pHighS, pHighV), highImgThresholded); //Higher part threshold

        bitwise_or(lowImgThresholded, highImgThresholded, imgThresholded); //Fusion of the two range
    }
    else{
        cv::inRange(imgHSV, cv::Scalar(pLowH, pLowS, pLowV), cv::Scalar(pHighH, pHighS, pHighV), imgThresholded); //Threshold the image
    }
    pImage = imgThresholded;
}

void LEDDetector::LedFilteringArDrone(const cv::Mat &image, const int &min_radius, const int &morph_type, const double &dilatation, const double &erosion, const double &min_blob_area, const double &max_blob_area,
                                      const double &intensity_ratio_tolerance, const double &max_deviation_horizontal,
                                      std::vector< std::vector<cv::Point2f> > & dot_hypothesis_distorted, std::vector< std::vector<cv::Point2f> > & dot_hypothesis_undistorted,
                                      const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                                      const cv::Mat &camera_matrix_P, bool debug) {
    cv::Rect ROI = cv::Rect(0, 0, image.cols, image.rows);

    cv::Mat orangeMask = image.clone();
    cv::Mat blueMask = image.clone();
    //cv::Mat greenMask = image.clone();
    // Orange Thresholding
    colorThresholding(orangeMask, m_highHOrange, m_highSOrange, m_highVOrange,
                   m_lowHOrange, m_lowSOrange, m_lowVOrange);

    // Blue Thresholding
    colorThresholding(blueMask, m_highHBlue, m_highSBlue, m_highVBlue,
                    m_lowHBlue, m_lowSBlue, m_lowVBlue);

    int dilation_type;
    if( morph_type == 0 ){      dilation_type = cv::MORPH_RECT; }
    else if( morph_type == 1 ){ dilation_type = cv::MORPH_CROSS; }
    else if( morph_type == 2) { dilation_type = cv::MORPH_ELLIPSE; }

    if(dilatation > 0)
        cv::dilate(orangeMask, orangeMask, cv::getStructuringElement(dilation_type, cv::Size( 2*dilatation +1, 2*dilatation+1),
                                                                                          cv::Point( -1, -1)) );

    if(erosion > 0)
        cv::erode (orangeMask, orangeMask, cv::getStructuringElement(dilation_type, cv::Size( 2*erosion+1, 2*erosion+1),
                                                                                          cv::Point( erosion, erosion)) );

    // Green Thresholding to remove
    /*
    colorThresholding(greenMask, 89, 139, 237,
                                 69, 18,  93);
    bitwise_not(greenMask, greenMask);
    bitwise_and(blueMask, greenMask, greenMask); // We remove the green part from the blue mask
    */

    if(debug)
        m_visualisationImg = blueMask;
    else
        m_visualisationImg = orangeMask;

    // Find all contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(orangeMask.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // Vector for containing the detected points that will be undistorted later
    // std::vector<cv::Point2f> distorted_points;
    int DataIndex = 1; // For matlab-compatible output, when chasing the parameters.

    std::vector<cv::Point2f> KeptContoursPosition;
    std::vector<cv::Point2f> detection_centers;
    dot_hypothesis_distorted.clear();
    std::vector<double> KeptRadius, KeptAvgIntensity, KeptArea;

    // Identify the blobs in the image
    for (unsigned i = 0; i < contours.size(); i++)
    {
        cv::Rect rect = cv::boundingRect(contours[i]); // Bounding box
        double radius = (rect.width + rect.height) / 4; // Average radius
        //double radius = (sqrt(rect.width*rect.width + rect.height*rect.height)); // Average radius

         cv::Moments mu;
         mu = cv::moments(contours[i], false);
         cv::Point2f mc;
         mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00) + cv::Point2f(ROI.x, ROI.y);
        double area = cv::contourArea(contours[i]); // Blob area

        if (area < 0.01){ continue; }

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
                cv::Scalar intensity = orangeMask.at<uchar>(y, x);
                total_intensity+= float(intensity.val[0]);
            }
        }
        avg_intensity = total_intensity/area;

        if (false){
            // We want to output some data for further analysis in matlab.
            printf("%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %8.2f, %6.2f, %.2f, %d\n",
                   mc.x, mc.y, area, radius,
                   width_height_distortion, circular_distortion1, circular_distortion2,
                   total_intensity, avg_intensity ,RatioEllipse, DataIndex);
            DataIndex++;
        }


        // We will prune the blobs set based on appearance. These were found using matlab.
        //if ((area < max_blob_area) && (circular_distortion1 < max_circular_distortion) && (circular_distortion2 < max_circular_distortion)
        //      && (RatioEllipse < max_ratio_ellipse) && (RatioEllipse > min_ratio_ellipse )) {
        // These will be used further down the filtering pipeline
        // Ideally we would sort in order of intensity, and do a n choose k on a sliding window of that ranked intensity.
        KeptArea.push_back(area);
        KeptContoursPosition.push_back(mc);
        KeptRadius.push_back(radius);
        KeptAvgIntensity.push_back(avg_intensity);
        // }
    }
    printf("\n");

    // Find the contours in the blue mask
    cv::findContours(blueMask.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    std::vector<double> KeptAreaBlue,  KeptRadiusBlue;
    std::vector<cv::Point2f> blue_centers;
    for (unsigned i = 0; i < contours.size(); i++){

       double area = cv::contourArea(contours[i]); // Blob area

       if(area <= 0.01) continue;

       KeptAreaBlue.push_back(area); // Blob area

       cv::Moments mu = cv::moments(contours[i], false);
       blue_centers.push_back(cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00));
       cv::Rect rect = cv::boundingRect(contours[i]);
       KeptRadiusBlue.push_back((rect.width + rect.height) / 4);
       //KeptRadiusBlue.push_back(sqrt(rect.width*rect.width + rect.height*rect.height));
    }
    // Extract the trio
    vector<int> trio;
    vector< vector<int> > trioStack;
    double length, lengthSquare, radiusI, radiusJ, radiusISquare, radiusJSquare;
    cv::Point2f p0, p;
    for(int i = 0; i < KeptContoursPosition.size(); i++){
        p0 = KeptContoursPosition[i];
        radiusI = KeptRadius[i] * 7;
        if(radiusI < min_radius)
            radiusI = min_radius;
        radiusISquare = radiusI * radiusI;
        for(int j = i + 1; j < KeptContoursPosition.size(); j++){

            radiusJ = KeptRadius[j] * 7;
            if(radiusJ < min_radius)
                radiusJ = min_radius;
            radiusJSquare = radiusJ * radiusJ;

            p = p0 - KeptContoursPosition[j];
            lengthSquare =  p.x*p.x + p.y*p.y;
            if(radiusISquare > lengthSquare && radiusJSquare > lengthSquare ){
                length = sqrt(lengthSquare);
                //double maxArea = std::max(KeptArea[i], KeptArea[j]);
                //double minArea = std::min(KeptArea[i], KeptArea[j]);
                double maxArea = std::max(radiusI, radiusJ);
                double minArea = std::min(radiusI, radiusJ);
                /*printf("%3d vs. %3d => n=%6.2f rI=%6.2f rJ=%6.2f  minArea=%3.1f maxArea=%3.1f min/max=%1.3f\n", i+1, j+1, length, radiusI, radiusJ,
                       minArea,
                       maxArea,
                       minArea / maxArea);*/
                if(minArea/maxArea >= 0.3){

                    // Find the closest blue dot
                    int bDotId;
                    double bestDist = -1;
                    for(int k = 0; k < blue_centers.size(); k++){
                        double lengthA;
                        p = p0 - blue_centers[k];
                        lengthA =  sqrt(p.x*p.x + p.y*p.y);
                        if(radiusI >  lengthA){
                            double lengthB;
                            p = KeptContoursPosition[j] - blue_centers[k];
                            lengthB =  sqrt(p.x*p.x + p.y*p.y);
                            if(radiusJ >  lengthB){
                                double maxX = std::max(KeptContoursPosition[j].x, KeptContoursPosition[i].x);
                                double minX = std::min(KeptContoursPosition[j].x, KeptContoursPosition[i].x);
                                if(blue_centers[k].x > minX && blue_centers[k].x < maxX){
                                    //printf("\t\t r=%3f\n", distanceFromLineToPoint(blue_centers[k],  KeptContoursPosition[i], KeptContoursPosition[j]));
                                    double d = distanceFromLineToPoint(blue_centers[k],  KeptContoursPosition[i], KeptContoursPosition[j]);
                                    if(bestDist == -1 || d < bestDist){
                                        bestDist = d;
                                        bDotId = k;
                                        k = blue_centers.size();
                                    }
                                }
                            }
                        }
                    }
                    // If a blue dot was detected
                    if(bestDist != -1){
                        trio.clear();
                        trio.push_back(i);      // First Orange dot
                        trio.push_back(j);      // Second Orange dot
                        trio.push_back(bDotId); // Blue dot
                        trio.push_back(radiusI); // debug
                        trioStack.push_back(trio);
                    }
                }
            }
            /*else if(length < 50){
                printf("%3d vs. %3d => n=%6.2f rI=%6.2f rJ=%6.2f **\n", i+1, j+1, length);
                trio.clear();
                trio.push_back(i);      // First Orange dot
                trio.push_back(j);      // Second Orange dot
                trio.push_back(0); // Blue dot
                trio.push_back(radiusI); // debug
                trioStack.push_back(trio);
            }*/
        }

        // Let's remove the orange's dot that form a trio
        for(int i = 0; i < trioStack.size(); i++){
            trio = trioStack[i];
            detection_centers.clear();
            detection_centers.push_back(KeptContoursPosition[trio[0]]); // First Orange dot
            detection_centers.push_back(KeptContoursPosition[trio[1]]); // Second Orange dot
            detection_centers.push_back(blue_centers[trio[2]]);         // Blue dot
            detection_centers.push_back(cv::Point2f(trio[3], trio[0]+1));     // DEBUG FOR VISUALISATION
            dot_hypothesis_distorted.push_back(detection_centers);
        }

        // KeptContoursPosition.erase(KeptContoursPosition.begin() + j);
        // KeptArea.erase(KeptArea.begin() + j);
        // KeptRadius.erase(KeptRadius.begin() + j);

        // If we have a pair with similar radius
        //if(best/KeptRadius[i] < 10){

           // printf("%6.2f %6.2f %d *\n", best, best/KeptRadius[i], i+1);
       // }
       // else{
       //     printf("%6.2f %6.2f %d\n", best, best/KeptRadius[i], i+1);

       // }
    }



    /*
    double radius, length, diffArea, best;
    cv::Point2f p0, p;
    for(int i = 0; i < KeptContoursPosition.size(); i++){
        p0 = KeptContoursPosition[i];
        int blueDot = 0;
        best = -1;
        radius = KeptRadius[i] * 5;
        if(radius < 15.0)
            radius = 15.0;
        for(int k = 0; k < blue_centers.size(); k++){
            p = p0 - blue_centers[k];
            length =  sqrt(p.x*p.x + p.y*p.y);
            diffArea = KeptAreaBlue[k]/KeptArea[i];
            if(length < radius ){//&& diffArea < 3.0
                blueDot++;
                if(best == -1 || diffArea < best)
                    best = diffArea;
            }
        }
        if(blueDot > 0){
            printf("%d, %6.2f, %d *\n", blueDot, best, i+1);
            detection_centers.clear();
            detection_centers.push_back(p0);
            detection_centers.push_back(cv::Point2f(radius,0));
            dot_hypothesis_distorted.push_back(detection_centers);
        }
        else{
            printf("%d, %6.2f, %d\n", blueDot, KeptRadius[i] * 5, i+1);
        }
     }*/

  // Filtering step #2: doing all the permutations to find the one with the best characteristics.
  // We are basically looking at:
  //   -Three blobs in line (line angle tolerance)
  //   -With a 3-to-1 ratio between the shortest distance to the longest distance
  //   -And similar radius
  /*
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
  */
}

double LEDDetector::distanceFromLineToPoint(const cv::Point2f p, const cv::Point2f lineA, const cv::Point2f lineB){
    double A, B, C;
    A = lineA.y - lineB.y;
    B = lineB.x - lineA.x;
    C = lineA.x * lineB.y - lineB.x * lineA.y;

    return abs(A*p.x + B*p.y + C)/sqrt(A*A + B*B);

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
