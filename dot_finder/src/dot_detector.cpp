/*
 * dot_detector.cpp
 *
 *  Created on: Jun 15, 2014
 * Author: Philippe Babin
    Based on the work of Karl Schwabe
 */

/**
 * \file dot_detector.cpp
 * \brief File containing the function definitions required for detecting markers and visualising their detections and the pose of the tracked object.
 *
 */

#include "ros/ros.h"
#include "dot_finder/dot_detector.h"

using namespace std;
namespace dot_finder
{

typedef Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, 1> List2DPoints; //!< A dynamic column vector containing Vector2D elements. \see Vector2d

void DotDetector::setOrangeParameter(const int pHighH, const int pHighS, const int pHighV,
                                     const int pLowH,  const int pLowS,  const int pLowV){
    this->highHOrange = pHighH;
    this->highSOrange = pHighS;
    this->highVOrange = pHighV;
    this->lowHOrange  = pLowH;
    this->lowSOrange  = pLowS;
    this->lowVOrange  = pLowV;
}
void DotDetector::setBlueParameter(const int pHighH, const int pHighS, const int pHighV,
                                   const int pLowH,  const int pLowS,  const int pLowV){
    this->highHBlue = pHighH;
    this->highSBlue = pHighS;
    this->highVBlue = pHighV;
    this->lowHBlue  = pLowH;
    this->lowSBlue  = pLowS;
    this->lowVBlue  = pLowV;
}

void DotDetector::setDetectorParameter(const int pMin_radius, const int pMorph_type, const double pDilatation,
                          const double pErosion, const double pMax_angle, const double pMax_angle_duo,
                          const double pMax_norm_on_dist, const bool pMaskToggle){
    this->min_radius = pMin_radius;
    this->morph_type = pMorph_type;
    this->dilatation = pDilatation;
    this->erosion = pErosion;
    this->max_angle = pMax_angle;
    this->max_angle_duo = pMax_angle_duo;
    this->max_norm_on_dist = pMax_norm_on_dist;
    this->maskToggle = pMaskToggle;

}


void DotDetector::colorThresholding(cv::Mat & pImage,
                                    const int pHighH, const int pHighS, const int pHighV,
                                    const int pLowH,  const int pLowS,  const int pLowV){
    cv::Mat imgHSV;
    cv::cvtColor(pImage , imgHSV, CV_BGR2HSV); // Convert to color image for visualisation


    cv::Mat imgThresholded;

    if(pLowH > pHighH){// If the interval is in the around zero, we split the lower and higher part of the interval than we fusion them
        // We could also take the interval between the high and low and subtracts it to a white img
        cv::Mat lowImgThresholded;
        cv::Mat highImgThresholded;
        //cv::cvSet(whiteImg, CV_RGB(0,0,0));
        //cv::inRange(imgHSV, cv::Scalar(pHighH, pLowS, pLowV), cv::Scalar(pLowH, pHighS, pHighV), lowImgThresholded);
        //imgThresholded = whiteImg - lowImgThresholded;
        cv::inRange(imgHSV, cv::Scalar(pLowH, pLowS, pLowV), cv::Scalar(179, pHighS, pHighV), lowImgThresholded); //Lower part threshold

        cv::inRange(imgHSV, cv::Scalar(0, pLowS, pLowV), cv::Scalar(pHighH, pHighS, pHighV), highImgThresholded); //Higher part threshold

        //bitwise_or(lowImgThresholded, highImgThresholded, imgThresholded); //Fusion of the two range
        imgThresholded = lowImgThresholded + highImgThresholded; //Fusion of the two range

    }
    else{
        cv::inRange(imgHSV, cv::Scalar(pLowH, pLowS, pLowV), cv::Scalar(pHighH, pHighS, pHighV), imgThresholded); //Threshold the image
    }
    pImage = imgThresholded;
}

void DotDetector::LedFilteringArDrone(const cv::Mat &image,
                                      std::vector< std::vector<cv::Point2f> > & trio_distorted,
                                      std::vector< std::vector<cv::Point2f> > & dot_hypothesis_distorted, std::vector< std::vector<cv::Point2f> > & dot_hypothesis_undistorted,
                                      const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                                      const cv::Mat &camera_matrix_P, cv::Rect ROI) {

    //cv::Rect ROI = cv::Rect(0, 0, image.cols, image.rows);

    // If the ROI is bigger than the image, we resize it
    if(ROI.x + ROI.width > image.cols)
        ROI = cv::Rect(ROI.x, ROI.y, image.cols - ROI.x, ROI.height);
    if(ROI.y + ROI.height > image.rows)
        ROI = cv::Rect(ROI.x, ROI.y, ROI.width, image.rows - ROI.y);

    cv::Mat orangeMask = image.clone();
    cv::Mat blueMask = image.clone();
    //cv::Mat pouliotMask = image.clone();

    // Orange Thresholding
    colorThresholding(orangeMask, highHOrange, highSOrange, highVOrange,
                                  lowHOrange,  lowSOrange,  lowVOrange);

    // Blue Thresholding
    colorThresholding(blueMask, highHBlue, highSBlue, highVBlue,
                                lowHBlue,  lowSBlue,  lowVBlue);
    // Blue Thresholding
    /*colorThresholding(pouliotMask, 130, 55, 95,
                                120,  20,  65);
    blueMask = blueMask - pouliotMask - orangeMask;*/

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

    if(maskToggle)
        m_visualisationImg = blueMask;
    else
        m_visualisationImg = orangeMask;

    // Find all contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(orangeMask(ROI).clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // Vector for containing the detected points that will be undistorted later
    // std::vector<cv::Point2f> distorted_points;
    int DataIndex = 1; // For matlab-compatible output, when chasing the parameters.

    std::vector<cv::Point2f> KeptContoursPosition;
    std::vector<cv::Point2f> detection_centers;
    dot_hypothesis_distorted.clear();
    trio_distorted.clear();
    std::vector<double> KeptRadius, KeptAvgIntensity, KeptArea;

    // Identify the blobs in the image
    for (unsigned i = 0; i < contours.size(); i++)
    {
        cv::Rect rect = cv::boundingRect(contours[i]); // Bounding box
        //double radius = (rect.width + rect.height) / 4; // Average radius
        double radius = std::max(rect.width, rect.height) / 2.0;
        //double radius = (sqrt(rect.width*rect.width + rect.height*rect.height)); // Average radius

         cv::Moments mu;
         mu = cv::moments(contours[i], false);
         cv::Point2f mc;
         mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00) + cv::Point2f(ROI.x, ROI.y);
        double area = cv::contourArea(contours[i]); // Blob area

        if (area < 0.01){ continue; }

        if (true){
            // We want to output some data for further analysis in matlab.
            printf("%6.2f, %6.2f, %6.2f, %d\n",
                   mc.x, mc.y, area, radius, DataIndex);
                   //total_intensity, avg_intensity
            DataIndex++;
        }


        KeptArea.push_back(area);
        KeptContoursPosition.push_back(mc);
        KeptRadius.push_back(radius);
    }
    printf("\n");

    /*
    // Find the contours in the blue mask
    cv::findContours(blueMask(ROI).clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    std::vector<double> KeptAreaBlue,  KeptRadiusBlue;
    std::vector<cv::Point2f> blue_centers;
    for (unsigned i = 0; i < contours.size(); i++){

       double area = cv::contourArea(contours[i]); // Blob area

       if(area <= 0.01) continue;

       KeptAreaBlue.push_back(area); // Blob area

       cv::Moments mu = cv::moments(contours[i], false);
       blue_centers.push_back(cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00) + cv::Point2f(ROI.x, ROI.y));
       cv::Rect rect = cv::boundingRect(contours[i]);
       //KeptRadiusBlue.push_back((rect.width + rect.height) / 4);
       KeptRadiusBlue.push_back(std::max(rect.width, rect.height) / 2.0);
       //KeptRadiusBlue.push_back(sqrt(rect.width*rect.width + rect.height*rect.height));
    }

    */


    // ================= TRIO EXTRACTION =================
    vector<int> trio;
    vector< vector<int> > trioStack;
    double length, lengthSquare, radiusI, radiusJ, radiusISquare, radiusJSquare;
    cv::Point2f p0, p, centerTrio;
    for(int i = 0; i < KeptContoursPosition.size(); i++){ // First orange dot

        radiusI = KeptRadius[i] * 7;
        if(radiusI < min_radius){ radiusI = min_radius; }
        radiusISquare = radiusI * radiusI;

        p0 = KeptContoursPosition[i];
        for(int j = i + 1; j < KeptContoursPosition.size(); j++){ // Second orange dot

            radiusJ = KeptRadius[j] * 7;
            if(radiusJ < min_radius){ radiusJ = min_radius; }
            radiusJSquare = radiusJ * radiusJ;

            p = p0 - KeptContoursPosition[j];
            lengthSquare =  p.x*p.x + p.y*p.y;
            if(radiusISquare < lengthSquare || radiusJSquare < lengthSquare ) continue;

            length = sqrt(lengthSquare);
            //double maxArea = std::max(KeptArea[i], KeptArea[j]);
            //double minArea = std::min(KeptArea[i], KeptArea[j]);
            double maxArea = std::max(radiusI, radiusJ);
            double minArea = std::min(radiusI, radiusJ);
            printf("%3d vs. %3d => n=%6.2f rI=%6.2f rJ=%6.2f  minArea=%3.1f maxArea=%3.1f min/max=%1.3f\n", i+1, j+1, length, radiusI, radiusJ,
                   minArea,
                   maxArea,
                   minArea / maxArea);
            if(minArea/maxArea < 0.3) continue;

            if(atan(abs(p.y/p.x)) < max_angle){
                printf("\t angle remove \n");
                continue;
            }

            centerTrio = p * 0.5 + KeptContoursPosition[j];
            //int bluer = blueMask.data[blueMask.channels()*(blueMask.cols*centerTrio.x + centerTrio.y)];
            //uchar* rowi = blueMask.ptr/*<uchar>*/((int)centerTrio.x);
            //printf("\t blue: %i\n", rowi[(int)centerTrio.y]);
            int oshit = blueMask.channels()*(blueMask.cols*(int)centerTrio.x + (int)centerTrio.y);
            printf("\t blue: %i %i %i\n", blueMask.data[oshit], blueMask.data[oshit + 1], blueMask.data[oshit + 1]);


            // Find the closest blue dot
            /*
            int bDotId;
            double bestDist = -1;
            for(int k = 0; k < blue_centers.size(); k++){ // Blue dot between the orange dot
                double lengthASquare;
                p = p0 - blue_centers[k];
                lengthASquare =  p.x*p.x + p.y*p.y;
                if(radiusISquare <  lengthASquare) continue;

                double lengthBSquare;
                p = KeptContoursPosition[j] - blue_centers[k];
                lengthBSquare =  p.x*p.x + p.y*p.y;
                if(radiusJSquare <  lengthBSquare) continue;

                double maxY = std::max(KeptContoursPosition[j].y, KeptContoursPosition[i].y);
                double minY = std::min(KeptContoursPosition[j].y, KeptContoursPosition[i].y);
                if(blue_centers[k].y > minY && blue_centers[k].y < maxY){
                    //printf("\t\t r=%3f\n", distanceFromLineToPoint(blue_centers[k],  KeptContoursPosition[i], KeptContoursPosition[j]));
                    double d = distanceFromLineToPoint(blue_centers[k],  KeptContoursPosition[i], KeptContoursPosition[j]);
                    if(bestDist == -1 || d < bestDist){
                        bestDist = d;
                        bDotId = k;
                        k = blue_centers.size();
                    }
                }
            }*/
            // If a blue dot was detected
           // if(bestDist != -1){

                detection_centers.clear();
                detection_centers.push_back(KeptContoursPosition[i]); // First Orange dot
                detection_centers.push_back(KeptContoursPosition[j]); // Second Orange dot
                //detection_centers.push_back(blue_centers[bDotId]);         // Blue dot
                detection_centers.push_back(centerTrio);         // Blue dot
                detection_centers.push_back(cv::Point2f(radiusI, i+1));     // DEBUG FOR VISUALISATION
                detection_centers.push_back(cv::Point2f(radiusJ, j+1));     // DEBUG FOR VISUALISATION
                trio_distorted.push_back(detection_centers);


                trio.clear();
                trio.push_back(i);      // First Orange dot
                trio.push_back(j);      // Second Orange dot
                trioStack.push_back(trio);
            /*}
            else{
                printf("\t blue remove \n");
            }*/
        }
    }

    printf("\n");

    double ratioDotOnCameraPlan;
    // The old shell had 1.5, the new one is 0.5
    ros::param::get("~ratio", ratioDotOnCameraPlan);

    cv::Point2f topI, botI, topJ, botJ;
    cv::Point2f topV, botV, centerV;
    cv::Point2f centerI, centerJ;
    double topAngle, botAngle, centerAngle;
    // Each trio is pair with another one
    for(int i = 0; i < trioStack.size(); i++){
        if(KeptContoursPosition[trioStack[i][0]].y  < KeptContoursPosition[trioStack[i][1]].y){
            topI = KeptContoursPosition[trioStack[i][0]];
            botI = KeptContoursPosition[trioStack[i][1]];
        }
        else{
            topI = KeptContoursPosition[trioStack[i][1]];
            botI = KeptContoursPosition[trioStack[i][0]];
        }

        for(int j = i + 1; j < trioStack.size(); j++){
            if(KeptContoursPosition[trioStack[j][0]].y  < KeptContoursPosition[trioStack[j][1]].y){
                topJ = KeptContoursPosition[trioStack[j][0]];
                botJ = KeptContoursPosition[trioStack[j][1]];
            }
            else{
                topJ = KeptContoursPosition[trioStack[j][1]];
                botJ = KeptContoursPosition[trioStack[j][0]];
            }
            centerI = (topI - botI) * 0.5 + botI;
            centerJ = (topJ - botJ) * 0.5 + botJ;
            centerV = centerI - centerJ;
            centerAngle = atan(abs(centerV.y / centerV.x)) - atan(abs((topI - botI).x / (topI - botI).y));


            if(centerAngle > max_angle_duo){ continue;}

            double distance =  norm(centerV);
            double radiusI = KeptRadius[trioStack[i][0]] + KeptRadius[trioStack[i][1]];
            double radiusJ = KeptRadius[trioStack[j][0]] + KeptRadius[trioStack[j][1]];

            // This is the most effective filter
            double normOnDist = (norm(topI - botI) + norm(topJ - botJ))*0.5/distance;
            if(normOnDist > max_norm_on_dist ){  printf("\t norm reject=> %6.2f\n", normOnDist); continue;}
            printf("%3d-%3d vs %3d-%3d => ctrA=%6.2f radI/dist=%6.2f radJ/dist=%6.2f min/max=%6.2f  normOnDist=%6.2f \n",
                   trioStack[i][0] +1, trioStack[i][1] +1,
                   trioStack[j][0] +1, trioStack[i][1] +1,
                   centerAngle*180/M_PI,
                   radiusI/distance, radiusJ/distance,
                   min(radiusI,radiusJ)/max(radiusI,radiusJ),
                   normOnDist);

            if(min(radiusI,radiusJ)/max(radiusI,radiusJ) < 0.40){ printf("\t min/max Radius\n"); continue;}

            double ratioLenght = min(norm(topI - botI),norm(topJ - botJ))/max(norm(topI - botI),norm(topJ - botJ));

            if(ratioLenght < 0.66){ printf("\t ratioLength (%6.2f)\n ", ratioLenght); continue;}
            if(radiusI/distance < 0.05 || radiusJ/distance < 0.05 ){ printf("\t min/max (too small)\n"); continue;}
            if(radiusI/distance > 0.25 || radiusJ/distance > 0.25 ){ printf("\t min/max (too big)\n"); continue;}


            detection_centers.clear();
            //detection_centers.push_back(centerI); // First Orange dot
            //detection_centers.push_back(centerJ); // Second Orange dot

            // Those dots form a line with the camera
            cv::Point2f onCameraPlaneI = (botI-topI) * ratioDotOnCameraPlan + topI;
            cv::Point2f onCameraPlaneJ = (botJ-topJ) * ratioDotOnCameraPlan + topJ;
            if(botI.x < botJ.x){
                detection_centers.push_back(onCameraPlaneI); // Left Orange dot
                detection_centers.push_back(onCameraPlaneJ); // Right Orange dot
            }
            else{
                detection_centers.push_back(onCameraPlaneJ); // Left Orange dot
                detection_centers.push_back(onCameraPlaneI); // Right Orange dot
            }
            detection_centers.push_back(topI);         // Blue dot
            dot_hypothesis_distorted.push_back(detection_centers);
        }
    }
    printf("Avant %3d Après %3d\n", trioStack.size(), dot_hypothesis_distorted.size());


    // Undistort the pointsima
    dot_hypothesis_undistorted.clear();
    for(int i = 0; i < dot_hypothesis_distorted.size(); i++){
        std::vector<cv::Point2f> undistorted_points;
        cv::undistortPoints(dot_hypothesis_distorted[i], undistorted_points, camera_matrix_K, camera_distortion_coeffs, cv::noArray(),
                            camera_matrix_P);
        dot_hypothesis_undistorted.push_back(undistorted_points);
    }

}

double DotDetector::distanceFromLineToPoint(const cv::Point2f p, const cv::Point2f lineA, const cv::Point2f lineB){
    double A, B, C;
    A = lineA.y - lineB.y;
    B = lineB.x - lineA.x;
    C = lineA.x * lineB.y - lineB.x * lineA.y;

    return abs(A*p.x + B*p.y + C)/sqrt(A*A + B*B);

}

} // namespace
