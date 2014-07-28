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

    // If the interval is in the around zero, we split the lower and higher part of the interval than we fusion them
    if(pLowH > pHighH){
        this->redHueThresholding(imgHSV, pHighH, pHighS, pHighV, pLowH, pLowS, pLowV);
    }
    else{
        cv::inRange(imgHSV, cv::Scalar(pLowH, pLowS, pLowV), cv::Scalar(pHighH, pHighS, pHighV), imgHSV); //Threshold the image
    }
    pImage = imgHSV;
}

void DotDetector::redHueThresholding(cv::Mat & pImgHSV,
                                     const int pHighH, const int pHighS, const int pHighV,
                                     const int pLowH,  const int pLowS,  const int pLowV){
    // We could also take the interval between the high and low and subtracts it to a white img
    cv::Mat lowImgThresholded;
    cv::Mat highImgThresholded;

    cv::inRange(pImgHSV, cv::Scalar(pLowH, pLowS, pLowV), cv::Scalar(179, pHighS, pHighV), lowImgThresholded); //Lower part threshold

    cv::inRange(pImgHSV, cv::Scalar(0, pLowS, pLowV), cv::Scalar(pHighH, pHighS, pHighV), highImgThresholded); //Higher part threshold
    pImgHSV = lowImgThresholded + highImgThresholded; //Fusion of the two range
}


// If the ROI is bigger than the image, we resize it
void DotDetector::resizeRegionOfInterest(const int colsImg, const int rowsImg, cv::Rect & ROI){
    if(ROI.x + ROI.width > colsImg)
        ROI = cv::Rect(ROI.x, ROI.y, colsImg - ROI.x, ROI.height);
    if(ROI.y + ROI.height > rowsImg)
        ROI = cv::Rect(ROI.x, ROI.y, ROI.width, rowsImg - ROI.y);
}

//Todo add pParameter
void DotDetector::LedFilteringArDrone(const cv::Mat &image,
                                      std::vector< std::vector<cv::Point2f> > & trio_distorted,
                                      std::vector< std::vector<cv::Point2f> > & dot_hypothesis_distorted,
                                      std::vector< std::vector<cv::Point2f> > & dot_hypothesis_undistorted,
                                      const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
                                      const cv::Mat &camera_matrix_P, cv::Rect ROI) {
    this->resizeRegionOfInterest(image.cols, image.rows, ROI);

    cv::Mat orangeMask = image.clone();
    cv::Mat blueMask = image.clone();
    //cv::Mat pouliotMask = image.clone();

    // Orange Thresholding
    this->colorThresholding(orangeMask, highHOrange, highSOrange, highVOrange,
                                  lowHOrange,  lowSOrange,  lowVOrange);
    /*
    // Blue Thresholding
    colorThresholding(blueMask, highHBlue, highSBlue, highVBlue,
                                lowHBlue,  lowSBlue,  lowVBlue);*/
    int dilation_type;
    // TODO put in function
    if( morph_type == 0 ){      dilation_type = cv::MORPH_RECT; }
    else if( morph_type == 1 ){ dilation_type = cv::MORPH_CROSS; }
    else if( morph_type == 2) { dilation_type = cv::MORPH_ELLIPSE; }

    if(dilatation > 0)
        cv::dilate(orangeMask, orangeMask, cv::getStructuringElement(dilation_type, cv::Size( 2*dilatation +1, 2*dilatation+1),
                                                                                          cv::Point( -1, -1)) );

    if(erosion > 0)
        cv::erode (orangeMask, orangeMask, cv::getStructuringElement(dilation_type, cv::Size( 2*erosion+1, 2*erosion+1),
                                                                                          cv::Point( erosion, erosion)) );
    // Keep it?
    if(maskToggle){
        this->visualisationImg = blueMask;
     }
    else{
        this->visualisationImg = orangeMask;
    }

    // Find all contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(orangeMask(ROI).clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    int DataIndex = 1;

    // TODO put in function
    std::vector<cv::Point2f> KeptContoursPosition;
    std::vector<cv::Point2f> detection_centers;
    dot_hypothesis_distorted.clear();
    trio_distorted.clear();
    std::vector<double> KeptRadius, KeptAvgIntensity, KeptArea;

    // Identify the blobs in the image
    for (unsigned i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]); // Blob area

        if (area < 0.01){ continue; } // If blob area too small

        cv::Rect rect = cv::boundingRect(contours[i]); // Bounding box
        //double radius = (rect.width + rect.height) / 4; // Average radius
        double radius = std::max(rect.width, rect.height) / 2.0;
        //double radius = (sqrt(rect.width*rect.width + rect.height*rect.height)); // Average radius

         cv::Moments mu;
         mu = cv::moments(contours[i], false);
         cv::Point2f mc;
         mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00) + cv::Point2f(ROI.x, ROI.y);

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


    // TODO put in function
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

            // We add the trio to the stack
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
        }
    }

    printf("\n");

    double ratioDotOnCameraPlan;
    // The old shell had 1.5 (The position of the dot is under the marker)
    // the new one is 0.5 (Between the two orange dot)
    ros::param::get("~ratio", ratioDotOnCameraPlan);

    // TODO a) Put in function
    //      b) Use eigen instead of cv or use norm?
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

            if(min(radiusI,radiusJ)/max(radiusI,radiusJ) < 0.35){ printf("\t min/max Radius\n"); continue;}

            double ratioLenght = min(norm(topI - botI),norm(topJ - botJ))/max(norm(topI - botI),norm(topJ - botJ));

            if(ratioLenght < 0.66){ printf("\t ratioLength (%6.2f)\n ", ratioLenght); continue;}
            if(radiusI/distance < 0.05 || radiusJ/distance < 0.05 ){ printf("\t min/max (too small)\n"); continue;}
            if(radiusI/distance > 0.25 || radiusJ/distance > 0.25 ){ printf("\t min/max (too big)\n"); continue;}


            detection_centers.clear();

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


    // Undistord the duo hypothesis
    dot_hypothesis_undistorted.clear();
    for(int i = 0; i < dot_hypothesis_distorted.size(); i++){
        std::vector<cv::Point2f> undistorted_points;
        cv::undistortPoints(dot_hypothesis_distorted[i], undistorted_points, camera_matrix_K, camera_distortion_coeffs, cv::noArray(),
                            camera_matrix_P);
        dot_hypothesis_undistorted.push_back(undistorted_points);
    }

}


} // namespace
