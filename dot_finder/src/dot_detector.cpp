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

void DotDetector::setCameraParameter(const cv::Mat pCameraMatrixK,
                                     const cv::Mat pCameraMatrixP,
                                     const vector<double> pCameraDistortionCoeffs){
    this->cameraMatrixK = pCameraMatrixK;
    this->cameraMatrixP = pCameraMatrixP;
    this->cameraDistortionCoeffs = pCameraDistortionCoeffs;
}


void DotDetector::doColorThresholding(cv::Mat & pImage,
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
void DotDetector::ledFilteringArDrone(const cv::Mat &image,
                                      std::vector< std::vector<cv::Point2f> > & trio_distorted,
                                      std::vector< std::vector<cv::Point2f> > & dot_hypothesis_distorted,
                                      std::vector< std::vector<cv::Point2f> > & dot_hypothesis_undistorted,
                                      cv::Rect ROI) {
    this->resizeRegionOfInterest(image.cols, image.rows, ROI);

    cv::Mat orangeMask = image.clone();

    this->colorThresholdingDilateErode(orangeMask);

    this->visualisationImg = orangeMask;

    // TODO Need to become attribute
    trio_distorted.clear();

    this->findImageFeature(orangeMask, ROI);
    //printf("\n");

    this->extractImageTrio(trio_distorted);
    printf("\n");

    dot_hypothesis_distorted = this->paringTrio();
    dot_hypothesis_undistorted = this->removeCameraDistortion(dot_hypothesis_distorted);
}

void DotDetector::colorThresholdingDilateErode(cv::Mat &image){
    this->doColorThresholding(image, highHOrange, highSOrange, highVOrange,
                                  lowHOrange,  lowSOrange,  lowVOrange);

    if(dilatation > 0)
        cv::dilate(image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size( 2*dilatation +1, 2*dilatation+1),                                                                                   cv::Point( -1, -1)) );
    if(erosion > 0)
        cv::erode (image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size( 2*erosion+1, 2*erosion+1),
                                                                                          cv::Point( erosion, erosion)) );
}


void DotDetector::findImageFeature(const cv::Mat &image, cv::Rect ROI){
    this->keptArea.clear();
    this->keptContoursPosition.clear();
    this->keptRadius.clear();
    // Find all contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(image(ROI).clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    int DataIndex = 1;

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

         /*
        // We want to output some data for further analysis in matlab.
        printf("%6.2f, %6.2f, %6.2f, %d\n",
               mc.x, mc.y, area, radius, DataIndex);
               //total_intensity, avg_intensity
        DataIndex++;*/

        this->keptArea.push_back(area);
        this->keptContoursPosition.push_back(mc);
        this->keptRadius.push_back(radius);
    }
}

void DotDetector::extractImageTrio(vector< vector<cv::Point2f> > & trio_distorted){
    vector<int> trio;
    std::vector<cv::Point2f> feature_visualization;
    this->trioStack.clear();
    double length, lengthSquare, radiusI, radiusJ, radiusISquare, radiusJSquare;
    cv::Point2f p0, p, centerTrio;
    for(int i = 0; i < this->keptContoursPosition.size(); i++){ // First orange dot

        radiusI = this->keptRadius[i] * 7;
        if(radiusI < min_radius){ radiusI = min_radius; }
        radiusISquare = radiusI * radiusI;

        p0 = this->keptContoursPosition[i];
        for(int j = i + 1; j < this->keptContoursPosition.size(); j++){ // Second orange dot

            radiusJ = this->keptRadius[j] * 7;
            if(radiusJ < min_radius){ radiusJ = min_radius; }
            radiusJSquare = radiusJ * radiusJ;

            p = p0 - this->keptContoursPosition[j];
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

            centerTrio = p * 0.5 + this->keptContoursPosition[j];

            // We add the trio to the stack
            trio.clear();
            trio.push_back(i);      // First Orange dot
            trio.push_back(j);      // Second Orange dot
            trioStack.push_back(trio);


            // For visualization
            feature_visualization.clear();
            feature_visualization.push_back(this->keptContoursPosition[i]); // First Orange dot
            feature_visualization.push_back(this->keptContoursPosition[j]); // Second Orange dot
            feature_visualization.push_back(centerTrio);         // Blue dot
            feature_visualization.push_back(cv::Point2f(radiusI, i+1));     // DEBUG FOR VISUALISATION
            feature_visualization.push_back(cv::Point2f(radiusJ, j+1));     // DEBUG FOR VISUALISATION
            trio_distorted.push_back(feature_visualization);
        }
    }

}

std::vector<std::vector<cv::Point2f> > DotDetector::paringTrio(){

    std::vector<cv::Point2f> feature_visualization;
    std::vector< std::vector<cv::Point2f> > pairedTrio;
    double ratioDotOnCameraPlan;
    // The old shell had 1.5 (The position of the dot is under the marker)
    // the new one is 0.5 (Between the two orange dot)
    ros::param::get("~ratio", ratioDotOnCameraPlan);

    // TODO Use eigen instead of cv or use norm?
    cv::Point2f topI, botI, topJ, botJ;
    cv::Point2f centerV;
    cv::Point2f centerI, centerJ;
    double centerAngle;
    // Each trio is pair with another one
    for(int i = 0; i < trioStack.size(); i++){
        if(this->keptContoursPosition[trioStack[i][0]].y  < this->keptContoursPosition[trioStack[i][1]].y){
            topI = this->keptContoursPosition[trioStack[i][0]];
            botI = this->keptContoursPosition[trioStack[i][1]];
        }
        else{
            topI = this->keptContoursPosition[trioStack[i][1]];
            botI = this->keptContoursPosition[trioStack[i][0]];
        }

        for(int j = i + 1; j < trioStack.size(); j++){
            if(this->keptContoursPosition[trioStack[j][0]].y  < this->keptContoursPosition[trioStack[j][1]].y){
                topJ = this->keptContoursPosition[trioStack[j][0]];
                botJ = this->keptContoursPosition[trioStack[j][1]];
            }
            else{
                topJ = this->keptContoursPosition[trioStack[j][1]];
                botJ = this->keptContoursPosition[trioStack[j][0]];
            }
            centerI = (topI - botI) * 0.5 + botI;
            centerJ = (topJ - botJ) * 0.5 + botJ;
            centerV = centerI - centerJ;
            centerAngle = atan(abs(centerV.y / centerV.x)) - atan(abs((topI - botI).x / (topI - botI).y));


            if(centerAngle > max_angle_duo){ continue;}

            double distance =  norm(centerV);
            double radiusI = this->keptRadius[trioStack[i][0]] + this->keptRadius[trioStack[i][1]];
            double radiusJ = this->keptRadius[trioStack[j][0]] + this->keptRadius[trioStack[j][1]];

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


            feature_visualization.clear();

            // Those dots form a line with the camera
            cv::Point2f onCameraPlaneI = (botI-topI) * ratioDotOnCameraPlan + topI;
            cv::Point2f onCameraPlaneJ = (botJ-topJ) * ratioDotOnCameraPlan + topJ;
            if(botI.x < botJ.x){
                feature_visualization.push_back(onCameraPlaneI); // Left Orange dot
                feature_visualization.push_back(onCameraPlaneJ); // Right Orange dot
            }
            else{
                feature_visualization.push_back(onCameraPlaneJ); // Left Orange dot
                feature_visualization.push_back(onCameraPlaneI); // Right Orange dot
            }
            feature_visualization.push_back(topI);         // Blue dot
            pairedTrio.push_back(feature_visualization);
        }
    }
    printf("Avant %3d Après %3d\n", trioStack.size(), pairedTrio.size());

    return pairedTrio;
}


std::vector< std::vector<cv::Point2f> > DotDetector::removeCameraDistortion(std::vector< std::vector<cv::Point2f> > & distortedPoints){
    std::vector< std::vector<cv::Point2f> > dot_hypothesis_undistorted;
    for(int i = 0; i < distortedPoints.size(); i++){
        std::vector<cv::Point2f> setOfpoints;
        cv::undistortPoints(distortedPoints[i], setOfpoints, this->cameraMatrixK, this->cameraDistortionCoeffs, cv::noArray(),
                            this->cameraMatrixP);
        dot_hypothesis_undistorted.push_back(setOfpoints);
    }
    return dot_hypothesis_undistorted;
}


} // namespace
