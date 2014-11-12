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


void DotDetector::setRatioDotOnCameraPlan(double pRatioDotOnCameraPlan){
    this->ratioDotOnCameraPlan = pRatioDotOnCameraPlan;
}


void DotDetector::setDetectorParameter(const double pThreshold,
                                        const double pDilatation, const double pErosion,
                                       const double pMax_angle,
                                       const double pMinRadius){
    this->threshold = pThreshold;
    this->dilatation = pDilatation;
    this->erosion = pErosion;
    this->max_angle = pMax_angle;
    this->min_radius = pMinRadius;

}

void DotDetector::setCameraParameter(const cv::Mat pCameraMatrixK,
                                     const cv::Mat pCameraMatrixP,
                                     const vector<double> pCameraDistortionCoeffs){
    this->cameraMatrixK = pCameraMatrixK;
    this->cameraMatrixP = pCameraMatrixP;
    this->cameraDistortionCoeffs = pCameraDistortionCoeffs;
}


void DotDetector::doThresholding(cv::Mat & pImage){
    /* 0: Binary
        1: Binary Inverted
        2: Threshold Truncated
        3: Threshold to Zero
        4: Threshold to Zero Inverted
      */
    ROS_INFO("thres: %i", this->threshold);
    cv::threshold( pImage, pImage, this->threshold, 255, 3);
}

// If the ROI is bigger than the image, we resize it
void DotDetector::resizeRegionOfInterest(const int colsImg, const int rowsImg, cv::Rect & ROI){
    if(ROI.x + ROI.width > colsImg)
        ROI = cv::Rect(ROI.x, ROI.y, colsImg - ROI.x, ROI.height);
    if(ROI.y + ROI.height > rowsImg)
        ROI = cv::Rect(ROI.x, ROI.y, ROI.width, rowsImg - ROI.y);
}


/*************************************************************************************
 Detect duo of marker with almost no filtering, so it can later be convert to a Mathlab compatible file format.
 *************************************************************************************/

void DotDetector::trainingDataAcquiring(const cv::Mat &image,
                                        std::vector< std::vector<cv::Point2f> > & trio_distorted){
    cv::Rect ROI(0,0, image.cols, image.rows);
    cv::Mat orangeMask = image.clone();

    this->colorThresholdingDilateErode(orangeMask);

    this->visualisationImg = orangeMask;

    // TODO Need to become attribute
    pairStack.clear();
    trio_distorted.clear();

    this->extractContourAndFeatureFromImage(orangeMask, ROI);

    cv::Point2f p0, p;
    double distBetweenContour;
    std::vector<int> trio;
    std::vector<cv::Point2f> feature_visualization;
    double radiusI, radiusJ;
    for(int i = 0; i < this->contoursPosition.size(); i++){ // First orange dot

        radiusI = this->contoursFeatures[i]["radius"];

        p0 = this->contoursPosition[i];
        for(int j = i + 1; j < this->contoursPosition.size(); j++){ // Second orange dot

            radiusJ = this->contoursFeatures[j]["radius"];
            p = p0 - this->contoursPosition[j];
            distBetweenContour = cv::norm(p0 - this->contoursPosition[j]);
            //if(radiusI*10 < distBetweenContour || radiusJ*10 < distBetweenContour) continue;
            //if(atan(abs(p.y/p.x)) < max_angle) continue;

            trio.clear();
            trio.push_back(i);      // First Orange dot
            trio.push_back(j);      // Second Orange dot
            pairStack.push_back(trio);


            // For visualization
            feature_visualization.clear();
            feature_visualization.push_back(this->contoursPosition[i]); // First Orange dot
            feature_visualization.push_back(this->contoursPosition[j]); // Second Orange dot
            feature_visualization.push_back(cv::Point2f(pairStack.size() - 1, j));     // DEBUG FOR VISUALISATION
            trio_distorted.push_back(feature_visualization);
        }
    }
}

void DotDetector::saveToCSV(vector<int> trioPositive){
    string filename_pos, filename_neg;
    filename_pos  = "data_positive_raw.csv";
    filename_neg  = "data_negative_raw.csv";

    uint64_t time = ros::Time::now().toNSec();

    ofstream myfile;
    myfile.open(filename_neg.c_str(), ios::app);
    for(int i = 0; i < this->pairStack.size(); i++){
        if(std::find(trioPositive.begin(), trioPositive.end(), i) == trioPositive.end()){
            myfile << this->generateDataLine(time, pairStack[i]);
        }

    }
    myfile.close();
    ROS_INFO("%s saved.", filename_neg.c_str());

    myfile.open(filename_pos.c_str(), ios::app);
    for(int i = 0; i < trioPositive.size(); i++){
        int id = trioPositive[i];
        if(!isOutOfRangeDot(id)){
            myfile << this->generateDataLine(time, this->pairStack[id]);
        }
    }
    myfile.close();
    ROS_INFO("%s saved.", filename_pos.c_str());

}

string DotDetector::generateDataLine(uint64_t time, std::vector<int> contoursId){
    std::stringstream r;
    r << time << ","
      << this->contoursPosition[contoursId[0]].x << ","
      << this->contoursPosition[contoursId[0]].y<< ","
      << this->contoursPosition[contoursId[1]].x << ","
      << this->contoursPosition[contoursId[1]].y;

    std::map <std::string, double> firstDot  = this->contoursFeatures[ contoursId[0] ];
    std::map <std::string, double> secondDot = this->contoursFeatures[ contoursId[1] ];
    for(map<string,double>::iterator it = firstDot.begin(); it != firstDot.end(); ++it) {
        r << "," << it->second;
    }
    for(map<string,double>::iterator it = secondDot.begin(); it != secondDot.end(); ++it) {
        r << "," << it->second;
    }

    r << "\n";

    //ROS_INFO("%s", r.str().c_str());
    return r.str();
}

bool DotDetector::isOutOfRangeDot(int id){
    return id < 0 || id >= this->pairStack.size();
}

/*******************************************
            Markers extraction
 *******************************************/
void DotDetector::dotFilteringCube(const cv::Mat &image,
                                      std::vector< std::vector<cv::Point2f> > & trio_distorted,
                                      std::vector< std::vector<cv::Point2f> > & dot_hypothesis_distorted,
                                      std::vector< std::vector<cv::Point2f> > & dot_hypothesis_undistorted,
                                      cv::Rect ROI) {
    this->resizeRegionOfInterest(image.cols, image.rows, ROI);

    cv::Mat imgWithContour = image.clone();

    this->colorThresholdingDilateErode(imgWithContour);

    this->visualisationImg = imgWithContour;

    // TODO Need to become attribute
    trio_distorted.clear();

    this->extractContourAndFeatureFromImage(imgWithContour, ROI);
    //printf("\n");

    dot_hypothesis_distorted =  this->extractPairFromContour(trio_distorted);
    printf("\n");

    //dot_hypothesis_distorted = this->paringBlobPair();
    dot_hypothesis_undistorted = this->removeCameraDistortion(dot_hypothesis_distorted);
}

void DotDetector::colorThresholdingDilateErode(cv::Mat &image){
    this->doThresholding(image);

    if(dilatation > 0)
        cv::dilate(image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size( 2*dilatation +1, 2*dilatation+1),                                                                                   cv::Point( -1, -1)) );
    if(erosion > 0)
        cv::erode (image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size( 2*erosion+1, 2*erosion+1),
                                                                                          cv::Point( erosion, erosion)) );
}


void DotDetector::extractContourAndFeatureFromImage(const cv::Mat &image, cv::Rect ROI){
    //this->keptArea.clear();
    this->contoursPosition.clear();
    this->contoursFeatures.clear();
    //this->keptRadius.clear();

    std::map <std::string, double> features;
    // Find all contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(image(ROI).clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // Identify the blobs in the image
    for (unsigned i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]); // Blob area

        if (area < 0.01){ continue; } // If blob area too small

        cv::Rect rect = cv::boundingRect(contours[i]); // Bounding box
        //double radius = (rect.width + rect.height) / 4; // Average radius
        double radius = (rect.width + rect.height) * 0.5;
        //double radius = (sqrt(rect.width*rect.width + rect.height*rect.height)); // Average radius

         cv::Moments mu;
         mu = cv::moments(contours[i], false);
         cv::Point2f mc;
         mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00) + cv::Point2f(ROI.x, ROI.y);

        features.clear();
        features["area"] = area;
        features["radius"] = radius;
        features["width"] = rect.width;
        features["height"] = rect.height;
        this->contoursFeatures.push_back(features);
        this->contoursPosition.push_back(mc);
    }
}

std::vector<std::vector<cv::Point2f> > DotDetector::extractPairFromContour(vector< vector<cv::Point2f> > & trio_distorted){
    vector<int> blob;
    std::vector< std::vector<cv::Point2f> > pairedBlob;
    std::vector<cv::Point2f> feature_visualization;
    this->pairStack.clear();
    float length,
            areaI, areaJ,
            radiusI, radiusJ, dRadius,
            minRadius, maxRadius,
            minArea, maxArea;
    cv::Point2f p0, p;
    ROS_INFO("radius=%4.1f", min_radius);
    for(int i = 0; i < this->contoursPosition.size(); i++){ // First orange dot

        radiusI = this->contoursFeatures[i]["radius"];
        areaI = this->contoursFeatures[i]["area"];

        p0 = this->contoursPosition[i];
        for(int j = i + 1; j < this->contoursPosition.size(); j++){ // Second orange dot

            radiusJ = this->contoursFeatures[j]["radius"];

            p = p0 - this->contoursPosition[j];
            length =  cv::norm(p);

            minRadius = std::min(radiusI, radiusJ);
            maxRadius = std::max(radiusI, radiusJ);
            dRadius = abs(maxRadius-minRadius);

            ROS_INFO("(%d <=> %d) : minRadius=%4.3f length=%4.1f", i, j, minRadius, length);
            //ROS_INFO("length: %f minRadius: %f", length, minRadius);
            //Doesn't work at close range for some reason...
            //if(length*0.6 - 12  > minRadius){ROS_INFO("Fail 1"); continue;} // Minimun Radius in function of norm

            if(atan(abs(p.y/ p.x)) > max_angle)continue; // Angle thresholding

            if(minRadius < min_radius)continue; // Radius min thresholding

            /*if(length*2.6 - dRadius -14 < 0)continue; // Difference min/maxRadius function of norm

            areaJ = this->contoursFeatures[j]["area"];
            minArea = std::min(areaI, areaJ);
            maxArea = std::max(areaI, areaJ);
            if(length*8.2 - abs(maxArea-minArea) -41 < 0) continue; // Difference min/maxArea function of norm

            if(length*length*0.2 - minArea + 30 < 0) continue; // Minium Area in function of norm
            */

            // We add the trio to the stack
            blob.clear();
            blob.push_back(i);      // First dot
            blob.push_back(j);      // Second dot
            pairStack.push_back(blob);

            // For visualization
            feature_visualization.clear();
            if(this->contoursPosition[i].y > this->contoursPosition[j].y){
                feature_visualization.push_back(this->contoursPosition[i]); // Left blob
                feature_visualization.push_back(this->contoursPosition[j]); // Right blob
            }
            else{
                feature_visualization.push_back(this->contoursPosition[j]); // Left blob
                feature_visualization.push_back(this->contoursPosition[i]); // Right blob
            }
            //feature_visualization.push_back(this->contoursPosition[i]); // First blob
            //feature_visualization.push_back(this->contoursPosition[j]); // Second blob
            feature_visualization.push_back(cv::Point2f(i + 1, j + 1)); // DEBUG FOR VISUALISATION
            trio_distorted.push_back(feature_visualization);

            pairedBlob.push_back(feature_visualization);
        }
    }
    return pairedBlob;
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
