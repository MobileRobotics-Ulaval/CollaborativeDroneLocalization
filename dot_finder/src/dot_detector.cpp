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
    // We could also take the interval between the high and low and subtracts it to a white image
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

void DotDetector::trainingDataAcquiring(const cv::Mat &image,
                                        std::vector< std::vector<cv::Point2f> > & trio_distorted){
    cv::Rect ROI(0,0, image.cols, image.rows);
    cv::Mat orangeMask = image.clone();

    this->colorThresholdingDilateErode(orangeMask);

    this->visualisationImg = orangeMask;

    // TODO Need to become attribute
    trio_distorted.clear();

    this->findImageFeature(orangeMask, ROI);

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
            if(radiusI*10 < distBetweenContour || radiusJ*10 < distBetweenContour) continue;
            if(atan(abs(p.y/p.x)) < max_angle) continue;

            trio.clear();
            trio.push_back(i);      // First Orange dot
            trio.push_back(j);      // Second Orange dot
            trioStack.push_back(trio);


            // For visualization
            feature_visualization.clear();
            feature_visualization.push_back(this->contoursPosition[i]); // First Orange dot
            feature_visualization.push_back(this->contoursPosition[j]); // Second Orange dot
            feature_visualization.push_back(cv::Point2f(trioStack.size() - 1, j));     // DEBUG FOR VISUALISATION
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
    for(int i = 0; i < this->trioStack.size(); i++){
        if(std::find(trioPositive.begin(), trioPositive.end(), i) == trioPositive.end()){
            myfile << this->generateDataLine(time, trioStack[i]);
        }

    }
    myfile.close();
    ROS_INFO("%s saved.", filename_neg.c_str());

    myfile.open(filename_pos.c_str(), ios::app);
    for(int i = 0; i < trioPositive.size(); i++){
        int id = trioPositive[i];
        if(!isOutOfRangeDot(id)){
            myfile << this->generateDataLine(time, this->trioStack[id]);
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
    return id < 0 || id >= this->trioStack.size();
}

//Todo add pParameter
void DotDetector::dotFilteringArDrone(const cv::Mat &image,
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
    //this->keptArea.clear();
    this->contoursPosition.clear();
    //this->keptRadius.clear();

    std::map <std::string, double> features;
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

        features.clear();
        features["area"] = area;
        features["radius"] = radius;
        features["width"] = rect.width;
        features["height"] = rect.height;
        this->contoursFeatures.push_back(features);
        this->contoursPosition.push_back(mc);
    }
}

void DotDetector::extractImageTrio(vector< vector<cv::Point2f> > & trio_distorted){
    vector<int> trio;
    std::vector<cv::Point2f> feature_visualization;
    this->trioStack.clear();
    double length, lengthSquare, radiusI, radiusJ, radiusISquare, radiusJSquare;
    cv::Point2f p0, p;
    for(int i = 0; i < this->contoursPosition.size(); i++){ // First orange dot

        radiusI = this->contoursFeatures[i]["radius"] * 7;
        if(radiusI < min_radius){ radiusI = min_radius; }
        radiusISquare = radiusI * radiusI;

        p0 = this->contoursPosition[i];
        for(int j = i + 1; j < this->contoursPosition.size(); j++){ // Second orange dot

            radiusJ = this->contoursFeatures[j]["radius"] * 7;
            if(radiusJ < min_radius){ radiusJ = min_radius; }
            radiusJSquare = radiusJ * radiusJ;

            p = p0 - this->contoursPosition[j];
            lengthSquare =  p.x*p.x + p.y*p.y;
            if(radiusISquare < lengthSquare || radiusJSquare < lengthSquare ) continue;

            length = sqrt(lengthSquare);

            double maxArea = std::max(radiusI, radiusJ);
            double minArea = std::min(radiusI, radiusJ);
            printf("%3d vs. %3d => n=%6.2f rI=%6.2f rJ=%6.2f  minArea=%3.1f maxArea=%3.1f min/max=%1.3f\n", i+1, j+1, length, radiusI, radiusJ,
                   minArea,
                   maxArea,
                   minArea / maxArea);
            //Mathlab prove that isn't a good metric
            //if(minArea/maxArea < 0.3) continue;

            if(atan(abs(p.y/p.x)) < max_angle){
                printf("\t angle remove \n");
                continue;
            }

            //centerTrio = p * 0.5 + this->contoursPosition[j];

            // We add the trio to the stack
            trio.clear();
            trio.push_back(i);      // First Orange dot
            trio.push_back(j);      // Second Orange dot
            trioStack.push_back(trio);


            // For visualization
            feature_visualization.clear();
            feature_visualization.push_back(this->contoursPosition[i]); // First Orange dot
            feature_visualization.push_back(this->contoursPosition[j]); // Second Orange dot
            feature_visualization.push_back(cv::Point2f(i + 1, j + 1)); // DEBUG FOR VISUALISATION
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
        int idIA = trioStack[i][0];
        int idIB = trioStack[i][1];
        if(this->contoursPosition[trioStack[i][0]].y  < this->contoursPosition[trioStack[i][1]].y){
            topI = this->contoursPosition[idIA];
            botI = this->contoursPosition[idIB];
        }
        else{
            topI = this->contoursPosition[idIB];
            botI = this->contoursPosition[idIA];
        }

        for(int j = i + 1; j < trioStack.size(); j++){
            int idJA = trioStack[j][0];
            int idJB = trioStack[j][1];
            if(this->contoursPosition[trioStack[j][0]].y  < this->contoursPosition[trioStack[j][1]].y){
                topJ = this->contoursPosition[idJA];
                botJ = this->contoursPosition[idJB];
            }
            else{
                topJ = this->contoursPosition[idJB];
                botJ = this->contoursPosition[idJA];
            }
            centerI = (topI - botI) * 0.5 + botI;
            centerJ = (topJ - botJ) * 0.5 + botJ;
            centerV = centerI - centerJ;
            centerAngle = atan(abs(centerV.y / centerV.x)) - atan(abs((topI - botI).x / (topI - botI).y));


            if(centerAngle > max_angle_duo){ continue;}

            double distance =  norm(centerV);
            double radiusI = this->contoursFeatures[idIA]["radius"] + this->contoursFeatures[idIB]["radius"];
            double radiusJ = this->contoursFeatures[idJA]["radius"] + this->contoursFeatures[idJB]["radius"];

            // This is the most effective filter
            double normOnDist = (norm(topI - botI) + norm(topJ - botJ))*0.5/distance;
            if(normOnDist > max_norm_on_dist ){  printf("\t norm reject=> %6.2f\n", normOnDist); continue;}
            printf("%3d-%3d vs %3d-%3d => ctrA=%6.2f radI/dist=%6.2f radJ/dist=%6.2f min/max=%6.2f  normOnDist=%6.2f \n",
                   idIA +1, idIB +1,
                   idJA +1, idJB +1,
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
