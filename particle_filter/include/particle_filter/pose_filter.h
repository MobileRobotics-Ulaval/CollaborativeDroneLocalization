#ifndef POSE_FILTER_H
#define POSE_FILTER_H
#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <particle_filter/mutual_pose_estimation.h>

namespace particle_filter
{
class PoseFilter
{
public:
    PoseFilter();
    double comparePoseABtoBA(const Eigen::Vector2d &pixelA1, const Eigen::Vector2d &pixelA2,
                             const Eigen::Vector2d &pixelB1, const Eigen::Vector2d &pixelB2,
                             Eigen::Vector3d &positionAB,  Eigen::Matrix3d &rotationAB);
    void setMarkersParameters(const double distanceRightLedRobotA, const double distanceLeftLedRobotA,
                             const double distanceRightLedRobotB, const double distanceLeftLedRobotB);
    void setCameraParameters(const Eigen::Vector2d pFocalCam, const Eigen::Vector2d pCenterCam);
private:
    Eigen::Vector2d focalCam, centerCam;
    double rdA, ldA, rdB, ldB;
};
}
#endif // POSE_FILTER_H
