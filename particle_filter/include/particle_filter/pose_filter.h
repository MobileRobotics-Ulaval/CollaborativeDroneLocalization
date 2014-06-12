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
    PoseFilter(const double distanceRightLedRobotA, const double distanceLeftLedRobotA,
               const double distanceRightLedRobotB, const double distanceLeftLedRobotB,
               const Eigen::Vector2d focalCam, const Eigen::Vector2d centerCam);
    double comparePoseABtoBA(const Eigen::Vector2d &pixelA1, const Eigen::Vector2d &pixelA2,
                             const Eigen::Vector2d &pixelB1, const Eigen::Vector2d &pixelB2);
private:
    Eigen::Vector2d m_focalCam, m_centerCam;
    double m_rdA, m_ldA, m_rdB, m_ldB;
};
}
#endif // POSE_FILTER_H
