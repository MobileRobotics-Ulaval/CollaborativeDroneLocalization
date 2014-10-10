#ifndef AUTONOMOUS_CONTROL_H
#define AUTONOMOUS_CONTROL_H

#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class AutonomousControl
{

public:
    void generateCommand(geometry_msgs::Twist &pTwistMsg,
                         nav_msgs::Path &pPathMsg,
                         const geometry_msgs::PoseStamped pCamPose);
    visualization_msgs::Marker generateGoalMarkerMessage();
    void setGoal(const geometry_msgs::Pose &pPose);
    geometry_msgs::PoseStamped getGoal();
    geometry_msgs::PoseStamped getPoseStampedFromEigenVector3d(Eigen::Vector3d pVect);

    double getGoalRadius();
    void setGoalRadius(double pGoalRadius);
private:
    void generateTwistMessage(geometry_msgs::Twist &pTwistMsg,
                              const Eigen::Vector3d &pDirection);
    void generatePathMessage(nav_msgs::Path &pPathMsg,
                             const Eigen::Vector3d &pDronePosition,
                             const Eigen::Vector3d &pDirection);
    double getYawAngleFromQuaternion(const geometry_msgs::Quaternion &pQuartenionMsg);
    Eigen::Vector3d getDroneCenterPostionFromCameraPosition(const geometry_msgs::Point &pCamPositionMsg,
                                                            const double &pYaw);


    double goalRadius = 0.7;
    static const double MAX_SPEED = 0.2;
    Eigen::Vector3d goalPosition;
};
#endif // AUTONOMOUS_CONTROL_H
