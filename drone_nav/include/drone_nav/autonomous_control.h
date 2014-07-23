#ifndef AUTONOMOUS_CONTROL_H
#define AUTONOMOUS_CONTROL_H

#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class AutonomousControl
{

public:
    geometry_msgs::Twist generateCommand(geometry_msgs::PoseStamped pCurrentPose);
    void setGoal(geometry_msgs::Pose pPose);
private:
    Eigen::Vector3d poseGoal;
};

#endif // AUTONOMOUS_CONTROL_H
