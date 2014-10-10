#include "drone_nav/autonomous_control.h"



void AutonomousControl::generateCommand(geometry_msgs::Twist &pTwistMsg,
                                        nav_msgs::Path &pPathMsg,
                                        const geometry_msgs::PoseStamped pCamPose){
    double yaw = this->getYawAngleFromQuaternion(pCamPose.pose.orientation);
    //ROS_INFO("YAW  Radians: %1.6f, Degree: %3.4f", yaw, yaw * 180.0/M_PI);

    Eigen::Vector3d dronePosition = this->getDroneCenterPostionFromCameraPosition(pCamPose.pose.position, yaw);

    Eigen::Vector3d direction = this->goalPosition - dronePosition;


    // Control only on the 2d plan no altitude control
    direction[2] = 0;

    // If the distance is larger than the goal Radius
    if(direction.norm() >= this->goalRadius){
        direction = 0.15 * direction/direction.norm();

        // Make it a matrix calculation?
        direction[0] = -(-direction[0]*cos(yaw) - direction[1]*sin(yaw));
        direction[1] = -(direction[0]*sin(yaw) - direction[1]*cos(yaw));
    }
    else{
        // Empty twistMsg
        direction = Eigen::Vector3d::Zero();
    }
    this->generateTwistMessage(pTwistMsg, direction);


    this->generatePathMessage(pPathMsg, dronePosition, direction);
}

void AutonomousControl::generateTwistMessage(geometry_msgs::Twist &pTwistMsg, const Eigen::Vector3d &pDirection){
    pTwistMsg.linear.x = pDirection[0];
    pTwistMsg.linear.y = pDirection[1];
    pTwistMsg.linear.z = pDirection[2];
}

// Path publishing for debugging purpose
void AutonomousControl::generatePathMessage(nav_msgs::Path &pPathMsg,
                                            const Eigen::Vector3d &pDronePosition,
                                            const Eigen::Vector3d &pDirection){
    geometry_msgs::PoseStamped dronePos = this->getPoseStampedFromEigenVector3d(pDronePosition);
    geometry_msgs::PoseStamped goalVector = this->getGoal();
    geometry_msgs::PoseStamped twistVisualisation = this->getPoseStampedFromEigenVector3d(pDronePosition - 3 * pDirection);

    pPathMsg.header.frame_id = "ardrone_base_link";
    dronePos.header.frame_id = "ardrone_base_link";
    twistVisualisation.header.frame_id = "ardrone_base_link";
    goalVector.header.frame_id = "ardrone_base_link";

    pPathMsg.poses.push_back(goalVector);
    pPathMsg.poses.push_back(dronePos);
    pPathMsg.poses.push_back(twistVisualisation);
}

visualization_msgs::Marker AutonomousControl::generateGoalMarkerMessage(){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "ardrone_base_link";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = this->goalRadius;
    marker.scale.y = this->goalRadius;
    marker.scale.z = this->goalRadius;
    marker.color.a = 1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.pose = this->getGoal().pose;

    return marker;
}
double AutonomousControl::getYawAngleFromQuaternion(const geometry_msgs::Quaternion &pQuaternionMsg){
    // The only angle of importance to us is the yaw,
    // we extract it using Quaternion to euler angle conversion
    // from the eigen librairy
    double x, y, z, w;
    x = pQuaternionMsg.x;
    y = pQuaternionMsg.y;
    z = pQuaternionMsg.z;
    w = pQuaternionMsg.w;
    Eigen::Quaterniond orientation(w,x,y,z);
    Eigen::Vector3d angle = orientation.toRotationMatrix().eulerAngles(0,1,2);
    return angle[2];
}

Eigen::Vector3d AutonomousControl::getDroneCenterPostionFromCameraPosition(const geometry_msgs::Point &pCamPositionMsg,
                                                                           const double &pYaw){
    Eigen::Vector3d camPosition(pCamPositionMsg.x,
                                pCamPositionMsg.y,
                                pCamPositionMsg.z);
   // std::cout << "Old :"<< camPosition.transpose() << std::endl;

    // The camera is 21cm from the drone's center
    Eigen::Vector3d camToCenter(0.21, 0, 0);
    // We rotate it
    camToCenter = Eigen::AngleAxisd(pYaw, Eigen::Vector3d::UnitZ()) * camToCenter;
    return camPosition -  camToCenter;
}

void AutonomousControl::setGoal(const geometry_msgs::Pose &pPose){
    this->goalPosition[0] = pPose.position.x;
    this->goalPosition[1] = pPose.position.y;
    this->goalPosition[2] = pPose.position.z;
}

geometry_msgs::PoseStamped AutonomousControl::getGoal(){
    return this->getPoseStampedFromEigenVector3d(this->goalPosition);
}


geometry_msgs::PoseStamped AutonomousControl::getPoseStampedFromEigenVector3d(Eigen::Vector3d pVect){
    geometry_msgs::PoseStamped poseMsg;
    poseMsg.pose.position.x = pVect[0];
    poseMsg.pose.position.y = pVect[1];
    poseMsg.pose.position.z = pVect[2];
    return poseMsg;
}

double AutonomousControl::getGoalRadius(){
    return this->goalRadius;
}

void AutonomousControl::setGoalRadius(double pGoalRadius){
    this->goalRadius = pGoalRadius;
}
