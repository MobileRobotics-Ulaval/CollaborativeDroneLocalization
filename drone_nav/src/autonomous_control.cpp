#include "drone_nav/autonomous_control.h"



geometry_msgs::Twist AutonomousControl::generateCommand(geometry_msgs::PoseStamped pCurrentPose){
    Eigen::Vector3d pos(pCurrentPose.pose.position.x,
                         pCurrentPose.pose.position.y,
                         pCurrentPose.pose.position.z);

    Eigen::Vector3d direction = this->poseGoal - pos;

    // The only angle of importance to us is the yaw,
    // we extract it using Quarternion to euler angle conversion
    // from the eigen librairy
    double x, y, z, w, yaw;
    x = pCurrentPose.pose.orientation.x;
    y = pCurrentPose.pose.orientation.y;
    z = pCurrentPose.pose.orientation.z;
    w = pCurrentPose.pose.orientation.w;
    Eigen::Quaterniond orientation(w,x,y,z);
    Eigen::Vector3d angle = orientation.toRotationMatrix().eulerAngles(0,1,2);
    yaw = angle[2];
    ROS_INFO("YAW  Radians: %1.6f, Degree: %3.4f", yaw, yaw*180.0/M_PI);



    // Control only on dept not altitude or lateral position
    //direction[1] = 0;
    direction[2] = 0;

    geometry_msgs::Twist twistMsg;
    // If the distance is larger than 1m
    if(direction.norm() >= 1){
        //twistMsg.linear.x = direction[0];
        twistMsg.linear.x = -direction[0]*cos(yaw) - direction[1]*sin(yaw);
        twistMsg.linear.y = direction[0]*sin(yaw) - direction[1]*cos(yaw);
    }
    else{
        // Empty twistMsg
    }

    return twistMsg;
}

void AutonomousControl::setGoal(geometry_msgs::Pose pPose){
    this->poseGoal[0] = pPose.position.x;
    this->poseGoal[1] = pPose.position.y;
    this->poseGoal[2] = pPose.position.z;
}
