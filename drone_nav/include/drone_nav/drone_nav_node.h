#ifndef DRONE_NAV_H
#define DRONE_NAV_H

#include "ros/ros.h"

#include <iostream>
#include <ardrone_autonomy/Navdata.h>
#include <map>

#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Joy.h>
#include "drone_nav/autonomous_control.h"

class DroneNav
{

public:
    DroneNav(ros::NodeHandle node);
    void joyCallBack(const sensor_msgs::Joy& joyMsg);
    void navCallback(const ardrone_autonomy::Navdata& navMsg);
    void poseCallback(const geometry_msgs::PoseStamped& poseMsg);

private:
    void loop();
    void enableJoystick(const sensor_msgs::Joy& joyMsg);
    void disableJoystick();
    void createPublishers(const std::string& topic);
    void createSubscribers(const std::string& topic);
    void createServices(const std::string& topic);

    void waitingInitiation(ros::Rate loopRate);
    void getDroneState(int &tickCount, int maxCount);
    void takeoffObserver(ros::Rate loopRate);
    void landingObserver(ros::Rate loopRate);
    void resetObserver(ros::Rate loopRate);
    void controlObserver();
    void movementObserver();
    void autonomousControlObserver();
    void flatTrimObserver();
    double altitudeObserver();
    void calibrateJoyAxis();

    ros::NodeHandle nodeHandle;
    static const double MAX_SPEED = 0.5;
    int deadManSwitch;
    bool joyInitiated;
    bool navInitiated;
    bool poseInitiated;
    std::map <std::string, bool> button;
    std::map <std::string, double> axis;
    ardrone_autonomy::Navdata droneData;
    geometry_msgs::PoseStamped poseData;

    ros::Publisher pubTwist;
    ros::Publisher pubReset;
    ros::Publisher pubLand;
    ros::Publisher pubTakeoff;
    ros::Publisher pubPath;
    ros::Subscriber subJoy;
    ros::Subscriber subNav;
    ros::Subscriber subPose;
    ros::ServiceClient clientFlatTrim;

    AutonomousControl autoCtrl;

};

#endif // DRONE_NAV_H
