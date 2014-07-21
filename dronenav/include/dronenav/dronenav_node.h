#ifndef DRONENAV_H
#define DRONENAV_H

#include "ros/ros.h"

#include <iostream>
#include <ardrone_autonomy/Navdata.h>
#include <map>

#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class DroneNav
{

public:
    DroneNav(ros::NodeHandle node);
    void joyCallBack(const sensor_msgs::Joy& joyMsg);
    void navCallback(const ardrone_autonomy::Navdata& navMsg);

private:
    void loop();
    void enableJoystick(const sensor_msgs::Joy& joyMsg);
    void disableJoystick();
    void createPublishers(const std::string& topic);
    void createSubscribers(const std::string& topic);
    void createServices(const std::string& topic);

    void waitingInitiation(ros::Rate loopRate);
    void getDroneState(int &tickCount, int maxCount);
    void takeoffObserver(ros::Rate loopRate, std_msgs::Empty emptyMsg);
    void landingObserver(ros::Rate loopRate, std_msgs::Empty emptyMsg);
    void resetObserver(ros::Rate loopRate, std_msgs::Empty emptyMsg);
    void movementObserver(ros::Rate loopRate);
    void flatTrimObserver();
    double altitudeObserver();
    void calibrateJoyAxis();

    ros::NodeHandle nodeHandle;
    static constexpr double MAX_SPEED = 0.5;
    int deadManSwitch;
    bool joyInitiated;
    bool navInitiated;
    std::map <std::string, bool> button;
    std::map <std::string, double> axis;
    ardrone_autonomy::Navdata droneData;

    ros::Publisher pubTwist;
    ros::Publisher pubReset;
    ros::Publisher pubLand;
    ros::Publisher pubTakeoff;
    ros::Subscriber subJoy;
    ros::Subscriber subNav;
    ros::ServiceClient clientFlatTrim;

};

#endif // DRONENAV_H
