#ifndef ARPE_NODE_H_
#define ARPE_NODE_H_

#include "ros/ros.h"

#include <iostream>
#include <fstream>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>

//#include <Eigen/Dense>
//#include <Eigen/Geometry>

#include <dynamic_reconfigure/server.h>
#include <ardrone_autonomy/Navdata.h>

#include <ardrone_pose_estimator/ardrone_pose_estimatorConfig.h>

namespace ardrone_pose_estimator
{
class ARPE{
public:
	ARPE(ros::NodeHandle node);
	void joyCallback(const sensor_msgs::Joy& joy_msg_in);
    void navCallback(const ardrone_autonomy::Navdata& msg_in);
	void loop();
	void dynamicParametersCallback(ardrone_pose_estimator::ardrone_pose_estimatorConfig &config, uint32_t level);
private:
    static const double m_max_speed = 0.5;
    static const double m_kp = 0.75;
    static const double m_kd = 0.75;
    bool m_buttonA, m_buttonB, m_buttonX, m_buttonR1, m_buttonR2;
    double m_axeLX, m_axeLY, m_axeRX, m_axeRY;
    sensor_msgs::Joy 		  m_joyState;	//!< Current joystick state
	ardrone_autonomy::Navdata m_drone;  //!< Current drone state (velocity, momentum, etc.)
    bool m_joyInitiated;
    bool m_navInitiated;


	//!< Publishing / subscribing variable >!
	ros::NodeHandle m_nodehandle;			//!< Ros nodehandler

	ros::Publisher  m_pub_twist; 			//!< The twist publisher
	ros::Publisher  m_pub_empty_reset; 		//!< Publisher for the reset state
	ros::Publisher  m_pub_empty_land; 		//!< Publisher for the landing command
	ros::Publisher  m_pub_empty_takeoff; 	//!< Publisher for the take off command
	ros::Publisher  m_pub_v3; 				//!< The speed control publisher
	ros::Subscriber m_joy_sub; 				//!< Subscriber to the joystick
	ros::Subscriber m_nav_sub; 				//!< Subscriber to the ARdrone state

	dynamic_reconfigure::Server<ardrone_pose_estimator::ardrone_pose_estimatorConfig> m_dr_server; 			//!< The dynamic reconfigure server
	dynamic_reconfigure::Server<ardrone_pose_estimator::ardrone_pose_estimatorConfig>::CallbackType m_cb; 	//!< The dynamic reconfigure callback type
};

}


#endif /* ARPE_NODE_H_ */
