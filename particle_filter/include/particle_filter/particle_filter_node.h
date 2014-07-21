#ifndef PARTICLE_FILTER_NODE_H_
#define PARTICLE_FILTER_NODE_H_

#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <dynamic_reconfigure/server.h>
#include <particle_filter/ParticleFilterConfig.h>
#include <dot_finder/DuoDot.h>


#include <particle_filter/pose_filter.h>
#include <particle_filter/mutual_pose_estimation.h>

namespace particle_filter
{
class ParticleFilter{
public:
	ParticleFilter(ros::NodeHandle n);
	void leaderCallback(const dot_finder::DuoDot::ConstPtr& dots_msg);
	void followerCallback(const dot_finder::DuoDot::ConstPtr& dots_msg);
    std::vector<Eigen::Vector2d> fromROSPoseArrayToVector2d(std::vector<geometry_msgs::Pose2D> ros_msg);

	void dynamicParametersCallback(particle_filter::ParticleFilterConfig &config, uint32_t level);

	
private:
	bool m_follower_initiation;
	dot_finder::DuoDot m_follower_last_msg;

	ros::Subscriber m_leader_dots_pub;
	ros::Subscriber m_follower_dots_pub;

	ros::Publisher m_pose_pub;
    ros::Publisher m_marker_pub;
    ros::Publisher m_marker_candidate_pub;

	ros::NodeHandle m_nodeHandler;


	dynamic_reconfigure::Server<particle_filter::ParticleFilterConfig> m_dr_server; //!< The dynamic reconfigure server
	dynamic_reconfigure::Server<particle_filter::ParticleFilterConfig>::CallbackType m_cb; //!< The dynamic reconfigure callback type

	double m_rdA, m_ldA, m_rdB, m_ldB;

};

}


#endif /* PARTICLE_FILTER_NODE_H_ */
