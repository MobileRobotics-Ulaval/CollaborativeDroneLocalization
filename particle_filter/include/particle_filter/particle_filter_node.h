#ifndef PARTICLE_FILTER_NODE_H_
#define PARTICLE_FILTER_NODE_H_

#include "ros/ros.h"

#include <iostream>
#include <fstream>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <dynamic_reconfigure/server.h>
#include <particle_filter/ParticleFilterConfig.h>
#include <dot_finder/DuoDot.h>



#include <particle_filter/region_of_interest.h>
#include <particle_filter/mutual_pose_estimation.h>

namespace particle_filter
{
class ParticleFilter{
public:
	ParticleFilter(ros::NodeHandle n);
    void leaderDotsCallback(const dot_finder::DuoDot::ConstPtr& dots_msg);
    void followerDotsCallback(const dot_finder::DuoDot::ConstPtr& dots_msg);
    void leaderImuCallback(const sensor_msgs::Imu::ConstPtr& pLeader_imu_msg);
    void followerImuCallback(const sensor_msgs::Imu::ConstPtr& pFollower_imu_msg);
	void dynamicParametersCallback(particle_filter::ParticleFilterConfig &config, uint32_t level);

	void visualizationCallbackLeader(const sensor_msgs::Image::ConstPtr& image_msg);
	void visualizationCallbackFollower(const sensor_msgs::Image::ConstPtr& image_msg);

    void generateCSVLog();

private:
    void createPublishers(const std::string& topic_leader, const std::string& topic_follower);
    void createSubscribers(const std::string& topic_leader, const std::string& topic_follower);
    std::vector<Eigen::Vector2d> fromROSPoseArrayToVector2d(std::vector<geometry_msgs::Pose2D> ros_msg);
    std::vector<cv::Point2f> fromROSPoseArrayToCvPoint(std::vector<geometry_msgs::Pose2D> ros_msg);
    void runParticleFilter();
    bool isAllMessageInitiated();
    void updateCameraParametersFromLastMessage();

    sensor_msgs::ImagePtr generateAndPublishROIVisualization(int idROI, cv::Mat &img);
    void drawDistortedMarker(int idROI, cv::Mat &img);
    void drawRegionOfInterest(int idROI, cv::Mat &img);

    bool followerDotsInitiation;
    bool leaderImuInitiation, followerImuInitiation;
    dot_finder::DuoDot leaderLastMsg, followerLastMsg;
    sensor_msgs::Imu leaderImuMsg, followerImuMsg;
    RegionOfInterest regionOfInterest[2];

    geometry_msgs::PoseArray candidatesPoseMsgs;

    MutualPoseEstimation poseEvaluator;

    ros::Subscriber subDotsLeader;
    ros::Subscriber subDotsFollower;
    ros::Subscriber subImuLeader;
    ros::Subscriber subImuFollower;
    ros::Subscriber subVisualizationLeader;
    ros::Subscriber subVisualizationFollower;

    ros::Publisher pubPose;
    ros::Publisher pubMarker;
    ros::Publisher pubMarkerCandidates;
    ros::Publisher pubPoseCandidates;
    image_transport::Publisher pubROILeader;
    image_transport::Publisher pubROIFollower;

    ros::NodeHandle nodeHandler;

    dynamic_reconfigure::Server<particle_filter::ParticleFilterConfig> dynamicReconfigServer; //!< The dynamic reconfigure server
    dynamic_reconfigure::Server<particle_filter::ParticleFilterConfig>::CallbackType dynamicReconfigCallback; //!< The dynamic reconfigure callback type

};

}


#endif /* PARTICLE_FILTER_NODE_H_ */
