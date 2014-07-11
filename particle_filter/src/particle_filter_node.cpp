#include "particle_filter/particle_filter_node.h"

using namespace std;

namespace particle_filter
{
ParticleFilter::ParticleFilter(ros::NodeHandle n) : 
    m_nodeHandler(n),
    m_follower_initiation(false)
  {
  string topic_leader, topic_follower, topic_poses;
  ros::param::get("~leader", topic_leader);
  ros::param::get("~follower", topic_follower);

  ROS_INFO("Subscribing to %s and %s", topic_leader.c_str(), topic_follower.c_str());

  topic_leader = topic_leader + "/dots";
  topic_follower = topic_follower + "/dots";
  topic_poses = topic_leader + "/pose";

  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  dynamic_reconfigure::Server<particle_filter::ParticleFilterConfig>::CallbackType m_cb;
  m_cb = boost::bind(&ParticleFilter::dynamicParametersCallback, this, _1, _2);
  m_dr_server.setCallback(m_cb);

  // Initialize subscribers
  m_leader_dots_pub = m_nodeHandler.subscribe(topic_leader, 1, &ParticleFilter::leaderCallback, this);
  m_follower_dots_pub = m_nodeHandler.subscribe(topic_follower, 1, &ParticleFilter::followerCallback, this);

  // Initialize detected leds publisher
  m_pose_pub = m_nodeHandler.advertise<geometry_msgs::PoseStamped>(topic_poses, 1);
}
void ParticleFilter::followerCallback(const dot_finder::DuoDot::ConstPtr& follower_msg){
	if(!m_follower_initiation)
		m_follower_initiation = true;

	m_follower_last_msg = *follower_msg;

}

void ParticleFilter::leaderCallback(const dot_finder::DuoDot::ConstPtr& leader_msg){
	if(m_follower_initiation){
		Eigen::Vector2d fCam, pp;
		fCam[0] = leader_msg->fx; fCam[1] = leader_msg->fy;
        pp[0] = leader_msg->px; pp[1] = leader_msg->py;

        vector<Eigen::Vector2d> leader_left_dot     = fromROSPoseArrayToVector2d(leader_msg->leftDot);
        vector<Eigen::Vector2d> leader_right_dot    = fromROSPoseArrayToVector2d(leader_msg->rightDot);
        vector<Eigen::Vector2d> follower_left_dot   = fromROSPoseArrayToVector2d(m_follower_last_msg.leftDot);
        vector<Eigen::Vector2d> follower_right_dot  = fromROSPoseArrayToVector2d(m_follower_last_msg.rightDot);

        double weight;
        double best = -1;
        Eigen::Vector3d position, bestPosition;
        Eigen::Matrix3d rotation, bestRotation;
        PoseFilter poseEvaluator( m_rdA, m_ldA, m_rdB, m_ldB, pp, fCam);
        for(int i = 0; i < leader_left_dot.size(); i++){
            for(int j = 0; j < follower_left_dot.size(); j++){
                weight = poseEvaluator.comparePoseABtoBA(leader_left_dot[i], leader_right_dot[i],
                                                         follower_left_dot[j], follower_right_dot[j],
                                                         position, rotation);
                cout << i << " on " << j << " Weight: " << weight << endl << "Pose: "<< position.transpose() << endl;
                if(best < 0 || (abs(weight) < best && abs(weight) > 0.00001)){
                    best = abs(weight);
                    bestPosition = position;
                    bestRotation = rotation;
                }
            }
        }
        if(best >= 0){
            printf("=> Best: %6.4f \nPose: ", best);
            cout << bestPosition.transpose() << endl << endl;
            m_pose_pub.publish(MutualPoseEstimation::generatePoseMessage(bestPosition, bestRotation));
        }
    }
}

vector<Eigen::Vector2d> ParticleFilter::fromROSPoseArrayToVector2d(vector<geometry_msgs::Pose2D> ros_msg){
    vector<Eigen::Vector2d> eigenVectorArray;
    for(int i = 0; i < ros_msg.size(); i++){
        eigenVectorArray.push_back(Eigen::Vector2d(ros_msg[i].x, ros_msg[i].y));
    }
    return eigenVectorArray;
}

/**
 * The dynamic reconfigure callback function. This function updates the variable within the program whenever they are changed using dynamic reconfigure.
 */
void ParticleFilter::dynamicParametersCallback(particle_filter::ParticleFilterConfig &config, uint32_t level){
  
  m_ldA = config.pos_left_led_cam_a; m_rdA = config.pos_right_led_cam_a;
  m_ldB = config.pos_left_led_cam_b; m_rdB = config.pos_right_led_cam_b;

  ROS_INFO("Parameters changed");
  
}

} // namespace particle_filter


int main(int argc, char* argv[]){

    
	ROS_INFO("Main start...\n");
	ros::init(argc, argv,  "particle_filter_node");

	ros::NodeHandle n;
	particle_filter::ParticleFilter particle_filter(n);
  	ros::spin();
}
