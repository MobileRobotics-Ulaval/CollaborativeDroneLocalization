#include "particle_filter/particle_filter_node.h"

using namespace std;

namespace particle_filter
{
ParticleFilter::ParticleFilter(ros::NodeHandle n) : 
    nodeHandler(n),
    follower_dots_initiation(false),
    leader_imu_initiation(false),
    follower_imu_initiation(false)
  {
  string topic_leader, topic_follower;
  ros::param::get("~leader", topic_leader);
  ros::param::get("~follower", topic_follower);

  ROS_INFO("Subscribing to %s and %s", topic_leader.c_str(), topic_follower.c_str());

  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  dynamic_reconfigure::Server<particle_filter::ParticleFilterConfig>::CallbackType dynamicReconfigCallback;
  dynamicReconfigCallback = boost::bind(&ParticleFilter::dynamicParametersCallback, this, _1, _2);
  dynamicReconfigServer.setCallback(dynamicReconfigCallback);

  this->createPublishers(topic_follower);
  this->createSubscribers(topic_leader, topic_follower);
}

void ParticleFilter::createPublishers(const string& topic_follower){
    this->pubPose = this->nodeHandler.advertise<geometry_msgs::PoseStamped>(topic_follower + "/pose", 1);
    this->pubMarker = this->nodeHandler.advertise<visualization_msgs::Marker>(topic_follower + "/marker", 1);
    this->pubMarkerCandidates = this->nodeHandler.advertise<visualization_msgs::MarkerArray>(topic_follower + "/pose_candidates", 1);
}

void ParticleFilter::createSubscribers(const string& topic_leader, const string& topic_follower){
    this->subDotsLeader = this->nodeHandler.subscribe(topic_leader + "/dots", 1, &ParticleFilter::leaderDotsCallback, this);
    this->subDotsFollower = this->nodeHandler.subscribe(topic_follower + "/dots", 1, &ParticleFilter::followerDotsCallback, this);

    this->subIMULeader = this->nodeHandler.subscribe(topic_leader + "/ardrone/imu", 1, &ParticleFilter::leaderIMUCallback, this);
    this->subIMUFollower = this->nodeHandler.subscribe(topic_follower + "/ardrone/imu", 1, &ParticleFilter::followerIMUCallback, this);
}

void ParticleFilter::followerDotsCallback(const dot_finder::DuoDot::ConstPtr& follower_msg){
    if(!this->follower_dots_initiation)
        this->follower_dots_initiation = true;

    this->follower_last_msg = *follower_msg;

}

void ParticleFilter::leaderDotsCallback(const dot_finder::DuoDot::ConstPtr& leader_msg){
    this->leader_last_msg = *leader_msg;

    if(this->isInitiated()){
        this->runParticleFilter();
    }
}


bool ParticleFilter::isInitiated(){
    return this->follower_dots_initiation && this->leader_imu_initiation && this->follower_imu_initiation;
}

void ParticleFilter::runParticleFilter(){
    Eigen::Vector2d fCam, pp;
    fCam[0] =this->leader_last_msg.fx; fCam[1] = this->leader_last_msg.fy;
    pp[0] = this->leader_last_msg.px; pp[1] = this->leader_last_msg.py;
    this->poseEvaluator.setCameraParameters(fCam, pp);

    vector<Eigen::Vector2d> leaderLeftDot     = fromROSPoseArrayToVector2d(this->leader_last_msg.leftDot);
    vector<Eigen::Vector2d> leaderRightDot    = fromROSPoseArrayToVector2d(this->leader_last_msg.rightDot);
    vector<Eigen::Vector2d> followerLeftDot   = fromROSPoseArrayToVector2d(this->follower_last_msg.leftDot);
    vector<Eigen::Vector2d> followerRightDot  = fromROSPoseArrayToVector2d(this->follower_last_msg.rightDot);

    double weight;
    double best = -1;
    Eigen::Vector3d position, bestPosition;
    Eigen::Matrix3d rotation, bestRotation;
    visualization_msgs::MarkerArray candidatesMsgs;
    for(int i = 0; i < leaderLeftDot.size(); i++){
        for(int j = 0; j < followerLeftDot.size(); j++){
            weight = this->poseEvaluator.comparePoseABtoBA(leaderLeftDot[i], leaderRightDot[i],
                                                           followerLeftDot[j], followerRightDot[j],
                                                           position, rotation);
            //cout << i << " on " << j << " Weight: " << weight << endl << "Pose: "<< position.transpose() << endl;

            // Create for a rviz Marker for each combination of dots, with a tranparency factor of 0.3
            candidatesMsgs.markers.push_back(MutualPoseEstimation::generateMarkerMessage(position, rotation, 0.3));
            if(best < 0 || (abs(weight) < best && abs(weight) > 0.00001)){
                best = abs(weight);
                bestPosition = position;
                bestRotation = rotation;
            }
        }
    }
    if(best >= 0){
        printf("=> Best: %6.4f \nPose: ", best);
        cout << bestPosition.transpose() << endl<<"Distance: "<< bestPosition.norm() << endl << endl;


        this->pubPose.publish(MutualPoseEstimation::generatePoseMessage(bestPosition, bestRotation));
        this->pubMarker.publish(MutualPoseEstimation::generateMarkerMessage(bestPosition, bestRotation, 1.0));
        this->pubMarkerCandidates.publish(candidatesMsgs);
    }
}
vector<Eigen::Vector2d> ParticleFilter::fromROSPoseArrayToVector2d(vector<geometry_msgs::Pose2D> ros_msg){
    vector<Eigen::Vector2d> eigenVectorArray;
    for(int i = 0; i < ros_msg.size(); i++){
        eigenVectorArray.push_back(Eigen::Vector2d(ros_msg[i].x, ros_msg[i].y));
    }
    return eigenVectorArray;
}


void ParticleFilter::leaderIMUCallback(const sensor_msgs::Imu::ConstPtr& leader_imu_msg){
    if(!this->leader_imu_initiation)
        this->leader_imu_initiation = true;

    this->leader_imu_msg = *leader_imu_msg;

}

void ParticleFilter::followerIMUCallback(const sensor_msgs::Imu::ConstPtr& follower_imu_msg){
    if(!this->follower_imu_initiation)
        this->follower_imu_initiation = true;

    this->follower_imu_msg = *follower_imu_msg;

}

void ParticleFilter::followerIMUCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

/**
 * The dynamic reconfigure callback function. This function updates the variable within the program whenever they are changed using dynamic reconfigure.
 */
void ParticleFilter::dynamicParametersCallback(particle_filter::ParticleFilterConfig &config, uint32_t level){
  this->poseEvaluator.setMarkersParameters(config.pos_right_led_cam_a,
                                           config.pos_left_led_cam_a,
                                           config.pos_right_led_cam_b,
                                           config.pos_left_led_cam_b);
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
