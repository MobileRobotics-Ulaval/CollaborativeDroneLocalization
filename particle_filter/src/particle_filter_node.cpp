#include "particle_filter/particle_filter_node.h"

using namespace std;

namespace particle_filter
{
ParticleFilter::ParticleFilter(ros::NodeHandle n) : 
    nodeHandler(n),
    followerDotsInitiation(false),
    leaderImuInitiation(false),
    followerImuInitiation(false)
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
    this->pubPoseCandidates = this->nodeHandler.advertise<geometry_msgs::PoseArray>(topic_follower + "/pose_array_candidates", 1);
}

void ParticleFilter::createSubscribers(const string& topic_leader, const string& topic_follower){
    this->subDotsLeader = this->nodeHandler.subscribe(topic_leader + "/dots", 1, &ParticleFilter::leaderDotsCallback, this);
    this->subDotsFollower = this->nodeHandler.subscribe(topic_follower + "/dots", 1, &ParticleFilter::followerDotsCallback, this);

    this->subImuLeader = this->nodeHandler.subscribe(topic_leader + "/ardrone/imu", 1, &ParticleFilter::leaderImuCallback, this);
    this->subImuFollower = this->nodeHandler.subscribe(topic_follower + "/ardrone/imu", 1, &ParticleFilter::followerImuCallback, this);
}

void ParticleFilter::followerDotsCallback(const dot_finder::DuoDot::ConstPtr& follower_msg){
    if(!this->followerDotsInitiation)
        this->followerDotsInitiation = true;

    this->followerLastMsg = *follower_msg;

}

void ParticleFilter::leaderDotsCallback(const dot_finder::DuoDot::ConstPtr& leader_msg){
    this->leaderLastMsg = *leader_msg;

    if(this->isInitiated()){
        this->runParticleFilter();
       // this->generateCSVLog();
    }
}


bool ParticleFilter::isInitiated(){
    return this->followerDotsInitiation && this->leaderImuInitiation && this->followerImuInitiation;
}

void ParticleFilter::runParticleFilter(){
    Eigen::Vector2d fCam, pp;
    fCam[0] =this->leaderLastMsg.fx; fCam[1] = this->leaderLastMsg.fy;
    pp[0] = this->leaderLastMsg.px; pp[1] = this->leaderLastMsg.py;
    this->poseEvaluator.setCameraParameters(fCam, pp);

    vector<Eigen::Vector2d> leaderLeftDot     = fromROSPoseArrayToVector2d(this->leaderLastMsg.leftDot);
    vector<Eigen::Vector2d> leaderRightDot    = fromROSPoseArrayToVector2d(this->leaderLastMsg.rightDot);
    vector<Eigen::Vector2d> followerLeftDot   = fromROSPoseArrayToVector2d(this->followerLastMsg.leftDot);
    vector<Eigen::Vector2d> followerRightDot  = fromROSPoseArrayToVector2d(this->followerLastMsg.rightDot);

    double weight;
    double best = -1;
    Eigen::Vector3d position, bestPosition;
    Eigen::Matrix3d rotation, bestRotation;
    visualization_msgs::MarkerArray candidatesMarkerMsgs;
    //geometry_msgs::PoseArray candidatesPoseMsgs;
    this->candidatesPoseMsgs.poses.clear();
    for(int i = 0; i < leaderLeftDot.size(); i++){
        for(int j = 0; j < followerLeftDot.size(); j++){
            weight = this->poseEvaluator.comparePoseABtoBA(leaderLeftDot[i], leaderRightDot[i],
                                                           followerLeftDot[j], followerRightDot[j],
                                                           position, rotation);
            cout << i << " on " << j << " Weight: " << weight << endl << "Pose: "<< position.transpose() << endl;

            // Create for a rviz Marker for each combination of dots, with a tranparency factor of 0.3
            candidatesMarkerMsgs.markers.push_back(MutualPoseEstimation::generateMarkerMessage(position, rotation, 0.3));
            candidatesPoseMsgs.poses.push_back(MutualPoseEstimation::generatePoseMessage(position, rotation).pose);
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
        this->pubMarkerCandidates.publish(candidatesMarkerMsgs);
        candidatesPoseMsgs.header.frame_id = "ardrone_base_link";
        this->pubPoseCandidates.publish(candidatesPoseMsgs);
    }
}
vector<Eigen::Vector2d> ParticleFilter::fromROSPoseArrayToVector2d(vector<geometry_msgs::Pose2D> ros_msg){
    vector<Eigen::Vector2d> eigenVectorArray;
    for(int i = 0; i < ros_msg.size(); i++){
        eigenVectorArray.push_back(Eigen::Vector2d(ros_msg[i].x, ros_msg[i].y));
    }
    return eigenVectorArray;
}


void ParticleFilter::leaderImuCallback(const sensor_msgs::Imu::ConstPtr& leaderImuMsg){
    if(!this->leaderImuInitiation)
        this->leaderImuInitiation = true;

    this->leaderImuMsg = *leaderImuMsg;
}
void ParticleFilter::followerImuCallback(const sensor_msgs::Imu::ConstPtr& follower_imu_msg){
    if(!this->followerImuInitiation)
        this->followerImuInitiation = true;

    this->followerImuMsg = *follower_imu_msg;
}

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

void ParticleFilter::generateCSVLog(){
    string filename_imu, filename_dot, filename_poses;
    filename_imu  = "_imu.csv";
    filename_dot  = "_dot.csv";
    filename_poses  = "pose_candidates.csv";
    const char *type[] = {"leader", "follower"};
    //double offset = 1406320000;

    ofstream myfile;

    // imu.csv
    sensor_msgs::Imu imu_log;
    for(int t = 0; t < 2; t++){
        myfile.open((string(type[t])+ filename_imu).c_str(), ios::app);

        if(t == 0)
            imu_log = this->leaderImuMsg;
        else
            imu_log = this->followerImuMsg;
        myfile << imu_log.header.stamp.toNSec() << ","
               << imu_log.orientation.x << ","
               << imu_log.orientation.y << ","
               << imu_log.orientation.z << ","
               << imu_log.orientation.w << ","
               << imu_log.angular_velocity.x << ","
               << imu_log.angular_velocity.y << ","
               << imu_log.angular_velocity.z << ","
               << imu_log.linear_acceleration.x << ","
               << imu_log.linear_acceleration.y << ","
               << imu_log.linear_acceleration.z << "\n";
        myfile.close();
    }

    // dot.csv
    dot_finder::DuoDot dot_log;
    for(int t = 0; t < 2; t++){
        myfile.open((string(type[t])+ filename_dot).c_str(), ios::app);
        if(t == 0)
            dot_log = this->leaderLastMsg;
        else
            dot_log = this->followerLastMsg;

        for(int i = 0;  i < dot_log.leftDot.size(); i++){
            myfile << dot_log.header.stamp.toNSec() << ","
                   << i << ","
                   << dot_log.leftDot.at(i).x << ","
                   << dot_log.leftDot.at(i).y << ","
                   << dot_log.rightDot.at(i).x << ","
                   << dot_log.rightDot.at(i).y << "\n";
        }
        myfile.close();
    }

    // Pose candidate
    ros::Time timeNow = ros::Time::now();
    myfile.open(filename_poses.c_str(), ios::app);
    for(int i = 0;  i < this->candidatesPoseMsgs.poses.size(); i++){
        myfile  << timeNow.toNSec() << ","
               << (i - i % this->followerLastMsg.leftDot.size())/ this->followerLastMsg.leftDot.size() << ","
               << i % this->followerLastMsg.leftDot.size() << ","
               << this->candidatesPoseMsgs.poses[i].orientation.x << ","
               << this->candidatesPoseMsgs.poses[i].orientation.y << ","
               << this->candidatesPoseMsgs.poses[i].orientation.z << ","
               << this->candidatesPoseMsgs.poses[i].orientation.w << ","
               << this->candidatesPoseMsgs.poses[i].position.x -0.21 << ","
               << this->candidatesPoseMsgs.poses[i].position.y << ","
               << this->candidatesPoseMsgs.poses[i].position.z << "\n";
        // z -x -y => x=z , y = -x, z = -y
    }

    myfile.close();

    ROS_INFO("All files saved!!!");

}


} // namespace particle_filter


int main(int argc, char* argv[]){

    
	ROS_INFO("Main start...\n");
	ros::init(argc, argv,  "particle_filter_node");

	ros::NodeHandle n;
	particle_filter::ParticleFilter particle_filter(n);

    ros::spin();
}
