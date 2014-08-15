#include "particle_filter/mutual_pose_estimation.h"



using namespace std;

namespace particle_filter
{

void MutualPoseEstimation::setMarkersParameters(const double distanceRightLedRobotA, const double distanceLeftLedRobotA,
                                 const double distanceRightLedRobotB, const double distanceLeftLedRobotB){
    this->rdA = distanceRightLedRobotA; this->ldA = distanceLeftLedRobotA;
    this->rdB = distanceRightLedRobotB; this->ldB = distanceLeftLedRobotB;
}

void MutualPoseEstimation::setCameraParameters(const Eigen::Vector2d pFocalCam, const Eigen::Vector2d pCenterCam){
    this->focalCam = pFocalCam;
    this->centerCam = pCenterCam;
}


visualization_msgs::Marker MutualPoseEstimation::generateMarkerMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation, const double alpha){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "ardrone_base_frontcam";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = alpha;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    //marker.mesh_resource = "package://particle_filter/mesh/quadrotor_3.stl";
    marker.mesh_resource = "package://particle_filter/mesh/quadrotor_2_move_cam.stl";


    // The camera is 21cm in front of the center of the drone
//    marker.pose.position.x = position[2] + 0.21;
//    marker.pose.position.y = -position[0];
//    marker.pose.position.z = -position[1];
    marker.pose.position.x = position[0] + 0.21;
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];

    rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
               * rotation;

    Eigen::Quaterniond q = Eigen::Quaterniond(rotation);
//    marker.pose.orientation.x = q.z();
//    marker.pose.orientation.y = -q.x();
//    marker.pose.orientation.z = -q.y();
//    marker.pose.orientation.w = q.w();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    return marker;
}

geometry_msgs::PoseStamped MutualPoseEstimation::generatePoseMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation){

    geometry_msgs::PoseStamped estimated_position;
    estimated_position.header.frame_id = "ardrone_base_frontcam";//ardrone_base_link

    //estimated_position.pose.position.x = position[2] + 0.21; // z
    //estimated_position.pose.position.y = -position[0];  // x
    //estimated_position.pose.position.z = -position[1]; //-y
    estimated_position.pose.position.x = position[0] + 0.21;
    estimated_position.pose.position.y = position[1];
    estimated_position.pose.position.z = position[2];

    rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
               * rotation;

    Eigen::Quaterniond q = Eigen::Quaterniond(rotation);
//    estimated_position.pose.orientation.x = q.z();
//    estimated_position.pose.orientation.y = -q.x();
//    estimated_position.pose.orientation.z = -q.y();
//    estimated_position.pose.orientation.w = q.w();
    estimated_position.pose.orientation.x = q.x();
    estimated_position.pose.orientation.y = q.y();
    estimated_position.pose.orientation.z = q.z();
    estimated_position.pose.orientation.w = q.w();
    return estimated_position;
}


double MutualPoseEstimation::comparePoseABtoBA(const Eigen::Vector2d &pixelA1, const Eigen::Vector2d &pixelA2,
                                               const Eigen::Vector2d &pixelB1, const Eigen::Vector2d &pixelB2,
                                               Eigen::Vector3d &positionAB,  Eigen::Matrix3d &rotationAB){

    Eigen::Matrix3d rotationBA;
    Eigen::Vector3d positionBA;
    Eigen::Vector2d lPixelA1 = -(pixelA1 - this->centerCam);
    Eigen::Vector2d lPixelA2 = -(pixelA2 - this->centerCam);
    Eigen::Vector2d lPixelB1 = -(pixelB1 - this->centerCam);
    Eigen::Vector2d lPixelB2 = -(pixelB2 - this->centerCam);
    Eigen::Vector2d notDOTHAT = this->centerCam;

    this->compute3DMutualLocalisation(lPixelA1, lPixelA2,
                                      lPixelB1, lPixelB2,
                                      positionAB, rotationAB);


    this->compute3DMutualLocalisation(lPixelB1, lPixelB2,
                                      lPixelA1, lPixelA2,
                                      positionBA, rotationBA);
    this->centerCam = notDOTHAT;
    double distanceError = positionAB.norm() - positionBA.norm();
    return distanceError;
}

/**
   * Compute the 3D pose in 6DOF using to camera for mutual localization
   *
   * \param pixelA1 Position of the left LED on robot A
   * \param pixelA2 Position of the right LED on robot A
   * \param pixelB1 Position of the left LED on robot B
   * \param pixelB2 Position of the right LED on robot
   * \param position (Output) the position vector
   * \param rotation (Output) the rotation matrix
   *
   * \return the rotation matrix of the relative pose
   *
   */
void MutualPoseEstimation::compute3DMutualLocalisation(const Eigen::Vector2d &pixelA1, const Eigen::Vector2d &pixelA2,
                                                       const Eigen::Vector2d &pixelB1,const  Eigen::Vector2d &pixelB2,
                                                       Eigen::Vector3d &position, Eigen::Matrix3d &rotation){

  Eigen::Vector2d fCamA = this->focalCam;
  Eigen::Vector2d fCamB = this->focalCam;
  Eigen::Vector2d ppA = this->centerCam;
  Eigen::Vector2d ppB = this->centerCam;
  /*
  cout<<"-Parameters-"<<endl;
  cout<<"pixelA1:"<<pixelA1<<endl;
  cout<<"pixelA2:"<<pixelA2<<endl;
  cout<<"pixelB1:"<<pixelB1<<endl;
  cout<<"pixelB2:"<<pixelB2<<endl;
  cout<<"ppA:"<<ppA<<endl;
  cout<<"ppB:"<<ppB<<endl;
  cout<<"fCamA:"<<fCamA<<endl;
  cout<<"fCamB:"<<fCamB<<endl;
  cout<<"rdA:"<<rdA<<endl;
  cout<<"ldA:"<<ldA<<endl;
  cout<<"rdB:"<<rdB<<endl;
  cout<<"ldB:"<<ldB<<endl;*/

  Eigen::Vector3d PAM1((pixelB1[0]-ppB[0])/fCamB[0], (pixelB1[1]-ppB[1])/fCamB[1], 1);
  Eigen::Vector3d PAM2((pixelB2[0]-ppB[0])/fCamB[0], (pixelB2[1]-ppB[1])/fCamB[1], 1);
  PAM1.normalize();
  PAM2.normalize();
  double alpha = acos(PAM1.dot(PAM2));
  //printf("Alpha: %f\n",alpha);

  double d = this->rdA + this->ldA;

  Eigen::Vector2d BLeftMarker = pixelA2;
  Eigen::Vector2d BRightMarker = pixelA1;
  
  Eigen::Vector2d PB1(BLeftMarker[0] + (this->ldB/(rdB+ldB)) * (BRightMarker[0] - BLeftMarker[0]),
                      BLeftMarker[1] + (this->ldB/(rdB+ldB)) * (BRightMarker[1] - BLeftMarker[1]));

  Eigen::Vector3d PB12((PB1[0]-ppA[0])/fCamA[0], (PB1[1]-ppA[1])/fCamA[1], 1);
  PB12.normalize();
  double phi = acos(PB12[0]);
  double beta = 0.5f * M_PI - phi;
  //printf("Beta: %f\n",beta);

  Eigen::Vector2d plane = MutualPoseEstimation::computePositionMutual(alpha, beta, d);

  double EstimatedDistance = plane.norm();

  position =  PB12 * EstimatedDistance;
    //====================================================================
    //=========================Axis Angle Part============================
    //Create the two plans
    //Plan in B Refs
  Eigen::Vector2d ALeftMarker = pixelB2;
  Eigen::Vector2d ARightMarker = pixelB1;

  Eigen::Vector3d ALM((ALeftMarker[0]-ppB[0])/fCamB[0], (ALeftMarker[1]-ppB[1])/fCamB[1], 1);
  ALM.normalize();

  Eigen::Vector3d ARM((ARightMarker[0]-ppB[0])/fCamB[0], (ARightMarker[1]-ppB[1])/fCamB[1], 1);
  ARM.normalize();
  //Plan in A Refs
  Eigen::Vector3d AToB = PB12;

  Eigen::Vector3d LeftMarker(1, 0, 0);

  //Align the two plans
  Eigen::Vector3d NormalPlanInB = ALM.cross(ARM);
  Eigen::Vector3d NormalPlanInA = AToB.cross(LeftMarker);
  Eigen::Vector3d AxisAlignPlans = NormalPlanInB.cross(NormalPlanInA);
  NormalPlanInB.normalize();
  NormalPlanInA.normalize();
  AxisAlignPlans.normalize();
  double AngleAlignPlans = acos(NormalPlanInB.dot(NormalPlanInA));

  Eigen::MatrixXd AlignPlans = vrrotvec2mat(AngleAlignPlans, AxisAlignPlans);

  //Align the vector of the cameraA seen from B with the plan
  Eigen::Vector3d CameraASeenFromB(
                      ((ALeftMarker[0] + (this->ldA/(this->rdA+this->ldA))*(ARightMarker[0] - ALeftMarker[0]))-ppB[0])/fCamB[0],
                      ((ALeftMarker[1] + (this->ldA/(this->rdA+this->ldA))*(ARightMarker[1] - ALeftMarker[1]))-ppB[1])/fCamB[1],
                      1);
  CameraASeenFromB.normalize();
  Eigen::Vector3d alignedBToA = AlignPlans * CameraASeenFromB;
  //Turn the vector BToA to make it align with AToB
  Eigen::Vector3d AxisAlignABwBA = alignedBToA.cross(AToB);
  AxisAlignABwBA.normalize();
  //Since we know that cameras are facing each other, we rotate pi
  double AngleAlignABwBA = acos(alignedBToA.dot(AToB)) - M_PI;
  
  Eigen::Matrix3d AlignVectors = vrrotvec2mat(AngleAlignABwBA, AxisAlignABwBA);

  rotation =  AlignVectors * AlignPlans;

  //Correction of the depth error:
  if(false){
      Eigen::Vector3d s_a(0, 0, 0.03); // 3 cm back from the plane
      double s_a_norm = s_a.norm();
      double distance = position.norm();

      double angle_error = acos( position.dot(s_a) / (distance*s_a_norm) );
      ROS_INFO("Errr: %f",angle_error);

      position = s_a_norm*cos(angle_error)*position/distance;
  }
}

Eigen::Vector2d MutualPoseEstimation::computePositionMutual(double alpha, double beta, double d){
  double r = 0.5*d/sin(alpha);
  
  // Position of the center
  double Cy = 0.5*d/tan(alpha);

  // From http://mathworld.wolfram.com/Circle-LineIntersection.html
  // Intersection of a line and of a circle, with new beta
  double OtherAngle = 0.5 * M_PI - beta;
  double x1 = 0; 
  double x2 = 50 * cos(OtherAngle);
  double y1 = -Cy; 
  double y2 = 50 * sin(OtherAngle) - Cy;

  double dx = x2-x1;
  double dy = y2-y1;

  double dr = sqrt(dx*dx + dy*dy);
  double D  = x1*y2 - x2*y1;
    
  double X = (D*dy+abs(dy)/dy*dx*sqrt(r*r*dr*dr-D*D))/dr/dr;
  double Y = (-D*dx + abs(dy)*sqrt(r*r*dr*dr-D*D))/dr/dr;
    
  Y = Y + Cy;

  Eigen::Vector2d p(X, Y);
  return  p;
}

Eigen::Matrix3d MutualPoseEstimation::vrrotvec2mat(double p, Eigen::Vector3d r){
  double s = sin(p);
  double c = cos(p);
  double t = 1 - c;
  r.normalize();
  Eigen::Vector3d n = r;

  double x = n[0];
  double y = n[1];
  double z = n[2];
  Eigen::Matrix3d m(3,3);
  m(0,0) = t*x*x + c; m(0,1) = t*x*y - s*z; m(0,2) = t*x*z + s*y;
  m(1,0) = t*x*y + s*z; m(1,1) = t*y*y + c; m(1,2) = t*y*z - s*x;
  m(2,0) = t*x*z - s*y; m(2,1) = t*y*z + s*x; m(2,2) = t*z*z + c;
  return m;
}





} // namespace particle_filter
