#include "particle_filter/mutual_pose_estimation.h"



using namespace std;

namespace particle_filter
{

Eigen::Matrix4d MutualPoseEstimation::fromPoseToTransformMatrix(Eigen::Vector3d position,  Eigen::Matrix3d rotation){
    Eigen::Matrix4d mat;
    mat(0, 0) = rotation(0, 0); mat(0, 1) = rotation(0, 1); mat(0, 2) = rotation(0, 2);
    mat(1, 0) = rotation(1, 0); mat(1, 1) = rotation(1, 1); mat(1, 2) = rotation(1, 2);
    mat(2, 0) = rotation(2, 0); mat(2, 1) = rotation(2, 1); mat(2, 2) = rotation(2, 2);

    mat(0, 3) = position[0];
    mat(1, 3) = position[1];
    mat(2, 3) = position[2];

    mat(3, 0) = 0;
    mat(3, 1) = 0;
    mat(3, 2) = 0;
    mat(3, 3) = 1;
    return mat;
}

geometry_msgs::PoseStamped MutualPoseEstimation::computePoseAndMessage(Eigen::Vector2d pixelA1, Eigen::Vector2d pixelA2, Eigen::Vector2d pixelB1, Eigen::Vector2d pixelB2,
                                                          double rdA, double ldA, double rdB, double ldB,
														  Eigen::Vector2d fCam, Eigen::Vector2d pp){
    Eigen::Vector3d position;
    Eigen::Matrix3d rotation;

    //MutualPoseEstimation::compute3DMutualLocalisation(pixelA1, pixelA2, pixelB1, pixelB2, pp, pp, fCam, fCam,
    //                                                       rdA, ldA, rdB, ldB, &position, &rotation);
    cout<<"Position: "<<position<<endl;
    cout<<"Distance: "<<position.norm()<<endl;
    cout<<"Rotation: "<<rotation<<endl;

    return MutualPoseEstimation::generatePoseMessage(position, rotation);
}

visualization_msgs::Marker MutualPoseEstimation::generateMarkerMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation, const double alpha){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "ardrone_base_link";
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



    marker.pose.position.x = position[2] + 0.21;
    marker.pose.position.y = -position[0];
    marker.pose.position.z = -position[1];
    double pitch, yaw, roll;
    /*
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation);

    double pitch = atan2(2.0*(q.y()*q.z() + q.w()*q.x()), q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());
    double yaw = asin(-2.0*(q.x()*q.z() - q.w()*q.y()));
    double roll = atan2(2.0*(q.x()*q.y() + q.w()*q.z()),  q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());

    printf("[OLD] Yaw %6.2f Pitch %6.2f Roll %6.2f\n", yaw, pitch, roll);
   // tf::Quaternion qTF = tf::Quaternion(M_PI*0.5-yaw, pitch- M_PI*0.5, -roll);
    tf::Quaternion qTF = tf::Quaternion(M_PI*1.5 - yaw, pitch- M_PI*0.5, M_PI - roll);
    qTF.setEulerZYX(yaw, pitch- M_PI*0.5, M_PI/2 - roll);*/

    // The rotation before the matrix inverse the rotation sign
    // The rotations after the matrix turn the 3d marker so it's facing the camera
    //marche pas
    /*rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
               * rotation
               * Eigen::AngleAxisd(-M_PI*0.5, Eigen::Vector3d::UnitZ())
               * Eigen::AngleAxisd(M_PI*0.5, Eigen::Vector3d::UnitY())*/;
    /*rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())
               * rotation
               * Eigen::AngleAxisd(-M_PI*0.5, Eigen::Vector3d::UnitZ())
               * Eigen::AngleAxisd(M_PI*0.5, Eigen::Vector3d::UnitY())*/;

    /*rotation = rotation
                   * Eigen::AngleAxisd(-M_PI*0.5, Eigen::Vector3d::UnitY())
                   * Eigen::AngleAxisd(-M_PI*0.5, Eigen::Vector3d::UnitX())
                   * Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitZ());*/
    //pitch
    /*rotation = Eigen::AngleAxisd(M_PI * 0.5, Eigen::Vector3d::UnitZ())
            * rotation
            * Eigen::AngleAxisd(-M_PI * 0.5, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());*/
    /*rotation = Eigen::AngleAxisd(-M_PI*0.5, Eigen::Vector3d::UnitZ())
               * Eigen::AngleAxisd(M_PI*0.5, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())
               * rotation
               * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).matrix().transpose()
               * Eigen::AngleAxisd(M_PI*0.5, Eigen::Vector3d::UnitX()).matrix().transpose()
               * Eigen::AngleAxisd(-M_PI*0.5, Eigen::Vector3d::UnitZ()).matrix().transpose();*/
    /*rotation = Eigen::AngleAxisd(-M_PI*0.5, Eigen::Vector3d::UnitZ())
               * Eigen::AngleAxisd(M_PI*0.5, Eigen::Vector3d::UnitX())
               * rotation
               * Eigen::AngleAxisd(M_PI*0.5, Eigen::Vector3d::UnitX()).matrix().transpose()
               * Eigen::AngleAxisd(M_PI*0.5, Eigen::Vector3d::UnitY()).matrix().transpose()
               * Eigen::AngleAxisd(-M_PI*0.5, Eigen::Vector3d::UnitZ()).matrix().transpose();*/
    rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())
               * rotation
               * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).matrix().transpose();
    // Matrix=>EulerXYZ (http://www.soi.city.ac.uk/~sbbh653/publications/euler.pdf)
    if(abs(rotation(2,0)) != 1){
        yaw   = -asin(rotation(2,1));
        pitch = atan2(rotation(2,1)/cos(yaw),rotation(2,2)/cos(yaw));
        roll  = atan2(rotation(1,0)/cos(yaw),rotation(0,0)/cos(yaw));
    }
    else{
        roll = 0;
        if(rotation(2,0) == -1){
            yaw = M_PI/2;
            pitch = roll + atan2(rotation(0,1), rotation(0,2));
        }
        else{
            yaw = -M_PI/2;
            pitch = -roll + atan2(-rotation(0,1), -rotation(0,2));
        }
    }
    printf("Matrix=>EulerXYZ Yaw:%1.2f Pitch:%1.2f Roll:%1.2f\n", yaw, pitch, roll);
/*
    tf::Quaternion qTF = tf::Quaternion(yaw, pitch, roll);
    qTF.setEuler(yaw, pitch, roll);
    marker.pose.orientation.x = qTF.x();
    marker.pose.orientation.y = qTF.y();
    marker.pose.orientation.z = qTF.z();
    marker.pose.orientation.w = qTF.w();*/
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation);
    marker.pose.orientation.x = q.z();// z
    marker.pose.orientation.y = -q.x();// x
    marker.pose.orientation.z = -q.y();//-y
    marker.pose.orientation.w = q.w();// w

    return marker;
}

void publishTransformation(){
    /*
    tf::StampedTransform tf_base_front;

    tf_base_front = tf::StampedTransform(
                    tf::Transform(
                        tf::createQuaternionFromRPY(0, 0.0, 0),
                        tf::Vector3(0, 0.0, 0.0)),
                ros::Time::now(), "ardrone_base_frontcam", "ardrone_base_cam_fix"
                    );
    tf::TransformBroadcaster tf_broad;
    tf_broad.sendTransform(tf_base_front);*/
}

geometry_msgs::PoseStamped MutualPoseEstimation::generatePoseMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation){

    geometry_msgs::PoseStamped estimated_position;
    estimated_position.pose.position.x = position[0]; // z
    estimated_position.pose.position.y = position[1];  // x
    estimated_position.pose.position.z = position[2]; //-y

    //Eigen::Matrix3d invRot = rotation.inverse();
    //rotation.col(0) = -1*rotation.col(0);
    cout<<"Rotation: "<<rotation<<endl;
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation);

    double pitch = atan2(2.0*(q.y()*q.z() + q.w()*q.x()), q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());
    double yaw = asin(-2.0*(q.x()*q.z() - q.w()*q.y()));
    double roll = atan2(2.0*(q.x()*q.y() + q.w()*q.z()),  q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    printf("[OLD] Yaw %6.2f Pitch %6.2f Roll %6.2f\n", yaw, pitch, roll);

   Eigen::Matrix3d m;// y => z => x
    // X Y Z = même output? (y et R inversé?)
    m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ())//Z
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY())//Y
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitZ());//Z
    //q = Eigen::Quaterniond(m);

   // rotation.col(0) = -1*rotation.col(0);
    //q = Eigen::Quaterniond(rotation);

    pitch = atan2(2.0*(q.y()*q.z() + q.w()*q.x()), q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());
    yaw = asin(-2.0*(q.x()*q.z() - q.w()*q.y()));
    roll = atan2(2.0*(q.x()*q.y() + q.w()*q.z()),  q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
    printf("[NEW] Yaw %6.2f Pitch %6.2f Roll %6.2f\n", yaw, pitch, roll);


    estimated_position.pose.orientation.x = q.x();// z
    estimated_position.pose.orientation.y = q.y();// x
    estimated_position.pose.orientation.z = q.z();//-y
    estimated_position.pose.orientation.w = q.w();// w
    estimated_position.header.frame_id = "ardrone_base_frontcam";
    //estimated_position.header.frame_id = "ardrone_base_link";
    return estimated_position;
}

/**
   * Compute the 3D pose in 6DOF using to camera for mutual localization
   *
   * \param pixelA1 Position of the left LED on robot A
   * \param pixelA2 Position of the right LED on robot A
   * \param pixelB1 Position of the left LED on robot B
   * \param pixelB2 Position of the right LED on robot
   * \param ppA Center of the camera of robot A
   * \param ppB Center of the camera of robot B
   * \param fCamA Focal length of the camera of robot A
   * \param fCamB Focal length of the camera of robot B
   * \param rdA Distance between the camera and the LED on the right side of robot A (Positive)
   * \param ldA Distance between the camera and the LED on the left side of robot A (Positive)
   * \param rdB Distance between the camera and the LED on the right side of robot B (Positive)
   * \param ldB Distance between the camera and the LED on the left side of robot B (Positive)
   * \param position (Output) the position vector
   * \param rotation (Output) the rotation matrix
   *
   * \return the rotation matrix of the relative pose
   *
   */
void MutualPoseEstimation::compute3DMutualLocalisation(const Eigen::Vector2d &pixelA1, const Eigen::Vector2d &pixelA2,
                                                 const Eigen::Vector2d &pixelB1,const  Eigen::Vector2d &pixelB2,
                                                 const Eigen::Vector2d &fCamA, const Eigen::Vector2d &fCamB,
                                                 const Eigen::Vector2d &ppA, const Eigen::Vector2d &ppB,
                                                 const double &rdA, const double &ldA, const double &rdB, const double &ldB,
                                                 Eigen::Vector3d & position, Eigen::Matrix3d & rotation){
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

  double d = rdA + ldA;

  //Eigen::Vector2d BLeftMarker(pixelA1[0], pixelA2[0]);
  //Eigen::Vector2d BRightMarker(pixelA1[1], pixelA2[1
  Eigen::Vector2d BLeftMarker = pixelA2;
  Eigen::Vector2d BRightMarker = pixelA1;
  
  Eigen::Vector2d PB1(BLeftMarker[0] + (ldB/(rdB+ldB)) * (BRightMarker[0] - BLeftMarker[0]),
                      BLeftMarker[1] + (ldB/(rdB+ldB)) * (BRightMarker[1] - BLeftMarker[1]));

  //cout<<"PB1: "<<PB1<<endl;
  //cout << "BLeftMarker: \n" << BLeftMarker << endl;
  Eigen::Vector3d PB12((PB1[0]-ppA[0])/fCamA[0], (PB1[1]-ppA[1])/fCamA[1], 1);
  PB12.normalize();
  double phi = acos(PB12[0]);
  double beta = 0.5f * M_PI - phi;
  //printf("Beta: %f\n",beta);

  Eigen::Vector2d plane = MutualPoseEstimation::computePositionMutual(alpha, beta, d);

  //cout<<"plane: "<<plane<<endl;
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
                      ((ALeftMarker[0] + (ldA/(rdA+ldA))*(ARightMarker[0] - ALeftMarker[0]))-ppB[0])/fCamB[0],
                      ((ALeftMarker[1] + (ldA/(rdA+ldA))*(ARightMarker[1] - ALeftMarker[1]))-ppB[1])/fCamB[1],
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
  //cout << "Rotation:\n" << Rotation << endl;
}

Eigen::Vector2d MutualPoseEstimation::computePositionMutual(double alpha, double beta, double d){
  double r = 0.5*d/sin(alpha);
  
  // Position of the center
  double Cx = 0.0;
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
