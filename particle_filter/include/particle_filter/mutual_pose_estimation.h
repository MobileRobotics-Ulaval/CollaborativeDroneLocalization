#ifndef POSE_ESTIMATION_H
#define POSE_ESTIMATION_H

#include <iostream>
#include <fstream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace particle_filter
{
class MutualPoseEstimation
{
public:
    static Eigen::Matrix4d fromPoseToTransformMatrix(Eigen::Vector3d position,  Eigen::Matrix3d rotation);

    static geometry_msgs::PoseStamped computePoseAndMessage(Eigen::Vector2d ImageA1, Eigen::Vector2d ImageA2,
                                                     Eigen::Vector2d ImageB1, Eigen::Vector2d ImageB2,
                                                     double rdA, double ldA, double rdB, double ldB,
                                                     Eigen::Vector2d fCam, Eigen::Vector2d pp);
    static visualization_msgs::Marker generateMarkerMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation, const double alpha);
    static geometry_msgs::PoseStamped generatePoseMessage(const Eigen::Vector3d &position, Eigen::Matrix3d rotation);
    static void compute3DMutualLocalisation(const Eigen::Vector2d &pixelA1, const Eigen::Vector2d &pixelA2,
                                            const Eigen::Vector2d &pixelB1,const  Eigen::Vector2d &pixelB2,
                                            const Eigen::Vector2d &fCamA, const Eigen::Vector2d &fCamB,
                                            const Eigen::Vector2d &ppA, const Eigen::Vector2d &ppB,
                                            const double &rdA, const double &ldA, const double &rdB, const double &ldB,
                                            Eigen::Vector3d & position, Eigen::Matrix3d & rotation);
    static Eigen::Vector2d computePositionMutual(double alpha, double beta, double d);
    static Eigen::Matrix3d vrrotvec2mat(double p, Eigen::Vector3d r);
};
}
#endif // POSE_ESTIMATION_H
