#include "particle_filter/pose_filter.h"
using namespace std;

namespace particle_filter
{

PoseFilter::PoseFilter(const double distanceRightLedRobotA, const double distanceLeftLedRobotA,
                       const double distanceRightLedRobotB, const double distanceLeftLedRobotB,
                       const Eigen::Vector2d focalCam, const Eigen::Vector2d centerCam):
                       m_rdA(distanceRightLedRobotA), m_ldA(distanceLeftLedRobotA),
                       m_rdB(distanceRightLedRobotB), m_ldB(distanceLeftLedRobotB),
                       m_focalCam(focalCam), m_centerCam(centerCam)
{
}

double PoseFilter::comparePoseABtoBA(const Eigen::Vector2d &pixelA1, const Eigen::Vector2d &pixelA2,
                                     const Eigen::Vector2d &pixelB1, const Eigen::Vector2d &pixelB2){

    Eigen::Matrix3d rotation;
    double distanceError;
    Eigen::Vector3d positionAB, positionBA;
    MutualPoseEstimation::compute3DMutualLocalisation(pixelA1, pixelA2,
                                                pixelB1, pixelB2,
                                                m_centerCam, m_centerCam, m_focalCam, m_focalCam,
                                                m_rdA, m_ldA, m_rdB, m_ldB, positionAB, rotation);
    // Convert Pose to a 4x4 transformation matrix
    //transformationAOnB = MutualPoseEstimation::fromPoseToTransformMatrix(positionAB, rotation);
    //cout << "transformationAOnB:\n" <<  transformationAOnB <<endl;


    MutualPoseEstimation::compute3DMutualLocalisation(pixelB1, pixelB2,
                                                pixelA1, pixelA2,
                                                m_centerCam, m_centerCam, m_focalCam, m_focalCam,
                                                m_rdA, m_ldA, m_rdB, m_ldB, positionBA, rotation);
   distanceError = positionAB.norm() - positionBA.norm();
    return distanceError;
}

} // namespace particle_filter
