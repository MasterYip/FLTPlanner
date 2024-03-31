#include <myDataType.h>


namespace MDT
{
    MDT::POINT pointRotationAndTrans(const MDT::POINT &pnt_currentF, const Eigen::Isometry3d &T_targetF_currentF)
    {
        Eigen::Vector3d point_targetFrame = T_targetF_currentF * pnt_currentF;
        return point_targetFrame;
    }

    MDT::POINT pointRotation(const MDT::POINT &pnt_currentF, const Eigen::Matrix3d &R_targetF_currentF)
    {
        Eigen::Vector3d point_targetFrame = R_targetF_currentF * pnt_currentF;
        return point_targetFrame;
    }


}