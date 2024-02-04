#include "single_leg_kin.h"

SingleLegKin buildElSpiderAir()
{
    SingleLegKin kin("/home/yip/CodeSpace/GithubRepo/LeggedKinModule/config/leg.urdf");
    // Eigen::Vector3d origin_calib = {0.0, 0.0, 0.0};
    Eigen::Vector3d origin_calib = {0.3, -0.08, 0.011};
    Eigen::Vector3d mirror_offset = {1.0, 1.0, 1.0};
    Eigen::Matrix3d rot_offset = Eigen::Matrix3d::Identity();
    Eigen::Vector3d pos_offset = {0.0, 0.0, 0.0};
    Eigen::VectorXd joint_limits;
    joint_limits.resize(6);
    joint_limits << -0.785, 0.785, -0.5233, 3.14, -0.6978, 3.925;
    Eigen::Vector3d ik_approx_point = {0.0535018, -0.279989, -0.276952}; // joint (0, 1.57, 1.57)
    kin.setEndEffectorName("RF_FOOT");
    kin.setOriginCalib(origin_calib);
    kin.setMirrorOffset(mirror_offset);
    kin.setPosOffset(pos_offset);
    kin.setRotOffset(rot_offset);
    kin.setJointLimits(joint_limits);
    kin.setIKApproxPoint(ik_approx_point);
    return kin;
}

int main()
{
    printf("IK: ");
    SingleLegKin kin = buildElSpiderAir();
    Eigen::Vector3d pos = {0.05350208, -0.14998902, -0.1480558};
    std::vector<Eigen::Vector3d> sols;
    kin.inverseKinConstraint(pos, sols);
    std::cout << sols.size() << std::endl;
    for (size_t i = 0; i < sols.size(); i++)
    {
        std::cout << sols[i].transpose() << std::endl;
    }

    printf("FK: ");
    Eigen::Vector3d joints = {0.0, 1.57, 1.57};
    Eigen::Vector3d pos_fk;
    kin.forwardKin(joints, pos_fk);
    std::cout << pos_fk.transpose() << std::endl;
}
