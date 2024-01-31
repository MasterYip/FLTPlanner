
/* related header files */
#include "ikfast_interface_RF.hpp"
/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */
using namespace IKFAST_NAMESPACE_RF_SPACE;


/**
 * @brief
 *
 * @param[in] trans
 * @param[out] solution
 * @return true
 * @return false
 */
bool IKFast_trans3D_RF(const double trans[3], double solret[10][3])
{
    IkReal eerot[9];
    IkReal eetrans[3];
    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());
    for (int i = 0; i < 3; ++i)
    {
        eetrans[i] = trans[i];
    }
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
    if (!bSuccess)
    {
        return -1;
    }
    else
    {
        std::vector<IkReal> solvalues(GetNumJoints());
        for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i)
        {
            const IkSolutionBase<IkReal> &sol = solutions.GetSolution(i);
            std::vector<IkReal> vsolfree(sol.GetFree().size());
            sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);
            for (std::size_t j = 0; j < solvalues.size(); ++j)
                solret[i][j] = solvalues[j];
        }
        return 1;
    }  
}


// 注意这里添加的角度限制
static std::vector<float> maxAngleLimit = {1.785f, 3.14f, 3.925f};
static std::vector<float> minAngleLimit = {-1.785f, -0.5233f, -0.6978f};
std::vector<std::vector<double>> IKFast_trans3D_RF(const std::vector<double> trans)
{
    IkReal eerot[9];  // 这个不需要考虑,因为设置的是末端只考虑位置的逆运动学求解
    IkReal eetrans[3];
    std::vector<std::vector<double>> solret;
    IkSolutionList<IkReal> solutions;
    std::vector<IkReal> vfree(GetNumFreeParameters());
    if (trans.size() != 3)
    {
        throw std::invalid_argument("trans must be a vector of size 3");
    }

    for (int i = 0; i < 3; ++i)
    {
        eetrans[i] = trans[i];
    }
    bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
    if (!bSuccess)
    {
        return solret;
    }
    else
    {
        std::vector<IkReal> solvalues(GetNumJoints());
        for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i)
        {
            const IkSolutionBase<IkReal> &sol = solutions.GetSolution(i);
            std::vector<IkReal> vsolfree(sol.GetFree().size());
            sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);
            std::vector<double> solution;
            for (std::size_t j = 0; j < solvalues.size(); ++j)
                solution.push_back(solvalues[j]);
            // ***********注意这里添加的角度限制****************** 
            bool valid = true;
            for (int j = 0; j < 3; ++j) {
                if (!(minAngleLimit[j] < solution[j] && solution[j] < maxAngleLimit[j])) {
                    valid = false;
                    break;
                }
            }
            if (!valid) {
                continue;
            }
            // **************************************************
            solret.push_back(solution);
        }
        return solret;
    }
}