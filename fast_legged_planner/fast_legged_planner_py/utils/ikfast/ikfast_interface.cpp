
/* related header files */
#include "ikfast_interface.hpp"
/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */

/**
 * @brief
 *
 * @param[in] trans
 * @param[out] solution
 * @return true
 * @return false
 */
bool IKFast_trans3D(const double trans[3], double solret[10][3])
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

std::vector<std::vector<double>> IKFast_trans3D(const std::vector<double> trans)
{
    IkReal eerot[9];
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
            solret.push_back(solution);
        }
        return solret;
    }
}