
// #include "ikfast_interface_RB.hpp"
// #include "ikfast_interface_RF.hpp"
#include "ik_fast_comm.h"






int main()
{
    std::vector<double> trans = {-0.13028, -0.2545, -0.31143};
    std::vector<std::vector<double>> solret;
    

    solret.clear();

    //  CY VERSION
    double trans_[3], rot_[9];
    trans_[0] = trans[0];
    trans_[1] = trans[1];
    trans_[2] = trans[2];
    ikfast::IkSolutionList<IkReal> solutions;
    auto bSuccess = el_mini_ComputeIk_lb(trans_, rot_, NULL, solutions);
    if(!bSuccess)
    {
        std::cout << "fail" << std::endl;
    }
    else
    {
        std::vector<IkReal> solvalues(GetNumJoints());
        for (std::size_t i = 0; i < solutions.GetNumSolutions(); ++i)
        {
            const ikfast::IkSolutionBase<IkReal> &sol = solutions.GetSolution(i);
            std::vector<IkReal> vsolfree(sol.GetFree().size());
            sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);
            std::vector<double> solution;
            for (std::size_t j = 0; j < solvalues.size(); ++j)
                solution.push_back(solvalues[j]);
            solret.push_back(solution);
        }
    }
    std::cout << "------CY-------------" << std::endl;
    for (int i = 0; i < solret.size(); i++)
    {
        for (int j = 0; j < 3; j++)
        {
            std::cout << solret[i][j] << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}