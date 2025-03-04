
#include "ikfast_interface_RB.hpp"
#include "ikfast_interface_RF.hpp"
#include "ikfast_interface_LB.hpp"
#include "ikfast_interface_LF.hpp"
// #include "ik_fast_comm.h"






int main()
{
    
    std::cout << "------RB-------------" << std::endl;
    // double trans[3] = {0.0, 0.0, 0.0};
    // double solret[10][3];
    std::vector<double> trans = {-0.13028, -0.2545, -0.31143};
    std::vector<std::vector<double>> solret;
    solret = IKFast_trans3D_RB(trans);
    for (int i = 0; i < solret.size(); i++)
    {
        for (int j = 0; j < solret[i].size(); j++)
        {
            std::cout << solret[i][j] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "------RF-------------" << std::endl;

    solret = IKFast_trans3D_RF(trans);
    for (int i = 0; i < solret.size(); i++)
    {
        for (int j = 0; j < solret[i].size(); j++)
        {
            std::cout << solret[i][j] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "------LB-------------" << std::endl;

    solret = IKFast_trans3D_LB(trans);
    for (int i = 0; i < solret.size(); i++)
    {
        for (int j = 0; j < solret[i].size(); j++)
        {
            std::cout << solret[i][j] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "------LF-------------" << std::endl;

    solret = IKFast_trans3D_LF(trans);
    for (int i = 0; i < solret.size(); i++)
    {
        for (int j = 0; j < solret[i].size(); j++)
        {
            std::cout << solret[i][j] << " ";
        }
        std::cout << std::endl;
    }


}