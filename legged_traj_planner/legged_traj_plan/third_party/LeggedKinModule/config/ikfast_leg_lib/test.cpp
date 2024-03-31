/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <iostream>
/* external project header files */
#include "ikfast_leg_lib.h"
/* internal project header files */

// Change namespace to call IK for different arms/legs
using namespace ikfast_leg;

int main()
{
    bool approx = false;
    std::vector<double> trans = {0.0, 0.0, 0.0};
    std::cout << "Input trans:";
    std::cin >> trans[0] >> trans[1] >> trans[2];
    std::cout << "Approx?:";
    std::cin >> approx;
    if (approx)
    {
        trans.resize(6);
        std::cout << "Input origin:";
        std::cin >> trans[3] >> trans[4] >> trans[5];
    }

    std::vector<std::vector<double>> solret;
    solret = IKFast_trans3D(trans, approx);
    for (uint i = 0; i < solret.size(); i++)
    {
        for (uint j = 0; j < solret[i].size(); j++)
        {
            std::cout << solret[i][j] << " ";
        }
        std::cout << std::endl;
    }

    // double trans[3] = {0.0, 0.0, 0.0};
    // double solret[10][3];
    // IKFast_trans3D(trans, solret);
    // for (int i = 0; i < 10; i++)
    // {
    //     for (int j = 0; j < 3; j++)
    //     {
    //         std::cout << solret[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
}
