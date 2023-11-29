/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include "ikfast_interface.hpp"
/* internal project header files */

int main()
{
    // double trans[3] = {0.0, 0.0, 0.0};
    // double solret[10][3];
    std::vector<double> trans = {0.0, 0.0, 0.0};
    std::vector<std::vector<double>> solret;
    solret = IKFast_trans3D(trans);
    for (int i = 0; i < solret.size(); i++)
    {
        for (int j = 0; j < solret[i].size(); j++)
        {
            std::cout << solret[i][j] << " ";
        }
        std::cout << std::endl;
    }
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