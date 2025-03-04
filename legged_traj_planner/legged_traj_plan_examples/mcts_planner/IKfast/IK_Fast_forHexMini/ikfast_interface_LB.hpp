// -*- coding: utf-8 -*-
#ifndef ikfast_interface_LB_
#define ikfast_interface_LB_
/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
/* external project header files */

/* internal project header files */
#define IKFAST_HAS_LIBRARY
#include "ikfast_LB.h"
using namespace ikfast_LB;

// Generated function statement
// IKFAST_API int GetNumFreeParameters();

// Interface
bool IKFast_trans3D_LB(const double trans[3], double solret[10][3]);

std::vector<std::vector<double>> IKFast_trans3D_LB(const std::vector<double> trans);

#endif