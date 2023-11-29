// -*- coding: utf-8 -*-
#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */
#include <vector>
/* external project header files */

/* internal project header files */
#define IKFAST_HAS_LIBRARY
#include "ikfast.h"
using namespace ikfast;

// Generated function statement
IKFAST_API int GetNumFreeParameters();

bool ComputeIk(const IkReal *eetrans, const IkReal *eerot, const IkReal *pfree, IkSolutionListBase<IkReal> &solutions);

// Interface
bool IKFast_trans3D(const double trans[3], double solret[10][3]);

std::vector<std::vector<double>> IKFast_trans3D(const std::vector<double> trans);