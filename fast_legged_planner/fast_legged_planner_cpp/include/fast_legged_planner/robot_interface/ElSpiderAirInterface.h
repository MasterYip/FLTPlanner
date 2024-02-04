/**
 * @file ElSpiderAirInterface.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-02-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */
#include "elspider_air_kin.h"
/* internal project header files */

class ElSpiderAirInterface : public BaseRobotInterface
{
    ElSpiderKin robot_kin_;
public:
    ElSpiderAirInterface(const std::string &urdf, const std::vector<std::string> &package_dirs = {}) : BaseRobotInterface(urdf, package_dirs)
    {
    }
    ~ElSpiderAirInterface()
    {
    }
}