/**
 * @file CollisionPenalty.h
 * @author Master Yip (2205929492@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-04-08
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/* related header files */

/* c system header files */

/* c++ standard library header files */

/* external project header files */

/* internal project header files */
#include "legged_traj_plan/robot_interface/ElSpiderAirInterface.h"
#include "legged_traj_plan/perception_interface/GridMapInterface.h"

class CollisionPenalty
{
private:
    ElSpiderAirInterface &robot_interface_;
    GridMapInterface &gridmap_interface_;

public:
    CollisionPenalty(ElSpiderAirInterface &robot_interface, GridMapInterface &gridmap_interface)
        : robot_interface_(robot_interface), gridmap_interface_(gridmap_interface)
    {
    }
}