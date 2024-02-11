/**
 * @file polycorridor.cpp
 * @author Master Yip (2205929492@qq.com)
 * @brief 
 * @version 0.1
 * @date 2024-02-11
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "geo_utils/polycorridor.hpp"

PolyCorridor::PolyCorridor(const std::vector<Polyhedra> &polys) : polys_(polys)
{
    poly_size = polys.size();
}

void PolyCorridor::appendPoly(const Polyhedra &poly)
{
    polys_.emplace_back(poly);
    poly_size++;
}
