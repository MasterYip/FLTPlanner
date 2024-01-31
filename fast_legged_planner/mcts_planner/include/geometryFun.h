#ifndef GEOMETRY_H
#define GEOMETRY_H
#include <iostream>
#include <vector>
#include "myDataType.h"




namespace GEOMETRY
{
    MDT::POINT getIntersection(std::vector<MDT::POINT> pnts, MDT::POINT innerPnt, MDT::POINT directionVector);
    bool InPolygon(MDT::POINT P,int n, std::vector<MDT::POINT> polygon);
    double distancePtSeg(const MDT::POINT Pnt, const MDT::POINT A, const MDT::POINT B);

    int dcmp(double x);



    MDT::POINT getTwoVectorIntersection(const MDT::POINT &line1_p1, const MDT::POINT line1_p2, 
        const MDT::POINT &line2_p1, const MDT::POINT &line2_p2);


    // int dcmp2(double x)
    // {
    //     if (fabs(x)< 0.00000000001) return 0;
    //     else
    //         return x<0 ? -1 : 1;
    // }
    MDT::POINT getIntersectionSector(const float &R_, const MDT::POINT &p_, const MDT::POINT &direction_);
}








#endif
