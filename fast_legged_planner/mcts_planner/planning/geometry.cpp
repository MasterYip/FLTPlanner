/**
 * @author Peng Xu
 * 
 * 
 */
#include <geometryFun.h>
namespace GEOMETRY
{


    double mult(MDT::POINT p0,MDT::POINT p1,MDT::POINT p2) //叉积计算,p0为公用节点
    {
        return (p0.x() - p1.x()) * (p0.y() - p2.y()) - (p0.y() - p1.y()) * (p0.x() - p2.x());
    }

    //aa、bb属于同一个矩形 cc、dd属于同一个矩形 相交返回true，不相交返回false
    bool isTwoLineInsection(MDT::POINT aa, MDT::POINT bb, MDT::POINT cc, MDT::POINT dd) 
    {
        //判断两个形成的矩形不相交
        if(std::max(aa.x() , bb.x()) < std::min(cc.x() , dd.x())) return false; 
        if(std::max(aa.y() , bb.y()) < std::min(cc.y() , dd.y())) return false;
        if(std::max(cc.x() , dd.x()) < std::min(aa.x() , bb.x())) return false;
        if(std::max(cc.y() , dd.y()) < std::min(aa.y() , bb.y())) return false;
        //现在已经满足快速排斥实验，那么后面就是跨立实验内容(叉积判断两个线段是否相交)
        if(mult(aa,cc,bb) * mult(aa,bb,dd) < 0) return false; //正确的话也就是aa,bb要在cc或者dd的两边
        if(mult(cc,aa,dd) * mult(cc,dd,bb) < 0) return false;
        return true;
    }



        // direction_需要时单位向量
    // 前提是，带入的点一定在多边形内才行
    // 获得COG到前进方向与支撑多边形的交点的距离
    // float HexapodParameter::getMaxLengthCOGtoPolygon(MDT::POINT RobotCenter,
    //      MDT::POINT direction_,std::vector<MDT::POINT> Pnt,int n_)
    MDT::POINT getIntersection(std::vector<MDT::POINT> Pnt, const MDT::POINT innerPnt, MDT::POINT directionVector)
    {

        bool flag = InPolygon(innerPnt, Pnt.size(), Pnt);
        if (flag == false) {
            std::cerr << "Error: innerPnt is not in the polygon!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            std::exit(1);
        }

        float tmp = directionVector.x()*directionVector.x() + directionVector.y()*directionVector.y();
        directionVector.x() = innerPnt.x() + directionVector.x()/tmp * 2.0;
        directionVector.y() = innerPnt.y() + directionVector.y()/tmp * 2.0;

        for(int i=0;i<Pnt.size()-1;i++)
        {
            if(GEOMETRY::isTwoLineInsection(innerPnt,directionVector, Pnt[i],Pnt[i+1]) == true)
            {
                MDT::POINT insectP_;
                insectP_ = getTwoVectorIntersection(innerPnt,directionVector,Pnt[i],Pnt[i+1]);
                return insectP_;
            }
        }

        MDT::POINT insectP_;
        insectP_ = getTwoVectorIntersection(innerPnt,directionVector,Pnt[Pnt.size()-1],Pnt[0]);
        return insectP_;
    }

    //计算静态稳定裕度使用的子函数
    int dcmp(double x)
    {
        if (fabs(x)< 0.00000000001) return 0;
        else
            return x<0 ? -1 : 1;
    }


    //计算静态稳定裕度使用的子函数
    bool OnSegment(MDT::POINT P1, MDT::POINT P2, MDT::POINT Q)
    {
        MDT::POINT A_,B_;
        A_.x() = P1.x() - Q.x();
        A_.y() = P1.y() - Q.y();

        B_.x() = P2.x() - Q.x();
        B_.y() = P2.y() - Q.y();
        return dcmp(A_.x() * B_.y() -  B_.x()*A_.y()) == 0 && dcmp(A_.x() * B_.x() +  A_.y() * B_.y()) <= 0;
    }


    // 判断点是否在多边形内
    bool InPolygon(MDT::POINT P,int n, std::vector<MDT::POINT> polygon)
    {
        bool flag = false;
        MDT::POINT P1, P2; 
        for (int i = 1, j = n;i <= n;j = i++)
        {
            P1 = polygon[i-1];
            P2 = polygon[j-1];
            if (OnSegment(P1, P2, P)) return true; 
            if ((dcmp(P1.y() - P.y())>0 != dcmp(P2.y() - P.y())>0) && dcmp(P.x() - (P.y() - P1.y())*(P1.x() - P2.x()) / (P1.y() - P2.y()) - P1.x())<0)
                flag = !flag;
        }
        return flag;
    }

    //计算静态稳定裕度使用的子函数
    double distancePtSeg(const MDT::POINT Pnt, const MDT::POINT A, const MDT::POINT B)
    {
        double pqx = B.x() - A.x();
        double pqy = B.y() - A.y();
        double dx = Pnt.x() - A.x();
        double dy = Pnt.y() - A.y();
        double d = pqx * pqx + pqy*pqy;
        double t = pqx*dx + pqy*dy;
        if (d > 0)
            t /= d;
        if (t < 0)
            t = 0;
        else if (t > 1)
            t = 1;

        dx = A.x() + t*pqx - Pnt.x();
        dy = A.y() + t*pqy - Pnt.y();
        return sqrt(dx*dx + dy*dy);
    }


    /**
     * @ : 数学上的操作，判断两条直线是否相交
     * @description:
     * @param {Point} aa：第一条直线上的第一个端点
     * @param {Point} bb：第一条直线上的第二个端点
     * @param {Point} cc：第二条直线上的第一个端点
     * @param {Point} dd：第二条直线上的第二个端点
     * @return {bool}：两条直线是否相交
     */
    bool isTwoLineInsection(const MDT::POINT &aa, const MDT::POINT &bb, const MDT::POINT &cc, const MDT::POINT &dd)
    {
        // 判断两个形成的矩形不相交
        if (std::max(aa.x(), bb.x()) < std::min(cc.x(), dd.x()))
            return false;
        if (std::max(aa.y(), bb.y()) < std::min(cc.y(), dd.y()))
            return false;
        if (std::max(cc.x(), dd.x()) < std::min(aa.x(), bb.x()))
            return false;
        if (std::max(cc.y(), dd.y()) < std::min(aa.y(), bb.y()))
            return false;
        // 现在已经满足快速排斥实验，那么后面就是跨立实验内容(叉积判断两个线段是否相交)
        if (mult(aa, cc, bb) * mult(aa, bb, dd) < 0)
            return false; // 正确的话也就是aa,bb要在cc或者dd的两边
        if (mult(cc, aa, dd) * mult(cc, dd, bb) < 0)
            return false;
        return true;
    }

    /**
     * @ : 数学操作，获得直线和直线的交点
     * @description:
     * @param {Point} line1_p1：第一条直线上的第一个端点
     * @param {Point} line1_p2：第一条直线上的第二个端点
     * @param {Point} line2_p1：第二条直线上的第一个端点
     * @param {Point} line2_p2：第二条直线上的第二个端点
     * @return {MDT::POINT}：两条直线的交点
     */
    MDT::POINT getTwoVectorIntersection(const MDT::POINT &line1_p1, const MDT::POINT line1_p2, const MDT::POINT &line2_p1, const MDT::POINT &line2_p2)
    {
        float a0 = line1_p1.y() - line1_p2.y();
        float b0 = line1_p2.x() - line1_p1.x();
        float c0 = line1_p1.x() * line1_p2.y() - line1_p2.x() * line1_p1.y();

        float a1 = line2_p1.y() - line2_p2.y();
        float b1 = line2_p2.x() - line2_p1.x();
        float c1 = line2_p1.x() * line2_p2.y() - line2_p2.x() * line2_p1.y();

        float D = a0 * b1 - a1 * b0;

        MDT::POINT resultP;
        resultP.x() = (b0 * c1 - b1 * c0) / D;
        resultP.y() = (a1 * c0 - a0 * c1) / D;

        return resultP;
    }

    /**
     * @ : 数学操作，获得直线和圆弧的交点
     * @description:
     * @param {float} R_：圆弧的半径（圆弧位于原点）
     * @param {Point} p_：起点
     * @param {Point} direction_：直线的方向
     * @return {MDT::POINT}：直线和圆弧的交点
     */
    MDT::POINT getLineAndCircleIntersection(const float &R_, const MDT::POINT &p_, const MDT::POINT &direction_)
    {
        MDT::POINT point_c, base_, pr_;
        point_c.x() = direction_.x() + p_.x();
        point_c.y() = direction_.y() + p_.y();

        // base_ = p_;
        base_.x() = point_c.x() - p_.x();
        base_.y() = point_c.y() - p_.y();

        float r_ = (base_.x() * (-p_.x()) + base_.y() * (-p_.y())) /
                    sqrt(base_.x() * base_.x() + base_.y() * base_.y());

        pr_.x() = p_.x() + base_.x() * r_;
        pr_.y() = p_.y() + base_.y() * r_;

        float deta_ = sqrt(R_ * R_ - pr_.x() * pr_.x() - pr_.y() * pr_.y());

        MDT::POINT resultP;
        resultP.x() = pr_.x() + direction_.x() * deta_;
        resultP.y() = pr_.y() + direction_.y() * deta_;

        return resultP;
    }

    /**
     * @ : 获取单腿前进与扇形交点（在扇形坐标系下描述所有点）
     * @description:
     * @param {float} R：扇形半径（圆弧位于原点）
     * @param {Point} p_：当前支撑点
     * @param {Point} direction_：前进方向
     * @return {MDT::POINT}：直线和扇形的交点
     */
    MDT::POINT getIntersectionSector(const float &R_, const MDT::POINT &p_, const MDT::POINT &direction_)
    {

        // 计算扇形弧的两个端点坐标
        MDT::POINT p1_,
            p2_;
        p1_.x() = R_ * cos(0.75f * _PI_);
        p1_.y() = R_ * sin(0.75f * _PI_);

        p2_.x() = R_ * cos(0.25f * _PI_);
        p2_.y() = R_ * sin(0.25f * _PI_);

        float angle_1 = atan2(p1_.y() - p_.y(), p1_.x() - p_.x());
        if (angle_1 < 0)
            angle_1 += 2 * _PI_;

        float angle_2 = atan2(p2_.y() - p_.y(), p2_.x() - p_.x());
        if (angle_2 < 0)
            angle_2 += 2 * _PI_;

        float angle_3 = atan2(0.0f - p_.y(), 0.0f - p_.x());
        if (angle_3 < 0)
            angle_3 += 2 * _PI_;

        float angle_ = atan2(direction_.y(), direction_.x());
        if (angle_ < 0)
            angle_ += 2 * _PI_;

        MDT::POINT resultP;
        if (p_.y() <= sqrt(2) / 2.0f * R_)
        {
            if (angle_ > angle_2 && angle_ < angle_1)
            {
                resultP = getLineAndCircleIntersection(R_, p_, direction_);
            }
            else if (angle_ >= angle_1 && angle_ < angle_3)
            {
                MDT::POINT tmpP;
                tmpP.x() = p_.x() + direction_.x();
                tmpP.y() = p_.y() + direction_.y();
                MDT::POINT tmpO;
                tmpO.x() = 0;
                tmpO.y() = 0;
                resultP = GEOMETRY::getTwoVectorIntersection(p_, tmpP, tmpO, p1_);
            }
            else
            {
                MDT::POINT tmpP;
                tmpP.x() = p_.x() + direction_.x();
                tmpP.y() = p_.y() + direction_.y();
                MDT::POINT tmpO;
                tmpO.x() = 0;
                tmpO.y() = 0;
                resultP = GEOMETRY::getTwoVectorIntersection(p_, tmpP, tmpO, p2_);
            }
        }
        else
        {
            if (angle_ > angle_1 && angle_ < angle_3)
            {
                MDT::POINT tmpP;
                tmpP.x() = p_.x() + direction_.x();
                tmpP.y() = p_.y() + direction_.y();
                MDT::POINT tmpO;
                tmpO.x() = 0;
                tmpO.y() = 0;
                resultP = GEOMETRY::getTwoVectorIntersection(p_, tmpP, tmpO, p1_);
            }
            else if (angle_ >= angle_3 && angle_ <= angle_2)
            {
                MDT::POINT tmpP;
                tmpP.x() = p_.x() + direction_.x();
                tmpP.y() = p_.y() + direction_.y();
                MDT::POINT tmpO;
                tmpO.x() = 0;
                tmpO.y() = 0;
                resultP = GEOMETRY::getTwoVectorIntersection(p_, tmpP, tmpO, p2_);
            }
            else
            {
                resultP = GEOMETRY::getLineAndCircleIntersection(R_, p_, direction_);
            }
        }
        return resultP;
    }



}


