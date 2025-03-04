#include "constrains/util.hh"

Matrix3 crossMatrix(const Vector3 &x)
{
    Matrix3 res;
    res.setZero();
    res(0, 1) = -x(2);
    res(0, 2) = x(1);
    res(1, 0) = x(2);
    res(1, 2) = -x(0);
    res(2, 0) = -x(1);
    res(2, 1) = x(0);
    return res;
}
