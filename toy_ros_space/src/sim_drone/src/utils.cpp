#include "utils.h"

Matrix3d crossMat(const Vector3d& v)
{
    Matrix3d m;
    m <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return m;
}

//
// Omega( v ) = -[v] v
//               -v  0
//
Matrix4d Omega(const Vector3d& v)
{
    Matrix4d m;
    m <<     0,  v(2), -v(1),  v(0),
         -v(2),     0,  v(0),  v(1),
          v(1), -v(0),     0,  v(2),
         -v(0), -v(1), -v(2),     0;
    return m;
}

