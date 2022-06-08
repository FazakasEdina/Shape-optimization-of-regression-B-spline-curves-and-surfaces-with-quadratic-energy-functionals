#include <cmath>
#include "TestFunctions.h"
#include "../Core/Constants.h"

using namespace cagd;
using namespace std;

GLdouble circle::u_min = 0;
GLdouble circle::u_max = +TWO_PI;

DCoordinate3 circle::d0(GLdouble u)
{
    return DCoordinate3(cos(u), sin(u), 0);
}

DCoordinate3 circle::d1(GLdouble u)
{
    GLdouble c = cos(u), s = sin(u);
    return DCoordinate3(- s, c, 1.0);
}

DCoordinate3 circle::d2(GLdouble u)
{
    GLdouble c = cos(u), s = sin(u);
    return DCoordinate3(- c, - s, 0);
}

GLdouble spiral_on_cone::u_min = -TWO_PI;
GLdouble spiral_on_cone::u_max = +TWO_PI;

DCoordinate3 spiral_on_cone::d0(GLdouble u)
{
    return DCoordinate3(u * cos(u), u * sin(u), u);
}

DCoordinate3 spiral_on_cone::d1(GLdouble u)
{
    GLdouble c = cos(u), s = sin(u);
    return DCoordinate3(c - u * s, s + u * c, 1.0);
}

DCoordinate3 spiral_on_cone::d2(GLdouble u)
{
    GLdouble c = cos(u), s = sin(u);
    return DCoordinate3(-2.0 * s - u * c, 2.0 * c - u * s, 0);
}

GLdouble spiral_on_cylinder::u_min = -TWO_PI;
GLdouble spiral_on_cylinder::u_max = +TWO_PI;

DCoordinate3 spiral_on_cylinder::d0(GLdouble u)
{
    //r = 1;
    return DCoordinate3(cos(u), sin(u), u);
}

DCoordinate3 spiral_on_cylinder::d1(GLdouble u)
{
    return DCoordinate3(- sin(u), cos(u), 1.0);
}

DCoordinate3 spiral_on_cylinder::d2(GLdouble u)
{
    return DCoordinate3(- cos(u), - sin(u), 0.0);
}

GLdouble hypotrochoid_curve::u_min = 0;
GLdouble hypotrochoid_curve::u_max = 3 * TWO_PI;

DCoordinate3 hypotrochoid_curve::d0(GLdouble u)
{
    GLdouble R = 5.0;
    GLdouble r = 3.0;
    GLdouble d = 4.0;
    return DCoordinate3( (R - r) * cos(u) + d * cos ((R - r) * u / r),
                         (R - r) * sin(u) - d * sin ((R - r) * u / r),
                         0.0);
}

DCoordinate3 hypotrochoid_curve::d1(GLdouble u)
{
    GLdouble R = 5.0;
    GLdouble r = 3.0;
    GLdouble d = 4.0;
    return DCoordinate3( - (R - r) * sin(u) - d * (R - r) * sin ((R - r) * u / r) / r,
                         (R - r) * cos(u) - d * (R - r) * cos ((R - r) * u / r) / r,
                         0.0);
}

DCoordinate3 hypotrochoid_curve::d2(GLdouble u)
{
    GLdouble R = 5.0;
    GLdouble r = 3.0;
    GLdouble d = 4.0;
    return DCoordinate3( - (R - r) * cos(u) - d * (R - r) * (R - r) * cos ((R - r) * u / r) / (r * r),
                         - (R - r) * sin(u) + d * (R - r) * (R - r) * sin ((R - r) * u / r) / (r * r),
                         0.0);
}

GLdouble vivians_curve::u_min = -TWO_PI;
GLdouble vivians_curve::u_max = +TWO_PI;

DCoordinate3 vivians_curve::d0(GLdouble u)
{
    // r = a/2
    GLdouble a = 3;
    return DCoordinate3(a/2 * ( 1 + cos(u)),
                        a/2 * sin (u),
                        -a * sin(u/2));
}

DCoordinate3 vivians_curve::d1(GLdouble u)
{
    GLdouble a = 3;
    return DCoordinate3( - a/2 *  sin(u),
                        a/2 * cos (u),
                         - a * cos(u/2) / 2);
}

DCoordinate3 vivians_curve::d2(GLdouble u)
{
    GLdouble a = 3;
    return DCoordinate3( - a/2 *  cos(u),
                         - a/2 * sin (u),
                         a * sin(u/2) / 4);
}


GLdouble rose_curve::u_min = 0.0;
GLdouble rose_curve::u_max = PI;
GLdouble rose_curve::a = 2.0;
GLdouble rose_curve::k = 3.0;

DCoordinate3 rose_curve::d0(GLdouble u)
{
    // x = r * cos(theta)    r = a * cos(k * theta)
    // y = r * sin(theta)
    // k = 7 | a = 1   ->u_max = PI
    // k = 2/3 | a = 2 ->u_max = 3 * TWO_PI
    return DCoordinate3(a * cos(k * u) * cos(u),
                        a * cos (k * u) * sin(u),
                        0.0 );

}

DCoordinate3 rose_curve::d1(GLdouble u)
{
    return DCoordinate3( - k * a * sin(k * u) * cos(u) - a * cos(k * u) * sin(u),
                         - k * a * sin(k * u) * sin(u) + a * cos(k * u) * cos(u),
                        0.0 );
}

DCoordinate3 rose_curve::d2(GLdouble u)
{
    return DCoordinate3( - (a * k * k + a) * cos(k * u) * cos(u) + 2 * a * k * sin(k * u) * sin(u),
                         - (a * k * k + a) * cos(k * u) * sin(u) - 2 * a * k * sin(k * u) * cos(u),
                        0.0 );
}

GLdouble epitrochoid::u_min = -PI;
GLdouble epitrochoid::u_max = PI;
GLdouble epitrochoid::R = 1.0;
GLdouble epitrochoid::r = 0.2;
GLdouble epitrochoid::d = 0.4;

DCoordinate3 epitrochoid::d0(GLdouble u)
{
    return DCoordinate3( (R + r) * cos(u) - d * cos(((R + r) / r) * u),
                         (R + r) * sin(u) - d * sin(((R + r) / r) * u)
                         );
}

DCoordinate3 epitrochoid::d1(GLdouble u)
{
    return DCoordinate3( - (R + r) * sin(u) + d * ((R + r) / r) * sin(((R + r) / r) * u),
                         (R + r) * cos(u) - d * ((R + r) / r) * cos(((R + r) / r) * u)
                         );
}

DCoordinate3 epitrochoid::d2(GLdouble u)
{
    return DCoordinate3( - (R + r) * cos(u) + ((R + r) * (R + r) / (r * r)) * d * cos(((R + r) / r) * u),
                         - (R + r) * sin(u) + ((R + r) * (R + r) / (r * r)) * d * sin(((R + r) / r) * u)
                         );
}

////////////////////////////////
/// 3d geometrics
////////////////////////////////
GLdouble torus::u_min = 0.0;
GLdouble torus::u_max = TWO_PI;
GLdouble torus::v_min = 0.0;
GLdouble torus::v_max = TWO_PI;
GLdouble torus::R = 4.0;
GLdouble torus::r = 1.0;

DCoordinate3 torus::d00(GLdouble u, GLdouble v)
{
   return DCoordinate3 (
               (R + r * cos(u)) * cos(v),
               (R + r * cos(u)) * sin(v),
               r * sin(u)
               );

}

DCoordinate3 torus::d10(GLdouble u, GLdouble v)
{
    return DCoordinate3 (
                r * sin(u) * cos(v),
                r * sin(u) * sin(v),
                - r * cos(u)
                );
}
DCoordinate3 torus::d01(GLdouble u, GLdouble v)
{
    return DCoordinate3 (
                - (R + r * cos(u)) * sin(v),
                (R + r * cos(u)) * cos(v),
                0.0
                );
}

GLdouble sphere::u_min = 0.0;
GLdouble sphere::u_max = TWO_PI;
GLdouble sphere::v_min = 0.0;
GLdouble sphere::v_max = PI;
GLdouble sphere::r = 1;

DCoordinate3 sphere::d00(GLdouble u, GLdouble v)
{
    return DCoordinate3 (
                r * sin(v) * cos(u),
                r * sin(v) * sin(u),
                r * cos(v)
                );
}
DCoordinate3 sphere::d10(GLdouble u, GLdouble v)
{
    return DCoordinate3 (
                r * cos(v) * cos(u),
                r * cos(v) * sin(u),
                - r * sin(v)
                );
}
DCoordinate3 sphere::d01(GLdouble u, GLdouble v)
{

    return DCoordinate3 (
                - r * sin(v) * sin(u),
                r * sin(v) * cos(u),
                0.0
                );
}

GLdouble cylinder::u_min = -3;
GLdouble cylinder::u_max = 3;
GLdouble cylinder::v_min = 0.0;
GLdouble cylinder::v_max = TWO_PI;
GLdouble cylinder::r = 1.0;

DCoordinate3 cylinder::d00(GLdouble u, GLdouble v)
{
    return DCoordinate3 (
                r * cos(v),
                r * sin(v),
                u
                );

//    return DCoordinate3 (
//                r * cos(u),
//                r * sin(u),
//                v
//                );
}

DCoordinate3 cylinder::d10(GLdouble, GLdouble v)
{
    return DCoordinate3 (
                - r * sin(v),
                r * cos(v),
                0.0
                );
}

DCoordinate3 cylinder::d01(GLdouble, GLdouble)
{
    return DCoordinate3 (
                0.0,
                0.0,
                1.0
                );
}

GLdouble cone::u_min = 0.0;
GLdouble cone::u_max = 3;
GLdouble cone::v_min = 0.0;
GLdouble cone::v_max = TWO_PI;
GLdouble cone::h = 3;
GLdouble cone::r = 1;

DCoordinate3 cone::d00(GLdouble u, GLdouble v)
{
    return DCoordinate3 (
                (h - u) * r * cos(v) / h,
                (h - u) * r * sin(v) / h,
                u
                );
}

DCoordinate3 cone::d10(GLdouble u, GLdouble v)
{
    return DCoordinate3 (
                - (h - u) * r * sin(v) / h,
                (h - u) * r * cos(v) / h,
                0.0
                );
}

DCoordinate3 cone::d01(GLdouble, GLdouble v)
{
    return DCoordinate3 (
                - r * cos(v) / h,
                - r * sin(v) / h,
                1.0
                );
}

GLdouble pseudosphere::u_min = 0.0;
GLdouble pseudosphere::u_max = TWO_PI;
GLdouble pseudosphere::v_min = 0.0;
GLdouble pseudosphere::v_max = PI;

DCoordinate3 pseudosphere::d00(GLdouble u, GLdouble v)
{
   return DCoordinate3 (
               cos(u) * sin(v),
               sin(u) * sin(v),
               cos(v) + log(tan(v/2))
               );
}

DCoordinate3 pseudosphere::d10(GLdouble u, GLdouble v)
{
    return DCoordinate3 (
                - sin(u) * sin(v),
                cos(u) * sin(v),
                0.0
                );
}
DCoordinate3 pseudosphere::d01(GLdouble u, GLdouble v)
{
    return DCoordinate3 (
                cos(u) * cos(v),
                sin(u) * cos(v),
                - sin(v) + 1 / (2 * cos(v/2) * cos(v/2) * tan(v/2) )
                );
}

GLdouble simple_surface::u_min = -3.0;
GLdouble simple_surface::u_max = 3.0;
GLdouble simple_surface::v_min = -3.0;
GLdouble simple_surface::v_max = 3.0;

DCoordinate3 simple_surface::d00(GLdouble u, GLdouble v)
{
   return DCoordinate3 (
               u,
               v,
               sin(u*u + v*v) / sqrt(u*u + v*v + 1)
               );
}
