#include "ParametricSurfaces3.h"

#include <algorithm>
#include <cstdlib>
#include <cmath>

using namespace std;

namespace cagd
{
    // special  constructor
    ParametricSurface3::ParametricSurface3(
            const TriangularMatrix<PartialDerivative> &pd,
            GLdouble u_min, GLdouble u_max,
            GLdouble v_min, GLdouble v_max):
        _pd(pd),
        _u_min(u_min), _u_max(u_max),
        _v_min(v_min), _v_max(v_max)
    {
    }

    DCoordinate3 ParametricSurface3::operator ()(GLdouble u, GLdouble v) const
    {
        return _pd(0, 0)(u, v);
    }

    // generates the approximated tesselated image of the parametric surface
    TriangulatedMesh3* ParametricSurface3::GenerateImage(
        GLuint u_div_point_count,
        GLuint v_div_point_count,
        GLenum usage_flag) const
    {
        if (_pd.GetRowCount() < 2 ||    // i.e., if we cannot evaluate the points and normal vectors of the surface
            u_div_point_count < 2 ||    // i.e., if the number of u-directional subdivion points is too small
            v_div_point_count < 2)      // i.e., if the number of v-directional subdivion points is too small
        {
            return nullptr;
        }

        TriangulatedMesh3 *result = nullptr;

        result = new (nothrow) TriangulatedMesh3(
                u_div_point_count * v_div_point_count,                  // number of unique vertices
                2 * (u_div_point_count - 1) * (v_div_point_count - 1),  // number of triangular faces
                usage_flag);

        if (!result)
        {
            return nullptr;
        }

        // distance between consecutive subdivision points
        GLdouble du = (_u_max - _u_min) / (u_div_point_count - 1);
        GLdouble dv = (_v_max - _v_min) / (v_div_point_count - 1);

        // distance between consecutive subdivision points for texture coordinates
        GLfloat ds = 1.0f / (u_div_point_count - 1);
        GLfloat dt = 1.0f / (v_div_point_count - 1);

        // current triangular face counter
        GLuint current_face = 0;

        for (GLuint i = 0; i < u_div_point_count; ++i)
        {
            GLdouble u = min(_u_min + i * du, _u_max);
            GLfloat  s = min(i * ds, 1.0f);

            for (GLuint j = 0; j < v_div_point_count; ++j)
            {
                GLdouble v = min(_v_min + j * dv, _v_max);
                GLfloat  t = min(j * dt, 1.0f);

                /*
                    3-2
                    |/|
                    0-1
                */

                // unique vertex identifiers
                GLuint index[4];

                index[0] = i * v_div_point_count + j;
                index[1] = index[0] + 1;
                index[2] = index[1] + v_div_point_count;
                index[3] = index[2] - 1;

                // surface point
                (*result)._vertex[index[0]] =  _pd(0, 0)(u, v);

                // the surface normal is obtained as the cross product of the first order partial derivatives
                (*result)._normal[index[0]] =  _pd(1, 0)(u, v);
                (*result)._normal[index[0]] ^= _pd(1, 1)(u, v);
                (*result)._normal[index[0]].normalize();

                // texture coordinates
                (*result)._tex[index[0]].s() = s;
                (*result)._tex[index[0]].t() = t;

                // connectivity information
                if (i < u_div_point_count - 1 && j < v_div_point_count - 1)
                {
                    (*result)._face[current_face][0] = index[0];
                    (*result)._face[current_face][1] = index[1];
                    (*result)._face[current_face][2] = index[2];
                    ++current_face;

                    (*result)._face[current_face][0] = index[0];
                    (*result)._face[current_face][1] = index[2];
                    (*result)._face[current_face][2] = index[3];
                    ++current_face;
                }
            }
        }

        return result;
    }

    GLvoid ParametricSurface3::SetDefinitionDomain(GLdouble u_min, GLdouble u_max, GLdouble v_min, GLdouble v_max)
    {
        this->_u_min = u_min;
        this->_u_max = u_max;
        this->_v_min = v_min;
        this->_v_max = v_max;
    }

    GLvoid ParametricSurface3::GetDefinitionDomain(GLdouble& u_min, GLdouble& u_max, GLdouble& v_min, GLdouble& v_max) const
    {
        u_min = this->_u_min;
        u_max = this->_u_max;
        v_min = this->_v_min;
        v_max = this->_v_max;
    }
}
