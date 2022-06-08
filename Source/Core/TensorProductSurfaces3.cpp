#include "TensorProductSurfaces3.h"
#include "RealSquareMatrices.h"
#include "Constants.h"
#include <algorithm>

using namespace cagd;
using namespace std;

// special constructor
TensorProductSurface3::PartialDerivatives::PartialDerivatives(GLuint maximum_order_of_partial_derivatives):
    TriangularMatrix<DCoordinate3>(maximum_order_of_partial_derivatives + 1)
{
}

GLvoid TensorProductSurface3::PartialDerivatives::LoadNullVectors()
{
    for(GLuint i = 0; i < this->GetRowCount(); i++)
    {
        for(GLuint j = 0; j <= i; j++)
        {
            _data[i][j] = DCoordinate3(0, 0, 0);
        }
    }
}

TensorProductSurface3::TensorProductSurface3(
        GLdouble u_min, GLdouble u_max,
        GLdouble v_min, GLdouble v_max,
        GLuint row_count, GLuint column_count,
        GLboolean u_closed, GLboolean v_closed):
    _u_closed(u_closed),
    _v_closed(v_closed),
    _vbo_data(0),
    _u_min(u_min),
    _u_max(u_max),
    _v_min(v_min),
    _v_max(v_max),
    _data(row_count, column_count),
    _fragments(10)
{
}

TensorProductSurface3::TensorProductSurface3(const TensorProductSurface3& surface):
    _u_closed(surface._u_closed),
    _v_closed(surface._v_closed),
    _vbo_data(0),
    _u_min(surface._u_min),
    _u_max(surface._u_max),
    _v_min(surface._v_min),
    _v_max(surface._v_max),
    _data(surface._data.GetRowCount(), surface._data.GetColumnCount()),
    _fragments(surface._fragments)
{
}

TensorProductSurface3& TensorProductSurface3::operator=(const TensorProductSurface3& surface)
{
    if(this != &surface)
    {
        _u_min = surface._u_min;
        _u_max = surface._u_max;
        _v_min = surface._v_min;
        _v_max = surface._v_max;
        _vbo_data = surface._vbo_data;
        _u_closed = surface._u_closed;
        _v_closed = surface._v_closed;
        _data = surface._data;
        _fragments = surface._fragments;
    }

    return *this;
}

GLvoid TensorProductSurface3::SetUInterval(GLdouble u_min, GLdouble u_max)
{
    _u_min = u_min;
    _u_max = u_max;
}

GLvoid TensorProductSurface3::SetVInterval(GLdouble v_min, GLdouble v_max)
{
    _v_min = v_min;
    _v_max = v_max;
}

GLvoid TensorProductSurface3::GetUInterval(GLdouble &u_min, GLdouble &u_max) const
{
    u_min = _u_min;
    u_max = _u_max;
}

GLvoid TensorProductSurface3::GetVInterval(GLdouble &v_min, GLdouble &v_max) const
{
    v_min = _v_min;
    v_max = _v_max;
}

GLboolean TensorProductSurface3::SetData(GLuint row, GLuint column, GLdouble x, GLdouble y, GLdouble z)
{
    if(row < 0 || column < 0 || row > _data.GetRowCount() || column > _data.GetColumnCount())
        return GL_FALSE;

    _data(row, column) = DCoordinate3(x, y, z);

    return GL_TRUE;
}

GLboolean TensorProductSurface3::SetData(GLuint row, GLuint column, const DCoordinate3 &point)
{
    if(row < 0 || column < 0 || row > _data.GetRowCount() || column > _data.GetColumnCount())
        return GL_FALSE;

    _data(row, column) = point;

    return GL_TRUE;
}

GLboolean TensorProductSurface3::GetData(GLuint row, GLuint column, GLdouble &x, GLdouble &y, GLdouble &z) const
{
    if(row >= _data.GetRowCount() || column >= _data.GetColumnCount())
        return GL_FALSE;

    x = _data(row, column)[0];
    y = _data(row, column)[1];
    z = _data(row, column)[2];

    return GL_TRUE;
}

GLboolean TensorProductSurface3::GetData(GLuint row, GLuint column, DCoordinate3 &point) const
{
    if(row >= _data.GetRowCount() || column >= _data.GetColumnCount())
        return GL_FALSE;

    point = _data(row, column);

    return GL_TRUE;
}

DCoordinate3 TensorProductSurface3::operator()(GLuint row, GLuint column) const
{
    return _data(row, column);
}

DCoordinate3& TensorProductSurface3::operator()(GLuint row, GLuint column)
{
    return _data(row, column);
}

// generates the image (i.e., the approximating triangulated mesh) of the tensor product surface
TriangulatedMesh3* TensorProductSurface3::GenerateImage(GLuint u_div_point_count, GLuint v_div_point_count,
                                                        RowMatrix<GLdouble> &quadratic_energies,
                                                        ImageColorScheme color_sheme,
                                                        GLenum usage_flag)
{
    if (u_div_point_count <= 1 || v_div_point_count <= 1)
        return GL_FALSE;

    if ( (u_div_point_count - 1) % 2)
    {
        u_div_point_count++;
    }

    if ( (v_div_point_count - 1) % 2)
    {
        v_div_point_count++;
    }

    // calculating number of vertices, unit normal vectors and texture coordinates
    GLuint vertex_count = u_div_point_count * v_div_point_count;

    // calculating number of triangular faces
    GLuint face_count = 2 * (u_div_point_count - 1) * (v_div_point_count - 1);

    TriangulatedMesh3 *result = nullptr;
    result = new TriangulatedMesh3(vertex_count, face_count, usage_flag);

    if (!result)
        return nullptr;

    // uniform subdivision grid in the definition domain
    GLdouble du = (_u_max - _u_min) / (u_div_point_count - 1);
    GLdouble dv = (_v_max - _v_min) / (v_div_point_count - 1);

    // uniform subdivision grid in the unit square
    GLfloat sdu = 1.0f / (u_div_point_count - 1);
    GLfloat tdv = 1.0f / (v_div_point_count - 1);

    // for face indexing
    GLuint current_face = 0;

    // partial derivatives of order 0, 1, 2, and 3
    PartialDerivatives pd;

    _fragments.ResizeRows(10);
    _fragments.ResizeColumns(vertex_count);
    GLdouble min_value = numeric_limits<GLdouble>::max();
    GLdouble max_value = -numeric_limits<GLdouble>::max();

    for (GLuint i = 0; i < u_div_point_count; ++i)
    {
        GLdouble u = min(_u_min + i * du, _u_max);
        GLfloat  s = min(i * sdu, 1.0f);
        for (GLuint j = 0; j < v_div_point_count; ++j)
        {
            GLdouble v = min(_v_min + j * dv, _v_max);
            GLfloat  t = min(j * tdv, 1.0f);

            /*
                3-2
                |/|
                0-1
            */
            GLuint index[4];

            index[0] = i * v_div_point_count + j;
            index[1] = index[0] + 1;
            index[2] = index[1] + v_div_point_count;
            index[3] = index[2] - 1;

            // calculating all needed surface data
            if (!CalculatePartialDerivatives(2, u, v, pd))
            {
                continue;
            }

            // surface point
            (*result)._vertex[index[0]] = pd(0, 0);

            // unit surface normal
            (*result)._normal[index[0]] = pd(1, 0);
            (*result)._normal[index[0]] ^= pd(1, 1);
            DCoordinate3 normal = (*result)._normal[index[0]];
            (*result)._normal[index[0]].normalize();

            // texture coordinates
            (*result)._tex[index[0]].s() = s;
            (*result)._tex[index[0]].t() = t;

            // fragments calculation
            _fragments(0, index[0]) = 0.0;
            _fragments(1, index[0]) = normal.length();
            _fragments(2, index[0]) = CalculateEnergy(GAUSSIAN_CURVATURE_FRAGMENT, pd);
            _fragments(3, index[0]) = CalculateEnergy(MEAN_CURVATURE_FRAGMENT, pd);
            _fragments(4, index[0]) = CalculateEnergy(WILLMORE_ENERGY_FRAGMENT, pd);
            _fragments(5, index[0]) = CalculateEnergy(LOG_WILLMORE_ENERGY_FRAGMENT, pd);
            _fragments(6, index[0]) = CalculateEnergy(UMBILIC_DEVIATION_ENERGY_FRAGMENT, pd);
            _fragments(7, index[0]) = CalculateEnergy(LOG_UMBILIC_DEVIATION_ENERGY_FRAGMENT, pd);
            _fragments(8, index[0]) = CalculateEnergy(TOTAL_CURVATURE_ENERGY_FRAGMENT, pd);
            _fragments(9, index[0]) = CalculateEnergy(LOG_TOTAL_CURVATURE_ENERGY_FRAGMENT, pd);

            if (min_value > _fragments(color_sheme, index[0]))
            {
                min_value = _fragments(color_sheme, index[0]);
            }

            if (max_value < _fragments(color_sheme, index[0]))
            {
                max_value = _fragments(color_sheme, index[0]);
            }

            // faces
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

    Matrix<GLdouble> weight (3, 3) ;
    weight (0, 0) = weight (0, 2) = weight (2, 0)= weight (2 , 2) = 1.0;
    weight (0, 1) = weight (1, 0) = weight (1 , 2)= weight (2, 1) = 4.0;
    weight (1, 1) = 16.0 ;

    quadratic_energies.ResizeColumns(10);
    for(GLint q = 1; q < 10; q++)
    {
        quadratic_energies[q] = 0.0;
        for (GLint i = 1; i < (GLint)u_div_point_count; i+=2)
        {
            for (GLint j = 1; j < (GLint)v_div_point_count; j+=2)
            {
                GLdouble S = 0.0;
                for (GLint k = -1; k <= 1; k++)
                {
                    for (GLint l = -1; l <= 1; l++)
                    {
                        S += weight(k + 1, l + 1) * _fragments(q, (i + k) * v_div_point_count + j + l);
                    }
                }
                quadratic_energies[q] += S;
            }
        }
        quadratic_energies[q] *= du * dv  / 9.0;
    }

    for (GLuint i_j = 0; i_j < vertex_count; i_j++)
    {
        result->_color[i_j] = ColdToHotColormap(_fragments(color_sheme, i_j), min_value, max_value);
    }

    return result;
}

// Generate image in a given interval
TriangulatedMesh3* TensorProductSurface3::GenerateImageInAGivenInterval(
        GLuint u_div_point_count, GLuint v_div_point_count,
        GLdouble u_min, GLdouble u_max, GLdouble v_min, GLdouble v_max,
        ImageColorScheme color_sheme,
        GLenum usage_flag) const
{
    if (u_div_point_count <= 1 || v_div_point_count <= 1)
        return GL_FALSE;

    // calculating number of vertices, unit normal vectors and texture coordinates
    GLuint vertex_count = u_div_point_count * v_div_point_count;

    // calculating number of triangular faces
    GLuint face_count = 2 * (u_div_point_count - 1) * (v_div_point_count - 1);

    TriangulatedMesh3 *result = nullptr;
    result = new TriangulatedMesh3(vertex_count, face_count, usage_flag);

    if (!result)
        return nullptr;

    // uniform subdivision grid in the definition domain
    GLdouble du = (u_max - EPS - u_min) / (u_div_point_count - 1);
    GLdouble dv = (v_max - EPS - v_min) / (v_div_point_count - 1);

    // for face indexing
    GLuint current_face = 0;

    // uniform subdivision grid in the unit square
    GLfloat sdu = 1.0f / (u_div_point_count - 1);
    GLfloat tdv = 1.0f / (v_div_point_count - 1);

    // partial derivatives of order 0, 1, 2, and 3
    PartialDerivatives pd;

    RowMatrix<GLdouble> fragments(vertex_count);
    GLdouble min_value = numeric_limits<GLdouble>::max();
    GLdouble max_value = -numeric_limits<GLdouble>::max();

    GLuint max_order_of_partial_derivatives = color_sheme == DEFAULT_NULL_FRAGMENT || color_sheme == NORMAL_LENGTH_FRAGMENT ? 1 : 2;
    for (GLuint i = 0; i < u_div_point_count; ++i)
    {
        GLdouble u = min(u_min + i * du, u_max - EPS);
        GLfloat  s = min(i * sdu, 1.0f);
        for (GLuint j = 0; j < v_div_point_count; ++j)
        {
            GLdouble v = min(v_min + j * dv, v_max - EPS);
            GLfloat  t = min(j * tdv, 1.0f);

            /*
                3-2
                |/|
                0-1
            */
            GLuint index[4];

            index[0] = i * v_div_point_count + j;
            index[1] = index[0] + 1;
            index[2] = index[1] + v_div_point_count;
            index[3] = index[2] - 1;

            // calculating all needed surface data
            if (!CalculatePartialDerivatives(max_order_of_partial_derivatives, u, v, pd))
            {
                continue;
            }

            // surface point
            (*result)._vertex[index[0]] = pd(0, 0);

            // unit surface normal
            (*result)._normal[index[0]] = pd(1, 0);
            (*result)._normal[index[0]] ^= pd(1, 1);
            DCoordinate3 normal = (*result)._normal[index[0]];
            (*result)._normal[index[0]].normalize();

            if (color_sheme == NORMAL_LENGTH_FRAGMENT)
            {
                fragments[index[0]] = normal.length();
            }
            else
            {
                if (color_sheme != DEFAULT_NULL_FRAGMENT)
                {
                    fragments[index[0]]  = CalculateEnergy(color_sheme, pd);
                }
                else
                {
                    fragments[index[0]] = 0.0;
                }
            }

            if (fragments[index[0]] < min_value)
            {
                min_value = fragments[index[0]];
            }

            if (fragments[index[0]] > max_value)
            {
                max_value = fragments[index[0]];
            }

            // texture coordinates
            (*result)._tex[index[0]].s() = s;
            (*result)._tex[index[0]].t() = t;

            // faces
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

        for (GLuint i = 0; i < vertex_count; i++)
        {
            (*result)._color[i] = ColdToHotColormap(fragments[i], min_value, max_value);
        }
    }

    return result;
}

// ensures interpolation, i.e. s(u_i, v_j) = d_{i,j}
GLboolean TensorProductSurface3::UpdateDataForInterpolation(const RowMatrix<GLdouble>& u_knot_vector, const ColumnMatrix<GLdouble>& v_knot_vector, Matrix<DCoordinate3>& data_points_to_interpolate)
{
    GLuint row_count = _data.GetRowCount();
    if (!row_count)
        return GL_FALSE;

    GLuint column_count = _data.GetColumnCount();
    if (!column_count)
        return GL_FALSE;

    if (u_knot_vector.GetColumnCount() != row_count || v_knot_vector.GetRowCount() != column_count || data_points_to_interpolate.GetRowCount() != row_count || data_points_to_interpolate.GetColumnCount() != column_count)
        return GL_FALSE;

    // 1: calculate the u-collocation matrix and perfom LU-decomposition on it
    RowMatrix<GLdouble> u_blending_values;

    RealSquareMatrix u_collocation_matrix(row_count);

    for (GLuint i = 0; i < row_count; ++i)
    {
        if (!UBlendingFunctionValues(u_knot_vector(i), u_blending_values))
            return GL_FALSE;
        u_collocation_matrix.SetRow(i, u_blending_values);
    }

    if (!u_collocation_matrix.PerformLUDecomposition())
        return GL_FALSE;

    // 2: calculate the v-collocation matrix and perform LU-decomposition on it
    RowMatrix<GLdouble> v_blending_values;

    RealSquareMatrix v_collocation_matrix(column_count);

    for (GLuint j = 0; j < column_count; ++j)
    {
        if (!VBlendingFunctionValues(v_knot_vector(j), v_blending_values))
            return GL_FALSE;
        v_collocation_matrix.SetRow(j, v_blending_values);
    }

    if (!v_collocation_matrix.PerformLUDecomposition())
        return GL_FALSE;

    // 3:   for all fixed j in {0, 1,..., column_count} determine control points
    //
    //      a_k(v_j) = sum_{l=0}^{column_count} _data(l, j) G_l(v_j), k = 0, 1,..., row_count
    //
    //      such that
    //
    //      sum_{k=0}^{row_count} a_k(v_j) F_k(u_i) = data_points_to_interpolate(i, j),
    //
    //      for all i = 0, 1,..., row_count.
    Matrix<DCoordinate3> a(row_count, column_count);
    if (!u_collocation_matrix.SolveLinearSystem(data_points_to_interpolate, a))
        return GL_FALSE;

    // 4:   for all fixed i in {0, 1,..., row_count} determine control point
    //
    //      _data[i][j], j = 0, 1,..., column_count
    //
    //      such that
    //
    //      sum_{l=0}^{column_count} _data(i, l) G_l(v_j) = a_i(v_j)
    //
    //      for all j = 0, 1,..., column_count.
    if (!v_collocation_matrix.SolveLinearSystem(a, _data, GL_FALSE))
        return GL_FALSE;

    return GL_TRUE;
}

// VBO handling
GLvoid TensorProductSurface3::DeleteVertexBufferObjectsOfData()
{
    if (_vbo_data)
    {
        glDeleteBuffers(1, &_vbo_data);
        _vbo_data = 0;
    }
}

GLboolean TensorProductSurface3::RenderData(GLenum render_mode) const
{
    if (!_vbo_data)
        return GL_FALSE;

    if (render_mode != GL_LINE_STRIP && render_mode != GL_LINE_LOOP && render_mode != GL_POINTS)
        return GL_FALSE;

    glEnableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, _vbo_data);
    glVertexPointer(3, GL_FLOAT, 0, (const GLvoid*)0);

    GLuint offset = 0;
    for(GLuint i = 0; i < _data.GetRowCount(); i++, offset += _data.GetColumnCount())
    {
        glDrawArrays(_v_closed ? GL_LINE_LOOP : GL_LINE_STRIP, offset, _data.GetColumnCount());
    }

    for(GLuint i = 0; i < _data.GetColumnCount(); i++, offset += _data.GetRowCount())
    {
        glDrawArrays(_u_closed ? GL_LINE_LOOP : GL_LINE_STRIP, offset, _data.GetRowCount());
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDisableClientState(GL_VERTEX_ARRAY);

    return GL_TRUE;
}

GLboolean TensorProductSurface3::RenderDerivates(GLuint u_div_point_count, GLuint v_div_point_count, GLfloat scale) const
{
    glBegin(GL_LINES);

    // uniform subdivision grid in the definition domain
    GLdouble du = (_u_max - _u_min) / (u_div_point_count - 1);
    GLdouble dv = (_v_max - _v_min) / (v_div_point_count - 1);

    PartialDerivatives pd;

    for (GLuint i = 0; i < u_div_point_count; ++i)
    {
        GLdouble u = min(_u_min + i * du, _u_max);
        for (GLuint j = 0; j < v_div_point_count; ++j)
        {
            GLdouble v = min(_v_min + j * dv, _v_max);

            if(!CalculatePartialDerivatives(1, u, v, pd))
            {
                return GL_FALSE;
            }

            // pd(0, 0) -> the vertex point
            // pd(1, 0), pd(1, 1) ->  derivatives point

            // draw the u derivative
            glVertex3f(pd(0, 0).x(), pd(0, 0).y(), pd(0, 0).z());

            glVertex3f(pd(0, 0).x() + scale * pd(1, 0).x(),
                       pd(0, 0).y() + scale * pd(1, 0).y(),
                       pd(0, 0).z() + scale * pd(1, 0).z());

            // draw the v derivative
            glVertex3f(pd(0, 0).x(), pd(0, 0).y(), pd(0, 0).z());

            glVertex3f(pd(0, 0).x() + scale * pd(1, 1).x(),
                       pd(0, 0).y() + scale * pd(1, 1).y(),
                       pd(0, 0).z() + scale * pd(1, 1).z());

        }
    }
    glEnd();

    return GL_TRUE;
}

GLboolean TensorProductSurface3::UpdateVertexBufferObjectsOfData(GLenum usage_flag)
{
    GLuint data_count = _data.GetRowCount() * _data.GetColumnCount();
    if (!data_count)
        return GL_FALSE;

    if (usage_flag != GL_STREAM_DRAW  && usage_flag != GL_STREAM_READ  && usage_flag != GL_STREAM_COPY
            && usage_flag != GL_DYNAMIC_DRAW && usage_flag != GL_DYNAMIC_READ && usage_flag != GL_DYNAMIC_COPY
            && usage_flag != GL_STATIC_DRAW  && usage_flag != GL_STATIC_READ  && usage_flag != GL_STATIC_COPY)
        return GL_FALSE;

    DeleteVertexBufferObjectsOfData();

    glGenBuffers(1, &_vbo_data);
    if (!_vbo_data)
        return GL_FALSE;

    glBindBuffer(GL_ARRAY_BUFFER, _vbo_data);
    glBufferData(GL_ARRAY_BUFFER, data_count * 2 * 3 * sizeof(GLfloat), 0, usage_flag);

    GLfloat *coordinate = (GLfloat*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    if (!coordinate)
    {
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        DeleteVertexBufferObjectsOfData();
        return GL_FALSE;
    }

    // row
    for(GLuint r = 0; r < _data.GetRowCount(); r++)
    {
        // column
        for(GLuint c = 0; c < _data.GetColumnCount(); c++)
        {
            // Coordinate
            for(GLuint dc = 0; dc < 3; dc++)
            {
                *coordinate = (GLfloat) _data(r, c)[dc];
                ++coordinate;
            }
        }
    }

    // column
    for(GLuint c = 0; c < _data.GetColumnCount(); c++)
    {
        // row
        for(GLuint r = 0; r < _data.GetRowCount(); r++)
        {
            // Coordinate
            for(GLuint dc = 0; dc < 3; dc++)
            {
                *coordinate = (GLfloat) _data(r, c)[dc]; // c, r
                ++coordinate;
            }
        }
    }

    if (!glUnmapBuffer(GL_ARRAY_BUFFER))
    {
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        DeleteVertexBufferObjectsOfData();
        return GL_FALSE;
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    return GL_TRUE;
}

RowMatrix<GenericCurve3*>* TensorProductSurface3::GenerateUIsoparametricLines(GLuint iso_line_count,
                                                                              GLuint maximum_order_of_derivatives,
                                                                              GLuint div_point_count,
                                                                              GLenum usage_flag) const
{
    RowMatrix<GenericCurve3*>* result = new RowMatrix<GenericCurve3*>(iso_line_count);

    GLdouble v_div = (_v_max - _v_min) / iso_line_count;
    GLdouble u_div = (_u_max - _u_min) / div_point_count;

    for(GLuint i = 0; i < iso_line_count; i++)
    {
        (*result)[i] = new GenericCurve3(maximum_order_of_derivatives, div_point_count, usage_flag);

        if(!result)
        {
            for(GLuint t = 0; t < i; t++)
                delete (*result)[t];

            delete result;

            return nullptr;
        }

        GLdouble v = min(_v_min + i * v_div, _v_max);
        for(GLuint j = 0; j < div_point_count; j++)
        {
            GLdouble u = min(_u_min + j * u_div, _u_max);
            PartialDerivatives pd;

            if(!CalculatePartialDerivatives(maximum_order_of_derivatives, u, v, pd))
            {
                for(GLuint t = 0; t <= i; t++)
                    delete (*result)[t];

                delete result;

                return nullptr;
            }

            for(GLuint k = 0; k < maximum_order_of_derivatives; k++)
            {
                (*result)[i]->SetDerivative(k, j, pd(k, 0));
            }
        }
    }

    return result;
}

RowMatrix<GenericCurve3*>* TensorProductSurface3::GenerateVIsoparametricLines(GLuint iso_line_count,
                                                                              GLuint maximum_order_of_derivatives,
                                                                              GLuint div_point_count,
                                                                              GLenum usage_flag) const
{
    RowMatrix<GenericCurve3*>* result = new RowMatrix<GenericCurve3*>(iso_line_count);

    GLdouble v_div = (_v_max - _v_min) / div_point_count;
    GLdouble u_div = (_u_max - _u_min) / iso_line_count;

    for(GLuint i = 0; i < iso_line_count; i++)
    {
        (*result)[i] = new GenericCurve3(maximum_order_of_derivatives, div_point_count, usage_flag);

        if(!result)
        {
            for(GLuint t = 0; t < i; t++)
                delete (*result)[t];

            delete result;

            return nullptr;
        }

        GLdouble u = min(_u_min + i * u_div, _u_max);

        for(GLuint j = 0; j < div_point_count; j++)
        {
            GLdouble v = min(_v_min + j * v_div, _v_max);
            PartialDerivatives pd;

            if(!CalculatePartialDerivatives(maximum_order_of_derivatives, u, v, pd))
            {
                for(GLuint t = 0; t <= i; t++)
                    delete (*result)[t];

                delete result;

                return nullptr;
            }

            for(GLuint k = 0; k < maximum_order_of_derivatives; k++)
            {
                (*result)[i]->SetDerivative(k, j, pd(k, k)[0], pd(k, k)[1], pd(k, k)[2]);
            }
        }
    }

    return result;
}

GLdouble TensorProductSurface3::CalculateEnergy(ImageColorScheme type, const PartialDerivatives &pd) const
{
    GLdouble result = 0.0;

    GLdouble e1 = 0.0, f1 = 0.0, g1 = 0.0, e2 = 0.0, f2 = 0.0, g2 = 0.0;
    DCoordinate3 n0;
    GLdouble K, H;

    e1 = pd(1, 0) * pd(1, 0);
    f1 = pd(1, 0) * pd(1, 1);
    g1 = pd(1, 1) * pd(1, 1);

    n0 = pd(1, 0);
    n0 ^= pd(1, 1);

    GLdouble n = n0.length();

    n0 /= n;

    e2 = n0 * pd(2, 0);
    f2 = n0 * pd(2, 1);
    g2 = n0 * pd(2, 2);

    K = (e2 * g2 - f2 * f2) / (e1 * g1 - f1 * f1);
    H = (e2 * g1 - 2 * f1 * f2 + e1 * g2) / (e1 * g1 - f1 * f1);

    switch (type)
    {
    case DEFAULT_NULL_FRAGMENT:
        return 0.0;
        break;
    case NORMAL_LENGTH_FRAGMENT:
        return n;
        break;
    case GAUSSIAN_CURVATURE_FRAGMENT:
        result = K * n;
        break;
    case MEAN_CURVATURE_FRAGMENT:
        result = H * n;
        break;
    case WILLMORE_ENERGY_FRAGMENT:
        result = H * H * n;
        break;
    case LOG_WILLMORE_ENERGY_FRAGMENT:
        result = log (1 + H * H) * n;
        break;
    case UMBILIC_DEVIATION_ENERGY_FRAGMENT:
        result = 4 * (H * H - K) * n;
        break;
    case LOG_UMBILIC_DEVIATION_ENERGY_FRAGMENT:
        result = log (1 + 4 * (H * H - K)) * n;
        break;
    case TOTAL_CURVATURE_ENERGY_FRAGMENT:
        result = (1.5 * H * H - 0.5 * K) * n;
        break;
    case LOG_TOTAL_CURVATURE_ENERGY_FRAGMENT:
        result = log(1 + 1.5 * H * H - 0.5 * K) * n;
        break;
    }

    return result;
}

Color4 TensorProductSurface3::ColdToHotColormap(GLfloat value, GLfloat min_value, GLfloat max_value) const
{
    Color4 color(1.0, 1.0, 1.0);

    if (value < min_value)
    {
        value = min_value;
    }

    if (value > max_value)
    {
        value = max_value;
    }

    float dv = max_value - min_value;

    if (value < (min_value + 0.25f * dv))
    {
        color.r() = 0.0;
        color.g() = 4.0f * (value - min_value) / dv;
    }
    else
    {
        if (value < (min_value + 0.5f * dv))
        {
            color.r() = 0.0f;
            color.b() = 1.0f + 4.0f * (min_value + 0.25f * dv - value) / dv;
        }
        else
        {
            if (value < (min_value + 0.75f * dv))
            {
                color.r() = 4.0f * (value - min_value - 0.5f * dv) / dv;
                color.b() = 0.0f;
            }
            else
            {
                color.g() = 1.0f + 4.0f * (min_value + 0.75f * dv - value) / dv;
                color.b() = 0.0f;
            }
        }
    }

    return color;
}

GLboolean TensorProductSurface3::UpdateColorShemeOfImage(TriangulatedMesh3 &image, ImageColorScheme color_sheme) const
{
    GLdouble min_value = numeric_limits<GLdouble>::max();
    GLdouble max_value = -numeric_limits<GLdouble>::max();

    for (GLuint i_j = 0; i_j < image.VertexCount(); i_j++)
    {
        if (min_value > _fragments(color_sheme, i_j))
        {
            min_value = _fragments(color_sheme, i_j);
        }

        if (max_value < _fragments(color_sheme, i_j))
        {
            max_value = _fragments(color_sheme, i_j);
        }
    }

    for (GLuint i_j = 0; i_j < image.VertexCount(); i_j++)
    {
        image._color[i_j] = ColdToHotColormap(_fragments(color_sheme, i_j), min_value, max_value);
    }

    return GL_TRUE;
}

TensorProductSurface3::~TensorProductSurface3()
{
    DeleteVertexBufferObjectsOfData();
}
