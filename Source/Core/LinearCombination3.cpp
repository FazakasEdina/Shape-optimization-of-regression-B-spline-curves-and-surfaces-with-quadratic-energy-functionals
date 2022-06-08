#include "LinearCombination3.h"
#include "RealSquareMatrices.h"
#include "Constants.h"

using namespace cagd;
using namespace std;

// special/default constructor
LinearCombination3::Derivatives::Derivatives(GLuint maximum_order_of_derivatives): ColumnMatrix<DCoordinate3>(maximum_order_of_derivatives + 1)
{
}

// copy constructor
LinearCombination3::Derivatives::Derivatives(const LinearCombination3::Derivatives& d): ColumnMatrix<DCoordinate3>(d)
{
}

// assignment operator
LinearCombination3::Derivatives& LinearCombination3::Derivatives::operator =(const LinearCombination3::Derivatives& rhs)
{
    if (this != &rhs)
    {
        ColumnMatrix<DCoordinate3>::operator =(rhs);
    }
    return *this;
}

// set every derivative to null vector
GLvoid LinearCombination3::Derivatives::LoadNullVectors()
{
    for (GLuint i = 0; i < _data.size(); ++i)
    {
        for (GLuint j = 0; j < 3; ++j)
            _data[i][0][j] = 0.0;
    }
}

// special constructor
LinearCombination3::LinearCombination3(GLdouble u_min, GLdouble u_max, GLuint data_count, GLenum data_usage_flag):
    _vbo_data(0),
    _data_usage_flag(data_usage_flag),
    _u_min(u_min), _u_max(u_max),
    _data(data_count)
{
}

// copy constructor
LinearCombination3::LinearCombination3(const LinearCombination3 &lc):
    _vbo_data(0),
    _data_usage_flag(lc._data_usage_flag),
    _u_min(lc._u_min), _u_max(lc._u_max),
    _data(lc._data)
{
    if (lc._vbo_data)
        UpdateVertexBufferObjectsOfData(_data_usage_flag);
}

// assignment operator
LinearCombination3& LinearCombination3::operator =(const LinearCombination3& rhs)
{
    if (this != &rhs)
    {
        DeleteVertexBufferObjectsOfData();

        _data_usage_flag = rhs._data_usage_flag;
        _u_min = rhs._u_min;
        _u_max = rhs._u_max;
        _data = rhs._data;

        if (rhs._vbo_data)
            UpdateVertexBufferObjectsOfData(_data_usage_flag);
    }

    return *this;
}

// vbo handling methods
GLvoid LinearCombination3::DeleteVertexBufferObjectsOfData()
{
    if (_vbo_data)
    {
        glDeleteBuffers(1, &_vbo_data);
        _vbo_data = 0;
    }
}

GLboolean LinearCombination3::RenderData(GLenum render_mode) const
{
    if (!_vbo_data)
        return GL_FALSE;

    if (render_mode != GL_LINE_STRIP && render_mode != GL_LINE_LOOP && render_mode != GL_POINTS)
        return GL_FALSE;

    glEnableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, _vbo_data);
    glVertexPointer(3, GL_FLOAT, 0, (const GLvoid*)0);
    glDrawArrays(render_mode, 0, _data.GetRowCount());
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDisableClientState(GL_VERTEX_ARRAY);

    return GL_TRUE;
}

GLboolean LinearCombination3::UpdateVertexBufferObjectsOfData(GLenum usage_flag)
{
    GLuint data_count = _data.GetRowCount();
    if (!data_count)
        return GL_FALSE;

    if (usage_flag != GL_STREAM_DRAW  && usage_flag != GL_STREAM_READ  && usage_flag != GL_STREAM_COPY
            && usage_flag != GL_DYNAMIC_DRAW && usage_flag != GL_DYNAMIC_READ && usage_flag != GL_DYNAMIC_COPY
            && usage_flag != GL_STATIC_DRAW  && usage_flag != GL_STATIC_READ  && usage_flag != GL_STATIC_COPY)
        return GL_FALSE;

    _data_usage_flag = usage_flag;

    DeleteVertexBufferObjectsOfData();

    glGenBuffers(1, &_vbo_data);
    if (!_vbo_data)
        return GL_FALSE;

    glBindBuffer(GL_ARRAY_BUFFER, _vbo_data);
    glBufferData(GL_ARRAY_BUFFER, data_count * 3 * sizeof(GLfloat), 0, _data_usage_flag);

    GLfloat *coordinate = (GLfloat*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    if (!coordinate)
    {
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        DeleteVertexBufferObjectsOfData();
        return GL_FALSE;
    }

    for (GLuint i = 0; i < data_count; ++i)
    {
        for (GLuint j = 0; j < 3; ++j)
        {
            *coordinate = (GLfloat)_data[i][j];
            ++coordinate;
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

// get data by value
DCoordinate3 LinearCombination3::operator [](GLuint index) const
{
    return _data[index];
}

// get data by reference
DCoordinate3& LinearCombination3::operator [](GLuint index)
{
    return _data[index];
}

// assure interpolation
GLboolean LinearCombination3::UpdateDataForInterpolation(const ColumnMatrix<GLdouble>& knot_vector, const ColumnMatrix<DCoordinate3>& data_points_to_interpolate)
{
    GLuint data_count = _data.GetRowCount();

    if (data_count != knot_vector.GetRowCount() ||
            data_count != data_points_to_interpolate.GetRowCount())
        return GL_FALSE;

    RealSquareMatrix collocation_matrix(data_count);

    RowMatrix<GLdouble> current_blending_function_values(data_count);
    for (GLuint r = 0; r < knot_vector.GetRowCount(); ++r)
    {
        if (!BlendingFunctionValues(knot_vector(r), current_blending_function_values))
            return GL_FALSE;
        else
            collocation_matrix.SetRow(r, current_blending_function_values);
    }

    return collocation_matrix.SolveLinearSystem(data_points_to_interpolate, _data);
}

// set/get definition domain
GLvoid LinearCombination3::SetDefinitionDomain(GLdouble u_min, GLdouble u_max)
{
    _u_min = u_min;
    _u_max = u_max;
}

GLvoid LinearCombination3::GetDefinitionDomain(GLdouble& u_min, GLdouble& u_max) const
{
    u_min = _u_min;
    u_max = _u_max;
}

// generate image/arc
GenericCurve3* LinearCombination3::GenerateImage(GLuint max_order_of_derivatives, GLuint div_point_count, GLenum usage_flag) const
{
    GenericCurve3* result = new (nothrow) GenericCurve3(max_order_of_derivatives, div_point_count, usage_flag);

    if(!result)
    {
        return nullptr;
    }

    GLdouble u_step = (_u_max - EPS - _u_min) / (div_point_count - 1);

    GLboolean aborted = GL_FALSE;

#pragma omp parallel for
    for (GLint i = 0; i < static_cast<GLint>(div_point_count); i++)
    {
#pragma omp flush(aborted)
        if (!aborted)
        {
            GLdouble u = min(_u_min + i * u_step, _u_max - EPS);
            Derivatives d;

            if(!CalculateDerivatives(max_order_of_derivatives, u, d) || !result->_derivative.SetColumn(i, d))
            {
                aborted = GL_TRUE;
#pragma omp flush(aborted)
            }
        }
    }
    if (aborted)
    {
        delete result;
        result = nullptr;
    }

    return result;
}

// Generate image in a given interval
GenericCurve3* LinearCombination3::GenerateImageInAGivenInterval(GLuint max_order_of_derivatives, GLuint div_point_count,
                                                                 GLdouble u_min, GLdouble u_max, GLenum usage_flag) const
{
    GenericCurve3* result = new (nothrow) GenericCurve3(max_order_of_derivatives, div_point_count, usage_flag);

    if(!result)
    {
        return nullptr;
    }

    GLdouble u_step = (u_max - EPS - u_min) / (div_point_count - 1);

    GLboolean aborted = GL_FALSE;

#pragma omp parallel for
    for (GLint i = 0; i < static_cast<GLint> (div_point_count); i++)
    {
#pragma omp flush(aborted)
        if (!aborted)
        {
            GLdouble u = min(u_min + i * u_step, u_max - EPS);
            Derivatives d;

            if(!CalculateDerivatives(max_order_of_derivatives, u, d) || !result->_derivative.SetColumn(i, d))
            {
                aborted = GL_TRUE;
#pragma omp flush(aborted)
            }
        }
    }
    if (aborted)
    {
        delete result;
        result = nullptr;
    }

    return result;
}

RowMatrix<GLdouble> LinearCombination3::LocalEnergies(GLdouble u) const
{
    Derivatives d;
    CalculateDerivatives(3, u, d);

    RowMatrix<GLdouble> localEnergies(5);
    localEnergies[0] = (d[1]^d[2]).length() / pow(d[1].length(), (int)3); // curvature
    localEnergies[1] = d(1).length(); // lenght
    localEnergies[2] = d(1) * d(1); // d(1).length() * d(1).length(); // kinetic energy
    localEnergies[3] = d(2) * d(2);
    localEnergies[4] = d(3) * d(3);

    return localEnergies;
}

RowMatrix<GLdouble> LinearCombination3::TotalEnergies(GLuint m) const
{
    if (m % 2)
    {
        m += 1;
    }

    GLuint n = m / 2;

    GLdouble step = (_u_max-_u_min) / m;

    RowMatrix<GLdouble> even(5);
    even[0] = 0.0; even[1] = 0.0; even[2] = 0.0;  even[3] = 0.0; even[4] = 0.0;
    for (GLuint j = 1; j <= n-1; j++)
    {
        GLdouble t = min(_u_min + 2 * j * step, _u_max);
        RowMatrix<GLdouble> locals = LocalEnergies(t);
        even[0] += locals[0];
        even[1] += locals[1];
        even[2] += locals[2];
        even[3] += locals[3];
        even[4] += locals[4];
    }
    even[0] *= 2.0;
    even[1] *= 2.0;
    even[2] *= 2.0;
    even[3] *= 2.0;
    even[4] *= 2.0;

    RowMatrix<GLdouble> odd(5);
    odd[0] = 0.0; odd[1] = 0.0; odd[2] = 0.0; odd[3] = 0.0; odd[4] = 0.0;
    for (GLuint j = 1; j <= n; j++)
    {
        GLdouble t = min(_u_min + (2 * j - 1) * step, _u_max);
        RowMatrix<GLdouble> locals = LocalEnergies(t);
        odd[0] += locals[0];
        odd[1] += locals[1];
        odd[2] += locals[2];
        odd[3] += locals[3];
        odd[4] += locals[4];
    }
    odd[0] *= 4.0;
    odd[1] *= 4.0;
    odd[2] *= 4.0;
    odd[3] *= 4.0;
    odd[4] *= 4.0;

    RowMatrix<GLdouble> integral(5);
    integral[0] = 0.0; integral[1] = 0.0; integral[2] = 0.0;  integral[3] = 0.0; integral[4] = 0.0;

    // a
    RowMatrix<GLdouble> locals = LocalEnergies(_u_min);
    integral[0] += locals[0];
    integral[1] += locals[1];
    integral[2] += locals[2];
    integral[3] += locals[3];
    integral[4] += locals[4];

    // at inner subdivision points
    integral[0] += even[0];
    integral[1] += even[1];
    integral[2] += even[2];
    integral[3] += even[3];
    integral[4] += even[4];

    integral[0] += odd[0];
    integral[1] += odd[1];
    integral[2] += odd[2];
    integral[3] += odd[3];
    integral[4] += odd[4];

    // b
    locals = LocalEnergies(_u_max);
    integral[0] += locals[0];
    integral[1] += locals[1];
    integral[2] += locals[2];
    integral[3] += locals[3];
    integral[4] += locals[4];


    // final scaling
    integral[0] *= step / 3.0;
    integral[1] *= step / 3.0;
    integral[2] *= step / 3.0;
    integral[3] *= step / 3.0;
    integral[4] *= step / 3.0;

//    cout << "Masodrendu mozgasi energia: " << integral[3] << endl;
//    cout << "Harmadrendu mozgasi energia: " << integral[4] << endl;

    return integral;
}

GLboolean LinearCombination3::RenderCurvatureComb(GLuint div_point_count, GLdouble scale)
{
    GLdouble step = (_u_max - _u_min) / div_point_count;
    for(GLuint i = 0; i < div_point_count; i++)
    {
        GLdouble u = _u_min + step * i;
        Derivatives d;
        CalculateDerivatives(2, u, d);
        GLdouble g = (d[1]^d[2]).length() / pow(d[1].length(), (int)3);
        DCoordinate3 c = d[0];
        DCoordinate3 n = - (d[2] / pow(d[1].length(), 2) / (d[2] / pow(d[1].length(), 2)).length());

        glBegin(GL_LINES);
        glVertex3f(c.x(), c.y(), c.z());
        glVertex3f(c.x() + scale * g * n.x(), c.y() + scale * g * n.y(), c.z() + scale * g * n.z());
        glEnd();
    }

    return GL_TRUE;
}

// destructor
LinearCombination3::~LinearCombination3()
{
    DeleteVertexBufferObjectsOfData();
}

