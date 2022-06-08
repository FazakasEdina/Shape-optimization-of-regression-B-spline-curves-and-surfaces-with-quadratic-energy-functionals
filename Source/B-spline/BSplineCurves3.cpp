#include "../B-spline/BSplineCurves3.h"
#include "../Core/Constants.h"
#include "../Core/Exceptions.h"

#include <map>

using namespace std;
namespace cagd
{
// special constructor
BSplineCurve3::BSplineCurve3(KnotVector::Type type, GLuint k, GLuint n, GLdouble u_min, GLdouble u_max, GLenum data_usage_flag):
    LinearCombination3(u_min, u_max, n+1, data_usage_flag),
    _n(n)
{
    _kv = new (nothrow) KnotVector(type, k, n, u_min, u_max);
}

// copy constructor
BSplineCurve3::BSplineCurve3(const BSplineCurve3 &curve):
    LinearCombination3(curve),
    _n(curve._n),
    _kv(curve._kv ? new KnotVector(*curve._kv) : nullptr)
{
}

// assignment operator
BSplineCurve3& BSplineCurve3::operator =(const BSplineCurve3 &rhs)
{
    if (this != &rhs)
    {
        if (_kv)
        {
            delete _kv;
            _kv = nullptr;
        }

        LinearCombination3::operator=(rhs);
        this->_n = rhs._n;
        cout << this->_n;

        this->_kv = rhs._kv ? new KnotVector(*rhs._kv) : nullptr; // deep copy policy
    }

    return *this;
}

GLboolean BSplineCurve3::BlendingFunctionValues(GLdouble u, RowMatrix<GLdouble>& values) const
{
    values.ResizeColumns(_kv->GetControlPointCount());

    TriangularMatrix<GLdouble> N;
    GLuint i;
    if (!_kv->EvaluateNonZeroBSplineFunctions(u, i, N))
    {
        return GL_FALSE;
    }

    GLuint k = _kv->GetOrder();

    for (GLuint j = 0; j < i - k + 1; j++)
    {
        values[j] = 0.0;
    }

    GLuint offset = i - k + 1;
    for (GLuint j = offset; j <= i; j++)
    {
        values[j] = N(k - 1, j - offset);
    }

    for (GLuint j = i + 1; j < values.GetColumnCount(); j++)
    {
        values[j] = 0.0;
    }

    return GL_TRUE;
}

GLboolean BSplineCurve3::BlendingFunctionValuesForPeriodicRegressionCurve(GLdouble u, RowMatrix<GLdouble>& values) const
{
    GLuint k = _kv->GetOrder();
    GLuint size = _kv->GetControlPointCount() - k + 1;
    values.ResizeColumns(size);

    TriangularMatrix<GLdouble> N;
    GLuint i;
    if (!_kv->EvaluateNonZeroBSplineFunctions(u, i, N))
    {
        return GL_FALSE;
    }

    GLuint offset = i - k + 1;
    for (GLuint j = 0; j < offset; j++)
    {
        values[j] = 0.0;
    }

    for (GLuint j = offset; j <= i; j++)
    {
        values[j % size] = N(k - 1, j - offset);
    }

    for (GLuint j = i + 1; j < values.GetColumnCount(); j++)
    {
        values[j] = 0.0;
    }

    return GL_TRUE;
}
GLboolean BSplineCurve3::CalculateDerivatives(GLuint max_order_of_derivatives, GLdouble u, Derivatives& d) const
{
    if (u < _u_min || u > _u_max || !_kv)
    {
        d.ResizeRows(0);
        return GL_FALSE;
    }

    GLuint i;
    TriangularMatrix<GLdouble> N;

    if (!_kv->EvaluateNonZeroBSplineFunctions(u, i, N))
    {
        d.ResizeRows(0);
        return GL_FALSE;
    }

    d.ResizeRows(max_order_of_derivatives + 1);
    d.LoadNullVectors();

    GLuint k                    =   _kv->GetOrder();
    GLuint offset               =   i - k + 1;
    ColumnMatrix<DCoordinate3>      clone(k);

    for (GLuint j = offset; j <= i; ++j)
    {
        GLuint index = j - offset;
        GLuint cp_index = j % (_n + 1);

        d[0] += _data[cp_index] * N(k - 1, index);

        clone[index] = _data[cp_index];
    }

    for (GLuint r = 1; r <= min(max_order_of_derivatives, k - 1); r++)
    {
        ColumnMatrix<DCoordinate3> dcp(k - r);

        GLuint roffset = offset + r;

        for (GLuint j = roffset; j <= i; j++)
        {
            GLdouble denominator = (*_kv)[j + k - r] - (*_kv)[j];

            GLuint rindex = j - roffset;

            if (denominator)
            {
                dcp[rindex]  = clone[rindex + 1];
                dcp[rindex] -= clone[rindex];
                dcp[rindex] /= denominator;
                d[r] += dcp[rindex] * N(k - 1 - r, rindex);
            }
        }
        clone = dcp;
    }

    GLdouble factor = 1.0;
    for (GLuint r = 1; r < d.GetRowCount(); r++)
    {
        factor *= (k - r);
        d[r] *= factor;
    }

    return GL_TRUE;
}

// generates the arcs of the B-spline curve
RowMatrix<GenericCurve3*>* BSplineCurve3::GenerateImageOfArcs(GLuint max_order_of_derivatives, GLuint div_point_count, GLenum usage_flag)
{
    GLuint k = _kv->GetOrder();
    GLuint cp_count = _kv->GetControlPointCount();
    GLuint size = cp_count - k + 1;

    RowMatrix<GenericCurve3*>* result = new (nothrow) RowMatrix<GenericCurve3*>(size);

    if (!result)
    {
        return nullptr;
    }

    result->ResizeColumns(size);

    GLuint offset = k - 1;
    for (GLuint i = offset; i < cp_count; i++)
    {
        (*result)(i - offset) = GenerateImageInAGivenInterval(max_order_of_derivatives, div_point_count,
                                                              (*_kv)[i], (*_kv)[i + 1], usage_flag);

        if (!(*result)(i - offset))
        {
            for (GLuint a = 0; a < i - offset; a++)
            {
                delete (*result)[a];
                (*result)[a] = nullptr;
            }
            delete result;
            result = nullptr;
            return nullptr;
        }
    }

    return result;
}

GenericCurve3* BSplineCurve3::GenerateImageOfAnArc(GLuint index, GLuint max_order_of_derivatives, GLuint div_point_count, GLenum usage_flag) const
{
    GLuint k = _kv->GetOrder();
    GLuint cp_count = _kv->GetControlPointCount();

    if (index < k - 1 || index > cp_count - 1)
    {
        return nullptr;
    }

    GenericCurve3* result = GenerateImageInAGivenInterval(max_order_of_derivatives, div_point_count,
                                                          (*_kv)[index], (*_kv)[index + 1], usage_flag);

    if(!result)
    {
        return nullptr;
    }
    return result;
}

KnotVector* BSplineCurve3::GetKnotVector() const
{
    return _kv;
}

// frees the dynamically allocated knot vector
BSplineCurve3::~BSplineCurve3()
{
    if (_kv)
    {
        delete _kv;
        _kv = nullptr;
    }
}
}
