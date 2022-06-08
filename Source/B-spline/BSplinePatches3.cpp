#include "../B-spline/BSplinePatches3.h"
#include "../Core/Constants.h"

using namespace std;
namespace cagd
{
// special constructor
BSplinePatch3::BSplinePatch3(KnotVector::Type u_type, KnotVector::Type v_type,
                             GLuint u_k, GLuint v_k,
                             GLuint u_n, GLuint v_n,
                             GLdouble u_min, GLdouble u_max,
                             GLdouble v_min, GLdouble v_max) :
    TensorProductSurface3(u_min, u_max, v_min, v_max, u_n + 1, v_n + 1,
                          u_type == KnotVector::PERIODIC ? GL_TRUE : GL_FALSE,
                          v_type == KnotVector::PERIODIC ? GL_TRUE : GL_FALSE),
    _v_n(v_n), _u_n(u_n)
{
    _u_kv = new (nothrow) KnotVector(u_type, u_k, u_n, u_min, u_max);
    _v_kv = new (nothrow) KnotVector(v_type, v_k, v_n, v_min, v_max);
}

// copy constructor
BSplinePatch3::BSplinePatch3(const BSplinePatch3 &patch) :
    TensorProductSurface3(patch),
    _v_n(patch._v_n),
    _u_n(patch._u_n),
    _v_kv(patch._v_kv ? new KnotVector(*patch._v_kv) : nullptr),
    _u_kv(patch._u_kv ? new KnotVector(*patch._u_kv) : nullptr)
{
}

// assignment operator
BSplinePatch3& BSplinePatch3::operator =(const BSplinePatch3 &rhs)
{
    if (this != &rhs)
    {
        if (_u_kv)
        {
            delete _u_kv;
            _u_kv = nullptr;
        }

        if (_v_kv)
        {
            delete _v_kv;
            _v_kv = nullptr;
        }

        TensorProductSurface3::operator=(rhs);
        this-> _u_n = rhs._u_n;
        this-> _v_n = rhs._v_n;

        this->_u_kv = rhs._u_kv ? new KnotVector(*rhs._u_kv) : nullptr;
        this->_v_kv = rhs._v_kv ? new KnotVector(*rhs._v_kv) : nullptr;
    }

    return *this;
}

GLboolean BSplinePatch3::UBlendingFunctionValues(GLdouble u, RowMatrix<GLdouble> &blending_values) const
{
    blending_values.ResizeColumns(_u_kv->GetControlPointCount());

    TriangularMatrix<GLdouble> N;
    GLuint i;
    if (!_u_kv->EvaluateNonZeroBSplineFunctions(u, i, N))
    {
        return GL_FALSE;
    }

    GLuint k = _u_kv->GetOrder();
    GLuint offset = i - k + 1;

    for (GLuint j = 0; j < offset; j++)
    {
        blending_values[j] = 0.0;
    }

    for (GLuint j = offset; j <= i; j++)
    {
        blending_values[j] = N(k - 1, j - offset);
    }

    for (GLuint j = i + 1; j < blending_values.GetColumnCount(); j++)
    {
        blending_values[j] = 0.0;
    }

    return GL_TRUE;
}

GLboolean BSplinePatch3::VBlendingFunctionValues(GLdouble v, RowMatrix<GLdouble> &blending_values) const
{
    blending_values.ResizeColumns(_v_kv->GetControlPointCount());

    TriangularMatrix<GLdouble> N;
    GLuint i;
    if (!_v_kv->EvaluateNonZeroBSplineFunctions(v, i, N))
    {
        return GL_FALSE;
    }

    GLuint k = _v_kv->GetOrder();

    GLuint offset = i - k + 1;
    for (GLuint j = 0; j < offset; j++)
    {
        blending_values[j] = 0.0;
    }
    for (GLuint j = offset; j <= i; j++)
    {
        blending_values[j] = N(k - 1, j - offset);
    }

    for (GLuint j = i + 1; j < blending_values.GetColumnCount(); j++)
    {
        blending_values[j] = 0.0;
    }

    return GL_TRUE;
}

GLboolean BSplinePatch3::CalculatePartialDerivatives(
        GLuint maximum_order_of_partial_derivatives,
        GLdouble u, GLdouble v, PartialDerivatives &pd) const
{
    Matrix<GLdouble> u_dN, v_dN;
    GLuint i, j;
    if(!_u_kv->FindSpan(u, i) || !_v_kv->FindSpan(v, j))
    {
        pd.ResizeRows(0);
        return GL_FALSE;
    }
    if (!_u_kv->ZerothAndHigherOrderDerivative(maximum_order_of_partial_derivatives, u, u_dN) ||
            !_v_kv->ZerothAndHigherOrderDerivative(maximum_order_of_partial_derivatives, v, v_dN))
    {
        pd.ResizeRows(0);
        return GL_FALSE;
    }

    pd.ResizeRows(maximum_order_of_partial_derivatives + 1);
    pd.LoadNullVectors();

    GLuint k = _u_kv->GetOrder();
    GLuint l = _v_kv->GetOrder();
    GLdouble offset_u = i - k + 1;
    GLdouble offset_v = j - l + 1;

    for (GLuint ro = 0; ro <= maximum_order_of_partial_derivatives; ro++)
    {
        for (GLuint r = 0; r <= ro; r++)
        {
            for (GLint p = static_cast<GLint> (offset_u); p <= static_cast<GLint> (i); p++)
            {
                DCoordinate3 aux;
                for (GLuint q = offset_v; q <= j; q++)
                {
                    aux += _data(p % (_u_n + 1), q % (_v_n + 1)) * v_dN(r, q);
                }
                pd(ro, r) += aux * u_dN(ro - r, p);
            }
        }
    }

    return GL_TRUE;
}

GLboolean BSplinePatch3::UBlendingFunctionValuesForPeriodicRegressionSurface(GLdouble u, RowMatrix<GLdouble>& blending_values) const
{
    GLuint k = _u_kv->GetOrder();
    GLuint size = _u_kv->GetControlPointCount() - k + 1;
    blending_values.ResizeColumns(size);

    TriangularMatrix<GLdouble> N;
    GLuint i;
    if (!_u_kv->EvaluateNonZeroBSplineFunctions(u, i, N))
    {
        return GL_FALSE;
    }

    GLuint offset = i - k + 1;
    for (GLuint j = 0; j < offset; j++)
    {
        blending_values[j] = 0.0;
    }

    for (GLuint j = offset; j <= i; j++)
    {
        blending_values[j % size] = N(k - 1, j - offset);
    }

    for (GLuint j = i + 1; j < blending_values.GetColumnCount(); j++)
    {
        blending_values[j] = 0.0;
    }

    return GL_TRUE;
}

GLboolean BSplinePatch3::VBlendingFunctionValuesForPeriodicRegressionSurface(GLdouble v, RowMatrix<GLdouble>& blending_values) const
{
    GLuint k = _v_kv->GetOrder();
    GLuint size = _v_kv->GetControlPointCount() - k + 1;
    blending_values.ResizeColumns(size);

    TriangularMatrix<GLdouble> N;
    GLuint i;
    if (!_v_kv->EvaluateNonZeroBSplineFunctions(v, i, N))
    {
        return GL_FALSE;
    }

    GLuint offset = i - k + 1;
    for (GLuint j = 0; j < offset; j++)
    {
        blending_values[j] = 0.0;
    }

    for (GLuint j = offset; j <= i; j++)
    {
        blending_values[j % size] = N(k - 1, j - offset);
    }

    for (GLuint j = i + 1; j < blending_values.GetColumnCount(); j++)
    {
        blending_values[j] = 0.0;
    }

    return GL_TRUE;
}

Matrix<TriangulatedMesh3*>* BSplinePatch3::GenerateImageOfPatches(GLuint u_div_point_count, GLuint v_div_point_count,
                                                                  ImageColorScheme color_sheme, GLenum usage_flag)
{
    if (u_div_point_count <= 1 || v_div_point_count <= 1)
        return GL_FALSE;

    GLuint u_k = _u_kv->GetOrder();
    GLuint v_k = _v_kv->GetOrder();

    GLuint u_cp_count = _u_kv->GetControlPointCount();
    GLuint v_cp_count = _v_kv->GetControlPointCount();

    GLuint u_size = u_cp_count - u_k + 1;
    GLuint v_size = v_cp_count - v_k + 1;

    Matrix<TriangulatedMesh3*>* result = new (nothrow) Matrix<TriangulatedMesh3*>(u_size, v_size);

    if (!result)
    {
        return nullptr;
    }

    GLuint u_offset = u_k - 1;
    GLuint v_offset = v_k - 1;

    for (GLuint row_index = u_offset; row_index < u_cp_count; row_index++)
    {
        for (GLuint column_index = v_offset; column_index < v_cp_count; column_index++)
        {
            (*result)(row_index - u_offset, column_index - v_offset) =
                    GenerateImageInAGivenInterval(u_div_point_count, v_div_point_count,
                                                  (*_u_kv)[row_index], (*_u_kv)[row_index+1],
                    (*_v_kv)[column_index], (*_v_kv)[column_index+1],
                    color_sheme, usage_flag);
            if (!(*result)(row_index - u_offset, column_index - v_offset))
            {
                for (GLuint c = 0; c < row_index - u_offset; c++)
                {
                    for (GLuint d = 0; d < column_index - v_offset; d++)
                    {
                        delete (*result)(c, d);
                        (*result)(c, d) = nullptr;
                    }
                }
                delete result;
                result = nullptr;
                return nullptr;
            }
        }
    }


    return result;
}

TriangulatedMesh3* BSplinePatch3::GenerateImageOfAnPatch(GLuint row_index, GLuint column_index,
                                                         GLuint u_div_point_count, GLuint v_div_point_count,
                                                         ImageColorScheme color_sheme, GLenum usage_flag) const
{
    if (u_div_point_count <= 1 || v_div_point_count <= 1)
        return GL_FALSE;

    GLuint u_k = _u_kv->GetOrder();
    GLuint v_k = _v_kv->GetOrder();

    GLuint u_cp_count = _u_kv->GetControlPointCount();
    GLuint v_cp_count = _v_kv->GetControlPointCount();

    if (row_index < u_k - 1 || row_index > u_cp_count - 1)
    {
        return nullptr;
    }

    if (column_index < v_k - 1 || column_index > v_cp_count - 1)
    {
        return nullptr;
    }

    TriangulatedMesh3* result = GenerateImageInAGivenInterval(u_div_point_count, v_div_point_count,
                                                              (*_u_kv)[row_index], (*_u_kv)[row_index+1],
            (*_v_kv)[column_index], (*_v_kv)[column_index+1],
            color_sheme, usage_flag);
    if(!result)
    {
        return nullptr;
    }

    return result;
}

KnotVector* BSplinePatch3::GetKnotVectorU() const
{
    return _u_kv;
}

KnotVector* BSplinePatch3::GetKnotVectorV() const
{
    return _v_kv;
}

BSplinePatch3::~BSplinePatch3()
{
    if (_u_kv)
    {
        delete _u_kv;
        _u_kv = nullptr;
    }

    if (_v_kv)
    {
        delete _v_kv;
        _v_kv = nullptr;
    }
}
}
