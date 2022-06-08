#include "KnotVectors.h"

using namespace std;
using namespace cagd;

//--------------------------------------
// implementation of class KnotVector
//--------------------------------------

// special constructor
KnotVector::KnotVector(Type type, GLuint k, GLuint n, GLdouble u_min, GLdouble u_max)
{
    _order = k;

    // [u_{k-1}, u_{n+1}] (unclamped/clamped) or [u_{k-1}, u_{n+k}] (periodic)
    _u_min = u_min;
    _u_max = u_max;

    // generate knots
    GLdouble step;
    GLuint r, offset;

    _type = type;
    switch(type)
    {
    // monotone increasing knot values:
    //   u_{0}, u_{1}, ..., u_{n+k}    (unclamped/clamped)
    //   u_{0}, u_{1}, ..., u_{n+2k-1} (periodic)
    case CLAMPED:
        _control_point_count = n + 1;
        _knots.ResizeColumns(n + k + 1);
        offset = k - 1;

        step = (u_max - u_min) / (n - k + 2);
        for (r = 0; r <= k - 1; r++)
            _knots[r] = u_min;

        for (r = k; r <= n; r++)
            _knots[r] = u_min + (static_cast<GLdouble>(r) - static_cast<GLdouble>(offset)) * step;

        for (r = n + 1; r < n + k + 1; r++)
            _knots[r] = u_max;

        break;
    case UNCLAMPED:
        _control_point_count = n + 1;
        _knots.ResizeColumns(n + k + 1);
        offset = k - 1;

        step = (u_max - u_min) / (n - k + 2);
        for (r = 0; r < n + k + 1; r++)
            _knots[r] = u_min + (static_cast<GLdouble>(r) - static_cast<GLdouble>(offset)) * step;

        break;
    case PERIODIC:
        _control_point_count = n + k;
        _knots.ResizeColumns(n + 2 * k);
        offset = k - 1;

        step = (u_max - u_min) / (n + 1);
        for (r = 0; r < n + 2 * k; r++)
        {
            _knots[r] = u_min + (static_cast<GLdouble>(r) - static_cast<GLdouble>(offset)) * step;
        }

        break;
    }
}

// get knot value by value
GLdouble KnotVector::operator [](GLuint index) const
{
    return _knots[index];
}

// get knot value by reference
GLdouble& KnotVector::operator [](GLuint index)
{
    return _knots[index];
}

// determines an index stored in variable i for which u is in [u_{i}, u_{i + 1})
GLboolean KnotVector::FindSpan(GLdouble u, GLuint& i) const
{
    if (u < _u_min || u > _u_max || _control_point_count <= 1 || _knots.GetColumnCount() < _control_point_count + _order)
    {
        return GL_FALSE;
    }

    if (u == _knots[_control_point_count])
    {
        i = _control_point_count - 1;
        return GL_TRUE;
    }

    GLuint left  = _order - 1;
    GLuint right = _control_point_count;

    i = (left + right) / 2;

    while (!(_knots[i] <= u && u < _knots[i + 1]))
    {
        if (u < _knots[i])
            right = i;
        else
            left = i;

        i = (left + right) / 2;
    }

    return GL_TRUE;

}

// evaluates non-vanishing normalized B-spline function values in a lower triangular matrix of the form
// N[row - 1][column] = N_{span - row + column + 1}^{row}, row = 1, ..., order, column = 0, 1, ..., row - 1
GLboolean KnotVector::EvaluateNonZeroBSplineFunctions(GLdouble u, GLuint& i, TriangularMatrix<GLdouble>& N) const
{
    if (!FindSpan(u, i))
    {
        return GL_FALSE;
    }

    N.ResizeRows(_order);

    N(0, 0) = 1.0; // N_{i}^{1}(u)

    GLdouble left, right;
    // k >= 2
    // N_{i}^{k}(u) = \frac{u - u_{i}}{u_{i+k-1} - u_{i}}N_{i}^{k-1}(u) + \frac{u_{i+k} - u}{u_{i+k} - u_{i+1}}N_{i+1}^{k-1}(u)
    // N_{j}^{k}(u), j = i-k+1, i-k+2, ..., i
    for (GLuint k = 2; k <= _order; ++k)
    {
        // j = i - k + 1
        right = _knots[i + 1] - _knots[i - k + 2];
        N(k - 1, 0) = (right > 0.0) ? N(k - 2, 0) * (_knots[i + 1] - u) / right : 0.0;

        // j = i
        left = _knots[i + k - 1] - _knots[i];
        N(k - 1, k - 1) = (left > 0.0) ? N(k - 2, k - 2) * (u - _knots[i]) / left : 0.0;

        // j = i - k + 2, ..., i - 1
        GLuint offset = i - k + 1;

        for (GLuint j = i - k + 2; j <= i - 1; j++)
        {
            left  = _knots[j + k - 1] - _knots[j];
            right = _knots[j + k] - _knots[j + 1];

            N(k - 1, j - offset) =
                    ((left  > 0.0) ? N(k - 2, j - offset - 1) * (u - _knots[j]) / left : 0.0) +
                    ((right > 0.0) ? N(k - 2, j - offset) * (_knots[j + k] - u) / right : 0.0);
        }
    }

    return GL_TRUE;
}

// getters
KnotVector::Type KnotVector::GetType() const
{
    return _type;
}

GLuint KnotVector::GetOrder() const
{
    return _order;
}

GLuint KnotVector::GetControlPointCount() const
{
    return _control_point_count;
}

GLdouble KnotVector::GetMin() const
{
    return _u_min;
}

GLdouble KnotVector::GetMax() const
{
    return _u_max;
}

GLuint KnotVector::GetN() const
{
    return _type == KnotVector::PERIODIC ? _control_point_count - _order : _control_point_count - 1;
}

std::ostream& cagd::operator << (std::ostream& lhs, const KnotVector& rhs)
{
    lhs << rhs._knots;
    return lhs;
}

GLboolean KnotVector::ZerothAndHigherOrderDerivative(GLuint maximum_order_of_derivatives, GLdouble u, Matrix<GLdouble> &dN) const
{
    GLuint k = _order;              // order
    GLuint i;                       // span: [u_{i}, u_{i+1})
    TriangularMatrix<GLdouble> N;   // non-vanishing normalized B-spline basis functions of order 1,...,k

    if (!EvaluateNonZeroBSplineFunctions(u, i, N))
    {
        dN.ResizeColumns(0);
        dN.ResizeRows(0);
        return GL_FALSE;
    }

    // resizing the output matrix dN
    dN.ResizeRows(maximum_order_of_derivatives + 1);
    dN.ResizeColumns(_control_point_count);

    // storing initial zeros
    for(GLuint r = 0; r <= maximum_order_of_derivatives; r++)
    {
        for(GLuint j = 0; j < _control_point_count; j++)
        {
            dN(r, j) = 0.0;
        }
    }
    // storing the zeroth order derivatives of the non-vanishing B-spline basis functions
    GLuint offset_0 = i - k + 1;
    for (GLuint j = offset_0; j <= i; j++)
    {
        dN(0, j) = N(k - 1, j - offset_0);
    }

    // the triangular matrix a[j] is associated with the non-vanishing B-spline function N_{j}^{k}
    vector< TriangularMatrix<GLdouble> > a(k, TriangularMatrix<GLdouble>(maximum_order_of_derivatives + 1));

    for (GLuint j = offset_0; j <= i; j++)
    {
        a[j - offset_0](0, 0) = 1.0;
    }

    // the variable factor will store the value (k-1)! / (k - 1 -r)! = (k-1)(k-2)...(k-r)
    // for each differentiation order r = 0,...,maximum_order_of_derivatives
    GLdouble factor = 1.0;

    // if the differentiation order r is bigger than k - 1, then d^r/du^r N_{j}^{k}(u) = 0 for all u
    for (GLuint r = 1; r <= min(maximum_order_of_derivatives, k - 1); r++)
    {
        // updating the value of the variable factor
        factor *= (k - r);

        // offset parameter associated with the differentiation order r: (i - k + 1) + r
        GLuint offset_r = offset_0 + r;

        // we only evaluate the derivatives of the non-vanishing normalized B-spline Basis functions
        for (GLuint j = offset_0; j <= i; j++)
        {
            // we determine the r-th row of the triangular matrix a[jj], where jj = j - offset_0
            GLuint jj = j - offset_0;
            for (GLuint l = 0; l <= r; l++)
            {
                if (l == 0)
                {
                    GLdouble denominator = _knots[j + k - r] - _knots[j];
                    a[jj](r, 0) = (denominator > 0.0) ? a[jj](r - 1, 0) / denominator : 0.0;
                }

                if (l == r)
                {
                    GLdouble denominator = _knots[j + k] - _knots[j + r];
                    a[jj](r, r) = (denominator > 0.0) ? -a[jj](r - 1, r - 1) / denominator : 0.0;
                }

                if (l > 0 && l < r)
                {
                    GLdouble denominator = _knots[j + k - r + l] - _knots[j + l];
                    a[jj](r, l) = (denominator > 0.0) ? (a[jj](r - 1, l) - a[jj](r - 1, l - 1)) / denominator : 0.0;
                }

                // we evaluate iteratively the sum that appears in the differentiation formula;
                // note that the normalized B-spline basis function N_{j+l}^{k-r}(u) vanishes
                // over [u_{i}, u_{i+1}) if j+l is not in the set {i - k + 1 + r = offset_r, ..., i}
                GLuint index = j + l;
                if (index >= offset_r && index <= i)
                {
                    dN(r, j) += a[jj](r, l) * N(k - 1 - r, index - offset_r);
                }
            }

            // multiply through by the correct factor
            dN(r, j) *= factor;
        }
    }
    return GL_TRUE;
}

RowMatrix<RealMatrix*> KnotVector::GenerateAllLookUpTablesUpToADifferentiationOrder(GLuint maximum_order_of_derivatives, GLuint division_of_integral) const
{
    GLint m = division_of_integral;
    if (m % 2)
    {
        m++;
    }

    GLint half_m = m / 2;

    RowMatrix<GLdouble> t(m + 1);

    GLuint k = _order;
    GLdouble step = (_u_max - _u_min) / m;

#pragma omp parallel for
    for (int i = 0; i < m; i++)
    {
        t[i] = _u_min + i * step;
    }
    t[m] = _u_max;

    vector< Matrix<GLdouble> > dNt(m + 1);

#pragma omp parallel for
    for (int s = 0; s <= m; s++)
    {
        ZerothAndHigherOrderDerivative(maximum_order_of_derivatives, t[s], dNt[s]);
    }

    GLuint n = GetN();

    RowMatrix<RealMatrix*> result(maximum_order_of_derivatives + 1);

    for (GLuint r = 0; r <= maximum_order_of_derivatives; r++)
    {
        result[r] = new RealMatrix(n + 1, n + 1);
    }

    if (_type == PERIODIC)
    {
        // d^r/du^r f_{n, j} (u) =  d^r/du^r [ N_{j}^{k}(u) + N_{n+1+j}^{k}(u) ], j = 0,1, ..., k-2,
        for (GLuint s = 0; s <=  static_cast<GLuint>(m); s++)
        {
            for (GLuint r = 0; r <= maximum_order_of_derivatives; r++)
            {
                for (GLuint j = 0; j <= k - 2; j++)
                {
                    dNt[s](r, j) += dNt[s](r, j + n + 1);
                }
            }

            dNt[s].ResizeColumns(n + 1);
        }
    }

    for (GLuint r = 0; r <= maximum_order_of_derivatives; r++)
    {
        for (GLuint i = 0; i <= n; i++)
        {
            for (GLuint j = 0; j <= i; j++)
            {
                double even = 0.0;
                for (GLuint s = 0; s <=  static_cast<GLuint>(half_m - 1); s++)
                {
                    even += dNt[2*s](r, i) * dNt[2*s](r, j);
                }
                even *= 2.0;

                double odd = 0.0;
                for (GLuint s = 1; s <=  static_cast<GLuint>(half_m); s++)
                {
                    odd += dNt[2*s-1](r, i) * dNt[2*s-1](r, j);
                }
                odd *= 4.0;

                GLdouble &integral = (*result[r])(i, j);
                integral += even + odd;
                integral += dNt[0](r, i) * dNt[0](r, j);
                integral += dNt[m](r, i) * dNt[m](r, j);
                integral *= step;
                integral /= 3.0;

                (*result[r])(j, i) = (*result[r])(i, j);
            }
        }
    }

    return result;

}

RealMatrix KnotVector::LookUpTableForCurveOptimizatioin(const RowMatrix<GLdouble> &weight, GLuint division_of_integral) const
{
    GLuint n = GetN();
    RealMatrix result(n + 1, n + 1);

    RowMatrix<RealMatrix*> allMatrix = GenerateAllLookUpTablesUpToADifferentiationOrder(weight.GetColumnCount(), division_of_integral);
    for (GLuint r = 1; r <= weight.GetColumnCount(); r++)
    {
        result = result + (*allMatrix[r]) * weight[r - 1];
        delete allMatrix[r];
        allMatrix[r] = nullptr;
    }
    return result;
}

RowMatrix<RealMatrix*> KnotVector::LookUpTablesForSurfaceOptimizatioin(GLdouble weight, GLuint r, GLuint division_of_integral) const
{
    TriangularMatrix<GLuint> binomialCoefficients(r + 1);
    for (GLuint i = 0; i <= r; i++)
    {
        for(GLuint j = 0; j < i; j++)
        {
            if (i == 0 || j == 0)
            {
                binomialCoefficients(i, j) = 1.0;
            }
            else
            {
                binomialCoefficients(i, j) = binomialCoefficients(i - 1, j) + binomialCoefficients(i - 1, j - 1);
            }
        }
        binomialCoefficients(i, i) = 1.0;
    }

    RowMatrix<RealMatrix*> result = GenerateAllLookUpTablesUpToADifferentiationOrder(r + 1, division_of_integral);
    for (GLuint i = 0; i <= r; i++)
    {
        GLdouble multiplier = sqrt(weight * binomialCoefficients(r, i));
        (*result[i]) = (*result[i]) * multiplier;
    }
    return result;
}
