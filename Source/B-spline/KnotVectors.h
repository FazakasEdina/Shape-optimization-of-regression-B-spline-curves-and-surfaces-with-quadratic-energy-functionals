#pragma once

#include "../Core/Matrices.h"
#include <Core/RealMatrices.h>
#include <GL/glew.h>

namespace cagd
{
    //-------------------------------------------------------------------
    // A B-spline curve of order k can be unclamped, clamped or periodic.
    //
    // In case of unclamped and clamped B-spline curves:
    //  - the number of control points is n + 1;
    //  - the number of monotone increasing knot values is n + k + 1;
    //  - in case of clamped B-spline curves the first and last k knot
    //    values coincide.
    //
    // In case of periodic B-spline curves:
    //  - the number of control points is n + k, where the last k - 1
    //    control points are repeated instances of the first k - 1
    //    control points;
    //  - the number of monotone increasing knot values is n + 2k, where
    //    the last k knot values determine k - 1 subintervals, the
    //    lengths of which coincide with the length of the first k - 1
    //    subintervals.
    //-------------------------------------------------------------------
    class KnotVector
    {
        friend std::ostream& operator << (std::ostream& lhs, const KnotVector& rhs);
    public:
        enum Type{ CLAMPED, UNCLAMPED, PERIODIC };

    protected:
        Type                _type;
        GLuint              _order;                 // k
        GLuint              _control_point_count;   // n + 1 (clamped/unclamped) or n + k (periodic)
        GLdouble            _u_min, _u_max;         // [u_{k-1}, u_{n+1}] (unclamped/periodic) or [u_{k-1}, u_{n+k}] (periodic)
        RowMatrix<GLdouble> _knots;                 // monotone increasing knot values:
                                                    //   u_{0}, u_{1}, ..., u_{n+k}    (unclamped/clamped)
                                                    //   u_{0}, u_{1}, ..., u_{n+2k-1} (periodic)
    public:
        // special constructor
        KnotVector(Type type, GLuint k, GLuint n, GLdouble u_min = 0.0, GLdouble u_max = 1.0);

        // get knot value by value
        GLdouble operator [](GLuint index) const;

        // get knot value by reference
        GLdouble& operator [](GLuint index);

        // determines an index stored in variable i for which u is in [u_{i}, u_{i + 1})
        GLboolean FindSpan(GLdouble u, GLuint& i) const;

        // evaluates non-vanishing normalized B-spline function values in a lower triangular matrix of the form
        // N[row - 1][column] = N_{span - row + column + 1}^{row}, row = 1, ..., order, column = 0, 1, ..., row - 1
        GLboolean EvaluateNonZeroBSplineFunctions(GLdouble u, GLuint& i, TriangularMatrix<GLdouble>& N) const;

        // derivatives
        GLboolean ZerothAndHigherOrderDerivative(GLuint maximum_order_of_derivatives, GLdouble u, Matrix<GLdouble> &dN) const;

        RowMatrix<RealMatrix*> GenerateAllLookUpTablesUpToADifferentiationOrder(GLuint maximum_order_of_derivatives, GLuint division_of_integral) const;

        RealMatrix LookUpTableForCurveOptimizatioin(const RowMatrix<GLdouble> &weight, GLuint division_of_integral) const;

        RowMatrix<RealMatrix*> LookUpTablesForSurfaceOptimizatioin(GLdouble weight, GLuint r, GLuint division_of_integral) const;

        // getters
        Type GetType() const;
        GLuint GetOrder() const;
        GLuint GetControlPointCount() const;
        GLdouble GetMin() const;
        GLdouble GetMax() const;
        GLuint GetN() const;
    };

}
