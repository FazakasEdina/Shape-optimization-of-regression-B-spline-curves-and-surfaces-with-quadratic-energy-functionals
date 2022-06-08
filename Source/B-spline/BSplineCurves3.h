#pragma once

#include "../Core/LinearCombination3.h"
#include "../Core/RealMatrices.h"
#include "KnotVectors.h"
#include <iostream>

using namespace std;

namespace cagd
{
    // even if the type of the B-spline curve is periodic
    // we store only n + 1 control points in the _data variable of
    // the base class LinearCombination3 (note that repeated control points
    // do not increase the number of unknown parameters)
    //
    // in order to access the repeated control points of a periodic B-spline
    // curve, one can use the modulo operation with divisor n + 1
    class BSplineCurve3: public LinearCombination3
    {
        friend QTextStream& operator << (QTextStream& lhs, const BSplineCurve3& rhs);
        friend QTextStream& operator >> (QTextStream& lhs, BSplineCurve3& rhs);
    protected:
        GLuint     _n;   // n + 1 denotes the number of unrepeated control points
        KnotVector *_kv; // knot vector specified by the user

    public:
        // special constructor
        BSplineCurve3(KnotVector::Type type, GLuint k, GLuint n, GLdouble u_min = 0.0, GLdouble u_max = 1.0, GLenum data_usage_flag = GL_STATIC_DRAW);

        // copy constructor
        BSplineCurve3(const BSplineCurve3 &curve);

        // assignment operator
        BSplineCurve3& operator =(const BSplineCurve3 &rhs);

        // pure virtual abstract methods inherited from LinearCombination3 that must be redeclared and defined
        GLboolean BlendingFunctionValues(GLdouble u, RowMatrix<GLdouble>& values) const;
        GLboolean CalculateDerivatives(GLuint max_order_of_derivatives, GLdouble u, Derivatives& d) const;

        GLboolean BlendingFunctionValuesForPeriodicRegressionCurve(GLdouble u, RowMatrix<GLdouble>& values) const;

        // generates the arcs of the B-spline curve
        RowMatrix<GenericCurve3*>* GenerateImageOfArcs(GLuint max_order_of_derivatives, GLuint div_point_count, GLenum usage_flag = GL_STATIC_DRAW);
        GenericCurve3* GenerateImageOfAnArc(GLuint index, GLuint max_order_of_derivatives, GLuint div_point_count, GLenum usage_flag = GL_STATIC_DRAW) const;

        KnotVector* GetKnotVector() const;

        // frees the dynamically allocated knot vector
        virtual ~BSplineCurve3();
    };

    inline QTextStream& operator << (QTextStream& lhs, const BSplineCurve3& rhs)
    {
        lhs << rhs._kv->GetType() << " " << rhs._kv->GetOrder() << " " << rhs._n << "\n";
        lhs << rhs._u_min << " " << rhs._u_max << "\n";
        lhs << rhs._data;
        return lhs;

    }

    inline QTextStream& operator >> (QTextStream& lhs, BSplineCurve3& rhs)
    {
        GLuint k, n, type;
        GLdouble u_min, u_max;

        lhs >> type >> k >> n;
        lhs >> u_min >> u_max;

        KnotVector::Type named_type = type == 2? KnotVector::PERIODIC : (type == 0 ? KnotVector::CLAMPED : KnotVector::UNCLAMPED);
        rhs = BSplineCurve3(named_type, k, n, u_min, u_max);
        lhs >> rhs._data;

        return lhs;
    }
}
