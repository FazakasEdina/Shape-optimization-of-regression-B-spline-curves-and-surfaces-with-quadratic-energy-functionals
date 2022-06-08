#pragma once

#include <Core/TensorProductSurfaces3.h>
#include "KnotVectors.h"

namespace cagd
{
    class BSplinePatch3: public TensorProductSurface3
    {
        friend QTextStream& operator << (QTextStream& lhs, const BSplinePatch3& rhs);
        friend QTextStream& operator >> (QTextStream& lhs, BSplinePatch3& rhs);
    private:
        GLuint _v_n, _u_n;
        KnotVector *_v_kv;
        KnotVector *_u_kv;
    public:
        // special constructor
        BSplinePatch3(KnotVector::Type u_type, KnotVector::Type v_type,
                      GLuint u_k, GLuint v_k,
                      GLuint u_n, GLuint v_n,
                      GLdouble u_min = 0.0, GLdouble u_max = 1.0,
                      GLdouble v_min = 0.0, GLdouble v_max = 1.0);

        // copy constructor
        BSplinePatch3(const BSplinePatch3 &patch);

        // assignment operator
        BSplinePatch3& operator =(const BSplinePatch3 &rhs);

        // inherited pure virtual methods have to be redeclared and defined
        GLboolean UBlendingFunctionValues(GLdouble u, RowMatrix<GLdouble> &blending_values) const;

        GLboolean VBlendingFunctionValues(GLdouble v, RowMatrix<GLdouble> &blending_values) const;
        GLboolean CalculatePartialDerivatives(
                        GLuint maximum_order_of_partial_derivatives,
                        GLdouble u, GLdouble v, PartialDerivatives &pd) const;

        GLboolean UBlendingFunctionValuesForPeriodicRegressionSurface(GLdouble u, RowMatrix<GLdouble>& blending_values) const;
        GLboolean VBlendingFunctionValuesForPeriodicRegressionSurface(GLdouble v, RowMatrix<GLdouble>& blending_values) const;

        // generates the patches of the B-spline patch
        Matrix<TriangulatedMesh3*>* GenerateImageOfPatches(GLuint u_div_point_count, GLuint v_div_point_count,
                                                           ImageColorScheme color_sheme = DEFAULT_NULL_FRAGMENT,
                                                           GLenum usage_flag = GL_STATIC_DRAW);

        TriangulatedMesh3* GenerateImageOfAnPatch(GLuint row_index, GLuint column_index,
                                                  GLuint u_div_point_count, GLuint v_div_point_count,
                                                  ImageColorScheme color_sheme = DEFAULT_NULL_FRAGMENT,
                                                  GLenum usage_flag = GL_STATIC_DRAW) const;

        KnotVector* GetKnotVectorU() const;
        KnotVector* GetKnotVectorV() const;

        virtual ~BSplinePatch3();
    };

    inline QTextStream& operator << (QTextStream& lhs, const BSplinePatch3& rhs)
    {
        lhs << rhs._u_kv->GetType() << " " << rhs._u_kv->GetOrder() << " " << rhs._u_n << "\n";
        lhs << rhs._v_kv->GetType() << " " << rhs._v_kv->GetOrder() << " " << rhs._v_n << "\n";
        lhs << rhs._u_min << " " << rhs._u_max << "\n";
        lhs << rhs._v_min << " " << rhs._v_max << "\n";

        lhs << rhs._data;
        return lhs;

    }

    inline QTextStream& operator >> (QTextStream& lhs, BSplinePatch3& rhs)
    {
        GLuint k_u, k_v, n_u, n_v, type_u, type_v;
        GLdouble u_min, u_max, v_min, v_max;

        lhs >> type_u >> k_u >> n_u;
        lhs >> type_v >> k_v >> n_v;
        lhs >> u_min >> u_max;
        lhs >> v_min >> v_max;

        KnotVector::Type named_type_u = type_u == 2? KnotVector::PERIODIC : (type_u == 0 ? KnotVector::CLAMPED : KnotVector::UNCLAMPED);
        KnotVector::Type named_type_v = type_v == 2? KnotVector::PERIODIC : (type_v == 0 ? KnotVector::CLAMPED : KnotVector::UNCLAMPED);

        rhs = BSplinePatch3(named_type_u, named_type_v, k_u, k_v, n_u, n_v, u_min, u_max, v_min, v_max);
        lhs >> rhs._data;
        return lhs;
    }
}
