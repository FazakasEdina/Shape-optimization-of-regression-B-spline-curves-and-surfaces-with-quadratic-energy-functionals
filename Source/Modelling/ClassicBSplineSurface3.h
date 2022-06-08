#pragma once

#include <B-spline/BSplinePatches3.h>
#include <Core/ShaderPrograms.h>
#include <Core/Materials.h>

namespace cagd
{
    class ClassicBSplineSurface3
    {
        friend QTextStream& operator << (QTextStream& lhs, const ClassicBSplineSurface3& rhs);
        friend QTextStream& operator >> (QTextStream& lhs, ClassicBSplineSurface3& rhs);
    protected:
        KnotVector::Type                _type_u = KnotVector::PERIODIC;
        KnotVector::Type                _type_v = KnotVector::PERIODIC;
        GLuint                          _n_u = 5, _n_v = 5;
        GLuint                          _k_u = 4, _k_v = 4;
        GLuint                          _div_point_count_u = 30;
        GLuint                          _div_point_count_v = 30;

        BSplinePatch3                   *_patch = nullptr;
        TriangulatedMesh3               *_img_patch = nullptr;
        Matrix<TriangulatedMesh3*>      *_patches = nullptr;
        double                          _point_size = 0.03;

        bool                            _no_shader = false;
        bool                            _show_control_polygon = true;

        bool                            _show_patches = true;
        bool                            _show_patch = false;

        TriangulatedMesh3               _unit_sphere;
        ShaderProgram                   _two_sided_lighting_shader;
        GLboolean                       _loging_is_enabled = GL_FALSE;
    public:
        ClassicBSplineSurface3();

        virtual bool createBSplinePatch();
        bool renderBSPlinePatch();
        void deleteBSplinePatch();

        void updatePatchByOnePoint(int row, int column);
        void updateOnePatchByIndexes(int row, int column, int index_1, int index_2);

        bool set_knotvector_type_u(int index);
        bool set_knotvector_type_v(int index);
        bool set_knotvector_order_u(int value);
        bool set_knotvector_order_v(int value);
        bool set_control_points_u(int value);
        bool set_control_points_v(int value);
        bool set_div_point_coint_u(int value);
        bool set_div_point_coint_v(int value);

        bool show_patches(bool value);
        bool show_patch(bool value);
        bool show_control_polygon(bool value);

        bool set_control_point_size(double value);

        GLuint get_size();
        DCoordinate3* get_control_point(int row, int column);
        void calculate_indexes(GLuint name, GLint &row, GLint &column);
        void set_shader_available(bool value);
    };

    inline QTextStream& operator << (QTextStream& lhs, const ClassicBSplineSurface3& rhs)
    {
        lhs << (*rhs._patch);
        return lhs;
    }

    inline QTextStream& operator >> (QTextStream& lhs, ClassicBSplineSurface3& rhs)
    {
        lhs >> *rhs._patch;

        KnotVector* kv_u = (*rhs._patch).GetKnotVectorU();
        rhs._type_u = kv_u->GetType();
        rhs._k_u = kv_u->GetOrder();
        rhs._n_u = kv_u->GetN();

        KnotVector* kv_v = (*rhs._patch).GetKnotVectorV();
        rhs._type_v = kv_v->GetType();
        rhs._k_v = kv_v->GetOrder();
        rhs._n_v = kv_v->GetN();

        if (!rhs._patch->UpdateVertexBufferObjectsOfData())
        {
            rhs.deleteBSplinePatch();
            throw Exception ("Could not create the image of the classic B-spline patch.");
        }

        RowMatrix<GLdouble> _energies;
        rhs._img_patch = rhs._patch->GenerateImage(rhs._div_point_count_u, rhs._div_point_count_v, _energies, TensorProductSurface3::DEFAULT_NULL_FRAGMENT);

        if (!rhs._img_patch || !rhs._img_patch->UpdateVertexBufferObjects())
        {
            rhs.deleteBSplinePatch();
            throw Exception("Could not create the VBO's of the classic B-spline patch.");
        }

        rhs._patches = rhs._patch->GenerateImageOfPatches(rhs._div_point_count_u, rhs._div_point_count_v, TensorProductSurface3::DEFAULT_NULL_FRAGMENT);
        if (!rhs._patches)
        {
            rhs.deleteBSplinePatch();
            throw Exception("Could not generate the patches of the classic B-spline patch!");
        }

        for (GLuint i = 0; i < rhs._patches->GetRowCount(); i++)
        {
            for (GLuint j = 0; j < rhs._patches->GetColumnCount(); j++)
            {
                if (!(*rhs._patches)(i, j)->UpdateVertexBufferObjects())
                {
                    rhs.deleteBSplinePatch();
                    throw Exception("Could not update the VBO of all patches of the classic B-spline patch!");
                }
            }
        }
        return lhs;
    }
}
