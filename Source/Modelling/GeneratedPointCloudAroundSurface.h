#pragma once

#include "ClassicBSplineSurface3.h"
#include <PointCloud/PointCloudAroundSurface3.h>
#include <Test/TestFunctions.h>

namespace cagd
{
    class GeneratedPointCloudAroundSurface : ClassicBSplineSurface3
    {
        friend QTextStream& operator << (QTextStream& lhs, const GeneratedPointCloudAroundSurface& rhs);
        friend QTextStream& operator >> (QTextStream& lhs, GeneratedPointCloudAroundSurface& rhs);

    private:
        RowMatrix<GLdouble>             *_sigma;
        RowMatrix<GLdouble>             _weight;
        double                          _cloud_point_size = 0.03;

        bool                            _show_cloud = true;
        bool                            _show_heat_map = false;

        GLuint                          _point_cloud_size_u = 20;
        GLuint                          _point_cloud_size_v = 20;
        ShaderProgram                   _twosided_color;
        GLboolean                       _loging_is_enabled = GL_FALSE;

        TensorProductSurface3::ImageColorScheme _selected_color_sheme =
                TensorProductSurface3::DEFAULT_NULL_FRAGMENT;
        RowMatrix<GLdouble>             _total_energies;

        // attributes of point cloud around surface
        GLdouble                       _surface_u_min, _surface_u_max;
        GLdouble                       _surface_v_min, _surface_v_max;

        int                            _selected_ps = 0;
        ParametricSurface3              *_ps = nullptr;
    public:
        PointCloudAroundSurface3        *_surface_cloud = nullptr;

    public:
        GeneratedPointCloudAroundSurface();

        bool createParametricSurface();
        bool createPointCloudAroundSurface();
        bool createBSplinePatch();

        void updatePatchByOnePoint(int row, int column);

        bool renderPointCloudAroundSurface(bool dark_mode);

        void deleteParametricSurface();
        void deletePointCloudAroundSurface();
        void deleteAllObjects();
        void deleteBSplinePatch();

        // methods from classic patch
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

        // methods for cloud and regression
        bool set_color_sheme(int index);

        bool set_parametric_surface_type(int index);

        bool set_sigma_x(double value);
        bool set_sigma_y(double value);
        bool set_sigma_z(double value);

        bool set_weight_size(int value);
        bool set_weight_value(int index, double value);

        bool set_cloud_size_in_u_direction(int value);
        bool set_cloud_size_in_v_direction(int value);

        bool show_cloud(bool value);
        bool show_heat_map(bool value);

        bool set_cloud_point_size(double value);

        RowMatrix<GLdouble> get_energies();

        QTextStream& OutSurface (QTextStream& lhs);
        QTextStream& InSurface (QTextStream& lhs);
    };

    inline QTextStream& operator << (QTextStream& lhs, const GeneratedPointCloudAroundSurface& rhs)
    {
        lhs << (*rhs._surface_cloud);
        return lhs;
    }

    inline QTextStream& operator >> (QTextStream& lhs, GeneratedPointCloudAroundSurface& rhs)
    {
        rhs.deleteParametricSurface();
        rhs.deleteBSplinePatch();

        lhs >> (*rhs._surface_cloud);

        rhs._surface_cloud->FindTheInterval(rhs._surface_u_min, rhs._surface_u_max, rhs._surface_v_min, rhs._surface_v_max);
        rhs.createBSplinePatch();
        return lhs;
    }

    inline QTextStream& GeneratedPointCloudAroundSurface::OutSurface (QTextStream& lhs)
    {
        lhs << (*this->_patch);
        return lhs;
    }

    inline QTextStream& GeneratedPointCloudAroundSurface::InSurface (QTextStream& lhs)
    {
        lhs >> *this->_patch;

        KnotVector* kv_u = (*this->_patch).GetKnotVectorU();
        this->_type_u = kv_u->GetType();
        this->_k_u = kv_u->GetOrder();
        this->_n_u = kv_u->GetN();

        KnotVector* kv_v = (*this->_patch).GetKnotVectorV();
        this->_type_v = kv_v->GetType();
        this->_k_v = kv_v->GetOrder();
        this->_n_v = kv_v->GetN();

        if (!this->_patch->UpdateVertexBufferObjectsOfData())
        {
            this->deleteBSplinePatch();
            throw Exception ("Could not create the image of the classic B-spline patch.");
        }

        this->_img_patch = this->_patch->GenerateImage(this->_div_point_count_u, this->_div_point_count_v, this->_total_energies, TensorProductSurface3::DEFAULT_NULL_FRAGMENT);

        if (!this->_img_patch || !this->_img_patch->UpdateVertexBufferObjects())
        {
            this->deleteBSplinePatch();
            throw Exception("Could not create the VBO's of the classic B-spline patch.");
        }

        this->_patches = this->_patch->GenerateImageOfPatches(this->_div_point_count_u, this->_div_point_count_v, TensorProductSurface3::DEFAULT_NULL_FRAGMENT);
        if (!this->_patches)
        {
            this->deleteBSplinePatch();
            throw Exception("Could not generate the patches of the classic B-spline patch!");
        }

        for (GLuint i = 0; i < this->_patches->GetRowCount(); i++)
        {
            for (GLuint j = 0; j < this->_patches->GetColumnCount(); j++)
            {
                if (!(*this->_patches)(i, j)->UpdateVertexBufferObjects())
                {
                    this->deleteBSplinePatch();
                    throw Exception("Could not update the VBO of all patches of the classic B-spline patch!");
                }
            }
        }
        return lhs;
    }
}
