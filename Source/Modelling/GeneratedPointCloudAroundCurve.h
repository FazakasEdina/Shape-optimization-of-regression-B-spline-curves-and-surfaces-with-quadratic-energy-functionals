#pragma once

#include "ClassicBSplineCurve3.h"
#include <Parametric/ParametricCurves3.h>
#include <PointCloud/PointCloudAroundCurve3.h>
#include <Test/TestFunctions.h>

namespace cagd
{
    class GeneratedPointCloudAroundCurve : ClassicBSplineCurve3
    {
    public:
        friend QTextStream& operator << (QTextStream& lhs, const GeneratedPointCloudAroundCurve& rhs);
        friend QTextStream& operator >> (QTextStream& lhs, GeneratedPointCloudAroundCurve& rhs);

    private:
        PointCloudAroundCurve3          *_curve_cloud = nullptr;

        RowMatrix<GLdouble>             *_sigma;
        RowMatrix<GLdouble>             _weight;
        double                          _cloud_point_size = 0.03;

        GLuint                          _point_cloud_size = 1000;

        GLdouble                        _scale_of_comb_vectors = 1.0;
        GLuint                          _count_of_comb_vectors = 100;

        bool                            _show_cloud = true;
        bool                            _show_comb = true;

        RowMatrix<GLdouble>             _total_energies;
        // attributes of point cloud around curve
        GLdouble                       _curve_u_min;
        GLdouble                       _curve_u_max;

        int                             _selected_pc = 0;
        ParametricCurve3               *_pc = nullptr;
        GenericCurve3                  *_img_of_pc = nullptr;
    public:
    public:
        GeneratedPointCloudAroundCurve();

        bool createParametricCurve();
        bool createPointCloudAroundCurve();
        bool createBSplineCurve();

        bool renderPointCloudAroundCurve(bool dark_mode);

        void deleteParametricCurve();
        void deletePointCloudAroundCurve();
        void deleteAllObject();
        void deleteBSplineCurve();

        // methods from classic curves
        void updateCurveByOnePoint(int index);

        bool set_control_points(int value);
        bool set_knotvector_type(int index);
        bool set_knotvector_order(int value);
        bool set_div_point_coint(int value);

        bool set_scale_of_vectors(double value);
        bool set_control_point_size(double value);

        bool show_control_polygon(bool value);
        bool show_curve(bool value);
        bool show_arcs(bool value);
        bool show_tangents(bool value);
        bool show_acceleration_vectors(bool value);

        GLuint get_n();
        DCoordinate3* get_control_point(int index);

        // methods for cloud and optimization
        RowMatrix<GLdouble> get_energies();

        bool set_parametric_curve_type(int index);

        bool set_sigma_x(double value);
        bool set_sigma_y(double value);
        bool set_sigma_z(double value);

        bool set_weight_size(int value);
        bool set_weight_value(int index, double value);

        bool set_cloud_size(int value);

        bool set_scale_of_comb_vectors(double value);
        bool set_count_of_comb_vectors(int value);

        bool show_cloud(bool value);
        bool show_comb(bool value);

        bool set_cloud_point_size(double value);
    };

    inline QTextStream& operator << (QTextStream& lhs, const GeneratedPointCloudAroundCurve& rhs)
    {
        lhs << (*rhs._curve_cloud);
        return lhs;
    }

    inline QTextStream& operator >> (QTextStream& lhs, GeneratedPointCloudAroundCurve& rhs)
    {
        rhs.deleteParametricCurve();
        rhs.deleteBSplineCurve();

        lhs >> (*rhs._curve_cloud);
        rhs._curve_cloud->FindTheInterval(rhs._curve_u_min, rhs._curve_u_max);
        rhs.createBSplineCurve();

        return lhs;
    }
}
