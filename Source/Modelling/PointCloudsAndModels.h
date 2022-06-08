#pragma once

#include <PointCloud/PointCloudAroundCurve3.h>
#include <PointCloud/PointCloudAroundSurface3.h>
#include <Core/ShaderPrograms.h>
#include <Core/Materials.h>

namespace cagd
{
    class PointCloudsAndModels
    {
        class OneVariablePointCloudAndItsRegression
        {
        public:
            PointCloudAroundCurve3          *_cloud = nullptr;

            BSplineCurve3                   *_bs = nullptr;
            GenericCurve3                   *_img_bs = nullptr;
            RowMatrix<GenericCurve3*>       *_arcs = nullptr;

            KnotVector::Type                _type = KnotVector::PERIODIC;
            GLuint                          _n = 10, _k = 4;
            GLuint                          _div_point_count = 20;
            GLdouble                        _curve_u_min;
            GLdouble                        _curve_u_max;

            RowMatrix<GLdouble>             _weight;
            double                          _trans_x = 0.0, _trans_y = 0.0, _trans_z = 0.0;
            double                          _scale = 1.0;
            int                             _angle_x = 0.0, _angle_y = 0.0, _angle_z = 0.0;
        };

        class BSplineCurve
        {
        public:
            BSplineCurve3                   *_bs = nullptr;
            GenericCurve3                   *_img_bs = nullptr;
            RowMatrix<GenericCurve3*>       *_arcs = nullptr;

            KnotVector::Type                _type;
            GLuint                          _n, _k;
            GLuint                          _div_point_count = 20;

            double                          _trans_x = 0.0, _trans_y = 0.0, _trans_z = 0.0;
            double                          _scale = 1.0;
            int                             _angle_x = 0.0, _angle_y = 0.0, _angle_z = 0.0;
        };

        GLdouble                        _scale_of_vectors = 0.4;
        GLdouble                        _scale_of_comb_vectors = 1.0;
        GLuint                          _count_of_comb_vectors = 100;

        bool                            _show_comb = true;
        bool                            _show_cloud = true;
        bool                            _show_control_polygon = true;
        bool                            _show_curve = false;
        bool                            _show_arcs = true;
        bool                            _show_tangents = false;
        bool                            _show_acceleration_vectors = false;

        class TwoVariablePointCloudAndItsRegression
        {
        public:
            PointCloudAroundSurface3        *_cloud = nullptr;

            BSplinePatch3                   *_patch = nullptr;
            TriangulatedMesh3               *_img_patch = nullptr;
            Matrix<TriangulatedMesh3*>      *_patches = nullptr;

            KnotVector::Type                _type_u = KnotVector::PERIODIC;
            KnotVector::Type                _type_v = KnotVector::PERIODIC;
            GLuint                          _n_u = 5, _n_v = 5;
            GLuint                          _k_u = 4, _k_v = 4;
            GLuint                          _div_point_count_u = 20;
            GLuint                          _div_point_count_v = 20;

            GLdouble                        _surface_u_min, _surface_u_max;
            GLdouble                        _surface_v_min, _surface_v_max;

            RowMatrix<GLdouble>             _total_energies;
            RowMatrix<GLdouble>             _weight;

            double                          _trans_x = 0.0, _trans_y = 0.0, _trans_z = 0.0;
            double                          _scale = 1.0;
            int                             _angle_x = 0.0, _angle_y = 0.0, _angle_z = 0.0;
        };

        class BSplineSurface
        {
        public:
            BSplinePatch3                   *_patch = nullptr;
            TriangulatedMesh3               *_img_patch = nullptr;
            Matrix<TriangulatedMesh3*>      *_patches = nullptr;

            KnotVector::Type                _type_u;
            KnotVector::Type                _type_v;
            GLuint                          _n_u, _n_v;
            GLuint                          _k_u, _k_v;
            GLuint                          _div_point_count_u = 20;
            GLuint                          _div_point_count_v = 20;

            RowMatrix<GLdouble>             _total_energies;

            double                          _trans_x = 0.0, _trans_y = 0.0, _trans_z = 0.0;
            double                          _scale = 1.0;
            int                             _angle_x = 0.0, _angle_y = 0.0, _angle_z = 0.0;
        };

        bool                            _show_heat_map = false;
        bool                            _no_shader = false;
        bool                            _show_patches = true;
        bool                            _show_patch = false;

        ShaderProgram                   _two_sided_lighting_shader;
        ShaderProgram                   _twosided_color;
        GLboolean                       _loging_is_enabled = GL_FALSE;

        TensorProductSurface3::ImageColorScheme _selected_color_sheme =
                TensorProductSurface3::DEFAULT_NULL_FRAGMENT;


        TriangulatedMesh3               _unit_sphere;
        double                          _cloud_point_size = 0.03;
        double                          _control_point_size = 0.03;

    public:
        enum ModelType {ONE_VARIABLE, TWO_VARIABLE, CURVE, SURFACE};
        ModelType _selected_type;
        GLuint _selected_model;


        RowMatrix<OneVariablePointCloudAndItsRegression> _one_var_point_clouds;
        RowMatrix<BSplineCurve> _curves;
        RowMatrix<TwoVariablePointCloudAndItsRegression> _two_var_point_clouds;
        RowMatrix<BSplineSurface> _surfaces;

    public:
        PointCloudsAndModels();

        bool createOneVariablePointCloudRegression(int index);
        bool createImageOfBSplineCurve(int index);
        bool createTwoVariablePointCloudRegression(int index);
        bool createImageOfBSplineSurface(int index);

        bool renderOneVariablePointCloudAndRegressions(bool dark_mode);
        bool renderBSplineCurves(bool dark_mode);
        bool renderTwoVariablePointCloudAndRegressions(bool dark_mode);
        bool renderBSplineSurfaces(bool dark_mode);

        void deleteOneVariablePointCloudRegression(int index);
        void deleteBSplineCurve(int index);
        void deleteTwoVariablePointCloudRegression(int index);
        void deleteBSplineSurface(int index);

        void deleteAllOneVariablePointCloudRegressions();
        void deleteAllBSplineCurves();
        void deleteAllTwoVariablePointCloudRegressions();
        void deleteAllBSplineSurfaces();

        // -------
        // Methods for regression and simple curves
        // -------
        void updateCurveByOnePoint(int point_index);

        bool set_control_points(int value);
        bool set_knotvector_type(int index);
        bool set_knotvector_order(int value);
        bool set_div_point_coint(int value);

        bool set_scale_of_vectors(double value);
        bool set_scale_of_comb_vectors(double value);
        bool set_count_of_comb_vectors(int value);

        DCoordinate3* get_control_point_of_curve(int index);
        RowMatrix<GLdouble> get_total_energies_of_selected_curve();

        bool show_curve(bool value);
        bool show_arcs(bool value);
        bool show_tangents(bool value);
        bool show_acceleration_vectors(bool value);
        bool show_comb(bool value);

        // -------
        // Methods for regression and simple surfaces
        // -------
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

        void set_shader_available(bool value);
        bool set_color_sheme(int index);

        DCoordinate3* get_control_point_of_surface(int row, int column);
        RowMatrix<GLdouble> get_total_energies_of_surface();

        bool show_heat_map(bool value);
        bool show_patches(bool value);
        bool show_patch(bool value);

        // -------
        // Common methods
        // -------
        bool set_weight_size(int value);
        bool set_weight_value(int index, double value);
        bool set_cloud_point_size(double value);
        bool set_control_point_size(double value);

        bool show_control_polygon(bool value);
        bool show_cloud(bool value);

        GLuint get_named_objects_count_of_one_var_point_clouds();
        GLuint get_named_objects_count_of_bspline_curves();
        GLuint get_named_objects_count_of_two_var_point_clouds();
        GLuint get_named_objects_count_of_bspline_surfaces();

        // return true -> clicked object is the rendered point cloud and regression, or model
        // return false -> clicked object is a control point
        bool find_and_select_clicked_object(GLuint name, GLint &row, int &column);

        bool set_angle_x(int value);
        bool set_angle_y(int value);
        bool set_angle_z(int value);

        bool set_scale_factor(double value);

        bool set_trans_x(double value);
        bool set_trans_y(double value);
        bool set_trans_z(double value);

        // -------
        // Methods for update gui
        // -------
        int get_control_points();
        int get_knotvector_type();
        int get_knotvector_order();
        int get_div_point_coint();
        int get_knotvector_type_u();
        int get_knotvector_type_v();
        int get_knotvector_order_u();
        int get_knotvector_order_v();
        int get_control_points_u();
        int get_control_points_v();
        int get_div_point_coint_u();
        int get_div_point_coint_v();

        int get_angle_x();
        int get_angle_y();
        int get_angle_z();

        double get_scale_factor();

        double get_trans_x();
        double get_trans_y();
        double get_trans_z();

        int get_weight_size();
        RowMatrix<GLdouble> get_weight();

        ~PointCloudsAndModels();

        void loadOneVariablePointCloud(QTextStream &in);
        void loadTwoVariablePointCloud(QTextStream &in);
        void loadBSplineCurve(QTextStream &in);
        void loadBSplineSurface(QTextStream &in);

        void saveOneVariablePointCloud(QTextStream &out);
        void saveTwoVariablePointCloud(QTextStream &out);
        void saveBSplineCurve(QTextStream &out);
        void saveBSplineSurface(QTextStream &out);

    };

}
