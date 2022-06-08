#pragma once

#include <GL/glew.h>
#include <QOpenGLWidget>
#include <QWheelEvent>

#include <QFileDialog>
#include <QDir>
#include <QMessageBox>
#include <QDataStream>

#include "PointCloud/PointCloudAroundCurve3.h"
#include "PointCloud/PointCloudAroundSurface3.h"
#include "Arcball.h"
#include "Modelling/ClassicBSplineCurve3.h"
#include "Modelling/ClassicBSplineSurface3.h"
#include "Modelling/GeneratedPointCloudAroundCurve.h"
#include "Modelling/GeneratedPointCloudAroundSurface.h"
#include "Modelling/PointCloudsAndModels.h"

namespace cagd
{
    class GLWidget: public QOpenGLWidget
    {
        Q_OBJECT

    public:
        enum Run {CURVE, PATCH, CURVEPOINTCLOUD, SURFACEPOINTCLOUD, ALL};
        Run _current_running = ALL;
    private:
        // variables defining the projection matrix
        double       _aspect;            // aspect ratio of the rendering window
        double       _fovy;              // field of view in direction y
        double       _z_near, _z_far;    // distance of near and far clipping planes

        // variables defining the model-view matrix
        double       _eye[3], _center[3], _up[3];

        // variables needed by transformations
        int         _angle_x, _angle_y, _angle_z;
        double      _zoom;
        double      _trans_x, _trans_y, _trans_z;

        // Arcball
        Arcball         *_arcball;
        double          _rotationAngle;
        DCoordinate3    *_rotationAxis;

        // mouse press
        GLboolean            _named_object_clicked;
        GLint                _row;
        GLint                _column;
        bool                 _clicked_point_cloud = false;
        GLdouble             _reposition_unit;

        // general attributes of point cloud
        RowMatrix<GLdouble>             *_sigma;
        RowMatrix<GLdouble>             _weight;
        GLuint                          _selected_weight_index = 0;

        bool                            _dark_mode = true;

        bool                            _global_transformation_selected = true;

        // models
        ClassicBSplineCurve3            *_bspline_curve_model = nullptr;
        ClassicBSplineSurface3          *_bspline_surface_model = nullptr;
        GeneratedPointCloudAroundCurve  *_regression_curve_model = nullptr;
        GeneratedPointCloudAroundSurface *_regression_surface_model = nullptr;
        PointCloudsAndModels            *_models = nullptr;

    public:
        // special and default constructor
        // the format specifies the properties of the rendering window
        GLWidget(QWidget* parent = 0);

        // redeclared virtual functions
        void initializeGL();
        void paintGL();
        void resizeGL(int w, int h);

        // handling mouse events
        void mousePressEvent(QMouseEvent *event);
        void wheelEvent(QWheelEvent *event);
        void mouseMoveEvent(QMouseEvent *event);
        void mouseReleaseEvent(QMouseEvent *event);

        // destructor that frees all dynamically allocated memory objects
        ~GLWidget();

    public slots:
        // public event handling methods/slots
        void set_angle_x(int value);
        void set_angle_y(int value);
        void set_angle_z(int value);

        void set_zoom_factor(double value);

        void set_trans_x(double value);
        void set_trans_y(double value);
        void set_trans_z(double value);

        void set_scale(double value);

        void set_global_transformation(bool value);

        // curve setters
        void set_knotvector_type(int index);
        void set_knotvector_order(int value);
        void set_control_points(int value);
        void set_div_point_coint(int value);

        void set_scale_of_vectors(double value);
        void set_scale_of_comb_vectors(double value);
        void set_count_of_comb_vectors(int value);

        // patch setters
        void set_knotvector_type_u(int index);
        void set_knotvector_type_v(int index);
        void set_knotvector_order_u(int value);
        void set_knotvector_order_v(int value);
        void set_control_points_u(int value);
        void set_control_points_v(int value);
        void set_div_point_coint_u(int value);
        void set_div_point_coint_v(int value);

        void set_color_sheme(int index);

        // cloud setters
        void set_parametric_curve_type(int index);
        void set_parametric_surface_type(int index);
        void set_sigma_x(double value);
        void set_sigma_y(double value);
        void set_sigma_z(double value);

        void set_weight_size(int value);
        void set_weight_index(int value);
        void set_weight_value(double value);

        void set_cloud_size(int value);
        void set_cloud_size_in_u_direction(int value);
        void set_cloud_size_in_v_direction(int value);

        // show/hide
        void show_control_polygon(bool value);
        void show_cloud(bool value);

        void show_curve(bool value);
        void show_arcs(bool value);
        void show_tangents(bool value);
        void show_acceleration_vectors(bool value);

        void show_patches(bool value);
        void show_patch(bool value);

        void show_comb(bool value);
        void show_heat_map(bool value);

        // other
        void set_control_point_size(double value);
        void set_cloud_point_size(double value);
        void set_dark_light_mode(bool value);

        // menu options
        void save_point_cloud_around_curve();
        void load_point_cloud_around_curve();
        void save_point_cloud_around_surface();
        void load_point_cloud_around_surface();

        void save_bspline_curve();
        void load_bspline_curve();
        void save_bspline_surface();
        void load_bspline_surface();

    signals:
        void display_angle_x(int value);
        void display_angle_y(int value);
        void display_angle_z(int value);

        void display_zoom_factor(double value);

        void display_trans_x(double value);
        void display_trans_y(double value);
        void display_trans_z(double value);

        void display_scale(double value);

        void display_elapsed_time(int value);

        void display_index_weight(double value);
        void display_index_of_derivativ(int value);
        void display_max_derivativ_num(int value);

        void display_total_curvature(QString value);
        void display_length_of_curve(QString value);
        void display_kinetic_energy(QString value);

        void display_surface_area(QString value);
        void display_gaussian_curvature(QString value);
        void display_mean_curvature(QString value);
        void display_willmore_curvature(QString value);
        void display_log_willmore_curvature(QString value);
        void display_umbilic_deviation(QString value);
        void display_log_umbilic_deviation(QString value);
        void display_total_curvature_surface(QString value);
        void display_log_total_curvature(QString value);

        void optimization_done(bool value);
        void set_focus();

        void display_zoom_value(double);

        void set_displayed_knotvector_type(int value);
        void set_displayed_knotvector_order(int value);
        void set_displayed_control_points(int value);
        void set_displayed_div_point_coint(int value);

        void set_displayed_knotvector_type_u(int index);
        void set_displayed_knotvector_type_v(int index);
        void set_displayed_knotvector_order_u(int value);
        void set_displayed_knotvector_order_v(int value);
        void set_displayed_control_points_u(int value);
        void set_displayed_control_points_v(int value);
        void set_displayed_div_point_coint_u(int value);
        void set_displayed_div_point_coint_v(int value);

        void mask_has_changed(uint mask);
    };
}
