#include "MainWindow.h"

namespace cagd
{
    MainWindow::MainWindow(QWidget *parent): QMainWindow(parent)
    {
        setupUi(this);

        /*

          the structure of the main window's central widget

         *---------------------------------------------------*
         |                 central widget                    |
         |                                                   |
         |  *---------------------------*-----------------*  |
         |  |     rendering context     |   scroll area   |  |
         |  |       OpenGL widget       | *-------------* |  |
         |  |                           | | side widget | |  |
         |  |                           | |             | |  |
         |  |                           | |             | |  |
         |  |                           | *-------------* |  |
         |  *---------------------------*-----------------*  |
         |                                                   |
         *---------------------------------------------------*

        */
        _side_widget = new SideWidget(this);

        _scroll_area = new QScrollArea(this);
        _scroll_area->setWidget(_side_widget);
        _scroll_area->setSizePolicy(_side_widget->sizePolicy());
        _scroll_area->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);

        _gl_widget = new GLWidget(this);

        centralWidget()->setLayout(new QHBoxLayout());
        centralWidget()->layout()->addWidget(_gl_widget);
        centralWidget()->layout()->addWidget(_scroll_area);

        // ---------------
        // slots
        // ---------------
        connect(_side_widget->rotate_x_slider, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_angle_x(int)));
        connect(_side_widget->rotate_y_slider, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_angle_y(int)));
        connect(_side_widget->rotate_z_slider, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_angle_z(int)));

        connect(_side_widget->zoom_factor_spin_box, SIGNAL(valueChanged(double)), _gl_widget, SLOT(set_zoom_factor(double)));

        connect(_side_widget->trans_x_spin_box, SIGNAL(valueChanged(double)), _gl_widget, SLOT(set_trans_x(double)));
        connect(_side_widget->trans_y_spin_box, SIGNAL(valueChanged(double)), _gl_widget, SLOT(set_trans_y(double)));
        connect(_side_widget->trans_z_spin_box, SIGNAL(valueChanged(double)), _gl_widget, SLOT(set_trans_z(double)));

        connect(_side_widget->global_trans, SIGNAL(toggled(bool)), _gl_widget, SLOT(set_global_transformation(bool)));
        connect(_side_widget->scale, SIGNAL(valueChanged(double)), _gl_widget, SLOT(set_scale(double)));

        // curve setters
        connect(_side_widget->type, SIGNAL(currentIndexChanged(int)), _gl_widget, SLOT(set_knotvector_type(int)));
        connect(_side_widget->k, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_knotvector_order(int)));
        connect(_side_widget->n, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_control_points(int)));
        connect(_side_widget->divPointCount, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_div_point_coint(int)));

        connect(_side_widget->scale_of_vectors, SIGNAL(valueChanged(double)), _gl_widget, SLOT(set_scale_of_vectors(double)));
        connect(_side_widget->scale_of_comb, SIGNAL(valueChanged(double)), _gl_widget, SLOT(set_scale_of_comb_vectors(double)));
        connect(_side_widget->count_of_comb, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_count_of_comb_vectors(int)));

        // patch setters
        connect(_side_widget->type_u, SIGNAL(currentIndexChanged(int)), _gl_widget, SLOT(set_knotvector_type_u(int)));
        connect(_side_widget->k_u, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_knotvector_order_u(int)));
        connect(_side_widget->n_u, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_control_points_u(int)));
        connect(_side_widget->divPointCount_u, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_div_point_coint_u(int)));
        connect(_side_widget->type_v, SIGNAL(currentIndexChanged(int)), _gl_widget, SLOT(set_knotvector_type_v(int)));
        connect(_side_widget->k_v, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_knotvector_order_v(int)));
        connect(_side_widget->n_v, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_control_points_v(int)));
        connect(_side_widget->divPointCount_v, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_div_point_coint_v(int)));

        connect(_side_widget->heatmap, SIGNAL(currentIndexChanged(int)), _gl_widget, SLOT(set_color_sheme(int)));

        // cloud setters
        connect(_side_widget->parCurve,  SIGNAL(currentIndexChanged(int)), _gl_widget, SLOT(set_parametric_curve_type(int)));
        connect(_side_widget->parSurface,  SIGNAL(currentIndexChanged(int)), _gl_widget, SLOT(set_parametric_surface_type(int)));
        connect(_side_widget->sigma_x, SIGNAL(valueChanged(double)), _gl_widget, SLOT(set_sigma_x(double)));
        connect(_side_widget->sigma_y, SIGNAL(valueChanged(double)), _gl_widget, SLOT(set_sigma_y(double)));
        connect(_side_widget->sigma_z, SIGNAL(valueChanged(double)), _gl_widget, SLOT(set_sigma_z(double)));

        connect(_side_widget->weight_num, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_weight_size(int)));
        connect(_side_widget->weight_index, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_weight_index(int)));
        connect(_side_widget->weight, SIGNAL(valueChanged(double)), _gl_widget, SLOT(set_weight_value(double)));

        connect(_side_widget->cloud_size, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_cloud_size(int)));
        connect(_side_widget->cloud_size_u, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_cloud_size_in_u_direction(int)));
        connect(_side_widget->cloud_size_v, SIGNAL(valueChanged(int)), _gl_widget, SLOT(set_cloud_size_in_v_direction(int)));

        // show/hide
        connect(_side_widget->showPolygon, SIGNAL(clicked(bool)), _gl_widget, SLOT(show_control_polygon(bool)));
        connect(_side_widget->showCloud, SIGNAL(clicked(bool)), _gl_widget, SLOT(show_cloud(bool)));
        connect(_side_widget->showCurve, SIGNAL(clicked(bool)), _gl_widget, SLOT(show_curve(bool)));
        connect(_side_widget->showArcs, SIGNAL(clicked(bool)), _gl_widget, SLOT(show_arcs(bool)));
        connect(_side_widget->showPatch, SIGNAL(clicked(bool)), _gl_widget, SLOT(show_patch(bool)));
        connect(_side_widget->showPatches, SIGNAL(clicked(bool)), _gl_widget, SLOT(show_patches(bool)));
        connect(_side_widget->showTangents, SIGNAL(clicked(bool)), _gl_widget, SLOT(show_tangents(bool)));
        connect(_side_widget->showAccVectors, SIGNAL(clicked(bool)), _gl_widget, SLOT(show_acceleration_vectors(bool)));
        connect(_side_widget->showComb, SIGNAL(clicked(bool)), _gl_widget, SLOT(show_comb(bool)));
        connect(_side_widget->showHeatMap, SIGNAL(clicked(bool)), _gl_widget, SLOT(show_heat_map(bool)));

        // other
        connect(_side_widget->cpSize, SIGNAL(valueChanged(double)), _gl_widget, SLOT(set_control_point_size(double)));
        connect(_side_widget->cloudPointSize, SIGNAL(valueChanged(double)), _gl_widget, SLOT(set_cloud_point_size(double)));
        connect(_side_widget->mode, SIGNAL(toggled(bool)), _gl_widget, SLOT(set_dark_light_mode(bool)));

        // ---------------
        // signals
        // ---------------
        connect(_gl_widget, SIGNAL(display_elapsed_time(int)), _side_widget->elapsedTime, SLOT(display(int)));

        connect(_gl_widget, SIGNAL(display_index_weight(double)), _side_widget->weight, SLOT(setValue(double)));

        connect(_gl_widget, SIGNAL(display_total_curvature(QString)), _side_widget->total_curvature, SLOT(setText(QString)));
        connect(_gl_widget, SIGNAL(display_length_of_curve(QString)), _side_widget->length_of_curve, SLOT(setText(QString)));
        connect(_gl_widget, SIGNAL(display_kinetic_energy(QString)), _side_widget->kinetic_energy, SLOT(setText(QString)));

        connect(_gl_widget, SIGNAL(display_surface_area(QString)), _side_widget->surface_area, SLOT(setText(QString)));
        connect(_gl_widget, SIGNAL(display_gaussian_curvature(QString)), _side_widget->gaussian_curvature, SLOT(setText(QString)));
        connect(_gl_widget, SIGNAL(display_mean_curvature(QString)), _side_widget->mean_curvature, SLOT(setText(QString)));
        connect(_gl_widget, SIGNAL(display_willmore_curvature(QString)), _side_widget->willmore_energy, SLOT(setText(QString)));
        connect(_gl_widget, SIGNAL(display_log_willmore_curvature(QString)), _side_widget->log_willmore, SLOT(setText(QString)));
        connect(_gl_widget, SIGNAL(display_umbilic_deviation(QString)), _side_widget->umbilic_deviation, SLOT(setText(QString)));
        connect(_gl_widget, SIGNAL(display_log_umbilic_deviation(QString)), _side_widget->log_umbilic_deviation, SLOT(setText(QString)));
        connect(_gl_widget, SIGNAL(display_total_curvature_surface(QString)), _side_widget->total_curvature_surface, SLOT(setText(QString)));
        connect(_gl_widget, SIGNAL(display_log_total_curvature(QString)), _side_widget->log_total_curvature, SLOT(setText(QString)));

        connect(_gl_widget, SIGNAL(optimization_done(bool)), _side_widget->weight, SLOT(setEnabled(bool)));
        connect(_gl_widget, SIGNAL(set_focus()), _side_widget->weight, SLOT(setFocus()));
        connect(_gl_widget, SIGNAL(display_zoom_value(double)), _side_widget->zoom_factor_spin_box, SLOT(setValue(double)));
        connect(_gl_widget, SIGNAL(display_scale(double)), _side_widget->scale, SLOT(setValue(double)));
        connect(_gl_widget, SIGNAL(display_trans_x(double)), _side_widget->trans_x_spin_box, SLOT(setValue(double)));
        connect(_gl_widget, SIGNAL(display_trans_y(double)), _side_widget->trans_y_spin_box, SLOT(setValue(double)));
        connect(_gl_widget, SIGNAL(display_trans_z(double)), _side_widget->trans_z_spin_box, SLOT(setValue(double)));
        connect(_gl_widget, SIGNAL(display_angle_x(int)), _side_widget->rotate_x_slider, SLOT(setValue(int)));
        connect(_gl_widget, SIGNAL(display_angle_y(int)), _side_widget->rotate_y_slider, SLOT(setValue(int)));
        connect(_gl_widget, SIGNAL(display_angle_z(int)), _side_widget->rotate_z_slider, SLOT(setValue(int)));

        connect(_gl_widget, SIGNAL(set_displayed_knotvector_type(int)), _side_widget->type, SLOT(setCurrentIndex(int)));
        connect(_gl_widget, SIGNAL(set_displayed_knotvector_order(int)), _side_widget->k, SLOT(setValue(int)));
        connect(_gl_widget, SIGNAL(set_displayed_control_points(int)), _side_widget->n, SLOT(setValue(int)));
        connect(_gl_widget, SIGNAL(set_displayed_div_point_coint(int)), _side_widget->divPointCount, SLOT(setValue(int)));

        connect(_gl_widget, SIGNAL(set_displayed_knotvector_type_u(int)), _side_widget->type_u, SLOT(setCurrentIndex(int)));
        connect(_gl_widget, SIGNAL(set_displayed_knotvector_type_v(int)), _side_widget->type_v, SLOT(setCurrentIndex(int)));
        connect(_gl_widget, SIGNAL(set_displayed_knotvector_order_u(int)), _side_widget->k_u, SLOT(setValue(int)));
        connect(_gl_widget, SIGNAL(set_displayed_knotvector_order_v(int)), _side_widget->k_v, SLOT(setValue(int)));
        connect(_gl_widget, SIGNAL(set_displayed_control_points_u(int)), _side_widget->n_u, SLOT(setValue(int)));
        connect(_gl_widget, SIGNAL(set_displayed_control_points_v(int)), _side_widget->n_v, SLOT(setValue(int)));
        connect(_gl_widget, SIGNAL(set_displayed_div_point_coint_u(int)), _side_widget->divPointCount_u, SLOT(setValue(int)));
        connect(_gl_widget, SIGNAL(set_displayed_div_point_coint_v(int)), _side_widget->divPointCount_v, SLOT(setValue(int)));

        connect(_gl_widget, SIGNAL(display_index_of_derivativ(int)), _side_widget->weight_index, SLOT(setValue(int)));
        connect(_gl_widget, SIGNAL(display_max_derivativ_num(int)), _side_widget->weight_num , SLOT(setValue(int)));

        // others
        connect(_side_widget->singleStep, SIGNAL(valueChanged(double)), this, SLOT(setSingleStepOfWeight(double)));
        connect(_gl_widget, SIGNAL(mask_has_changed(uint)), this, SLOT(applyMask(uint)));

        if (_gl_widget->_current_running == GLWidget::CURVE || _gl_widget->_current_running == GLWidget::CURVEPOINTCLOUD)
        {
            _side_widget->toolBox->setItemEnabled(2, false);
            _side_widget->cloud_size_u->setEnabled(false);
            _side_widget->cloud_size_v->setEnabled(false);
        }
        else if (_gl_widget->_current_running == GLWidget::PATCH || _gl_widget->_current_running == GLWidget::SURFACEPOINTCLOUD)
        {
            _side_widget->toolBox->setItemEnabled(1, false);
            _side_widget->cloud_size->setEnabled(false);
        }
        if (_gl_widget->_current_running == GLWidget::ALL)
        {
            _side_widget->cloud_size_u->setEnabled(false);
            _side_widget->cloud_size_v->setEnabled(false);
            _side_widget->cloud_size->setEnabled(false);
            _side_widget->sigma_x->setEnabled(false);
            _side_widget->sigma_y->setEnabled(false);
            _side_widget->sigma_z->setEnabled(false);
            _side_widget->parCurve->setEnabled(false);
            _side_widget->parSurface->setEnabled(false);
        }

        connect(this->save_one_variable_point_cloud, SIGNAL(triggered()), _gl_widget, SLOT(save_point_cloud_around_curve()));
        connect(this->open_one_variable_point_cloud, SIGNAL(triggered()), _gl_widget, SLOT(load_point_cloud_around_curve()));
        connect(this->save_two_variable_point_cloud, SIGNAL(triggered()), _gl_widget, SLOT(save_point_cloud_around_surface()));
        connect(this->open_two_variable_point_cloud, SIGNAL(triggered()), _gl_widget, SLOT(load_point_cloud_around_surface()));
        connect(this->save_B_spline_curve, SIGNAL(triggered()), _gl_widget, SLOT(save_bspline_curve()));
        connect(this->open_B_spline_curve, SIGNAL(triggered()), _gl_widget, SLOT(load_bspline_curve()));
        connect(this->save_B_spline_surface, SIGNAL(triggered()), _gl_widget, SLOT(save_bspline_surface()));
        connect(this->open_B_spline_surface, SIGNAL(triggered()), _gl_widget, SLOT(load_bspline_surface()));
    }

    //--------------------------------
    // implementation of private slots
    //--------------------------------
    void MainWindow::on_action_Quit_triggered()
    {
        qApp->exit(0);
    }

    void MainWindow::setSingleStepOfWeight(double value)
    {
        _side_widget->weight->setSingleStep(value);
    }

    void MainWindow::applyMask(uint mask)
    {
        // 0b0001 -> one variable point cloud
        // 0b0010 -> two variable point cloud
        // 0b0100 -> B-spline curve
        // 0b1000 -> B-spline surface
        _side_widget->type->setEnabled(mask & 0b0001);
        _side_widget->k->setEnabled(mask & 0b0001);
        _side_widget->n->setEnabled(mask & 0b0001);
        _side_widget->divPointCount->setEnabled(mask & 0b0101);

        _side_widget->type_u->setEnabled(mask & 0b0010);
        _side_widget->type_v->setEnabled(mask & 0b0010);
        _side_widget->k_u->setEnabled(mask & 0b0010);
        _side_widget->k_v->setEnabled(mask & 0b0010);
        _side_widget->n_u->setEnabled(mask & 0b0010);
        _side_widget->n_v->setEnabled(mask & 0b0010);
        _side_widget->divPointCount_u->setEnabled(mask & 0b1010);
        _side_widget->divPointCount_v->setEnabled(mask & 0b1010); //(mask & 0b1000) | (mask & 0b0010));

        _side_widget->weight->setEnabled(mask & 0b0011);
        _side_widget->weight_index->setEnabled(mask & 0b0011);
        _side_widget->weight_num->setEnabled(mask & 0b0011);

    }
}
