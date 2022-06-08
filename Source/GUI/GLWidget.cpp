#include "GLWidget.h"


#if !defined(__APPLE__)
#include <GL/glu.h>
#endif

#include <iostream>
#include <fstream>
#include <algorithm>
#include <QElapsedTimer>
#include <QtWidgets/QApplication>
using namespace std;

#include <Core/Materials.h>
#include <Core/Exceptions.h>
#include <Core/Constants.h>
#include <B-spline/KnotVectors.h>
#include <Test/TestFunctions.h>
#include <QDebug>

namespace cagd
{
//--------------------------------
// special and default constructor
//--------------------------------
GLWidget::GLWidget(QWidget *parent): QOpenGLWidget(parent)
{
}

//--------------------------------------------------------------------------------------
// this virtual function is called once before the first call to paintGL() or resizeGL()
//--------------------------------------------------------------------------------------
void GLWidget::initializeGL()
{
    // creating a perspective projection matrix
    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();

    _aspect = static_cast<double>(width()) / static_cast<double>(height());
    _z_near = 1.0;
    _z_far  = 1000.0;
    _fovy   = 45.0;

    gluPerspective(_fovy, _aspect, _z_near, _z_far);

    // setting the model view matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    _eye[0] = _eye[1] = 0.0; _eye[2] = 6.0;
    _center[0] = _center[1] = _center[2] = 0.0;
    _up[0] = _up[2] = 0.0; _up[1] = 1.0;

    gluLookAt(_eye[0], _eye[1], _eye[2], _center[0], _center[1], _center[2], _up[0], _up[1], _up[2]);

    // enabling the depth test
    glEnable(GL_DEPTH_TEST);

    // setting the background color
    if (!_dark_mode)
    {
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    }
    else
    {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    }

    // initial values of transformation parameters
    _angle_x = _angle_y = _angle_z = 0.0;
    _trans_x = _trans_y = _trans_z = 0.0;
    _zoom = 1.0;

    // initialization of arcball
    _arcball = new Arcball();
    _arcball->setWidthHeight(width(), height());
    _rotationAngle = 0.0;
    _rotationAxis = new DCoordinate3(0.0, 0.0, 0.0);

    _weight.ResizeColumns(1);
    _weight[0] = 0.0;

    try
    {
        // initializing the OpenGL Extension Wrangler library
        GLenum error = glewInit();

        if (error != GLEW_OK)
        {
            throw Exception("Could not initialize the OpenGL Extension Wrangler Library!");
        }

        if (!glewIsSupported("GL_VERSION_2_0"))
        {
            throw Exception("Your graphics card is not compatible with OpenGL 2.0+! "
                            "Try to update your driver or buy a new graphics adapter!");
        }

        glEnable(GL_LIGHTING);
        glEnable(GL_NORMALIZE);
        glEnable(GL_DEPTH_TEST);

        _named_object_clicked = GL_FALSE;
        _reposition_unit     = 0.05;

        _sigma = new (nothrow) RowMatrix<GLdouble>(3);
        (*_sigma)[0] = (GLdouble)0.5;
        (*_sigma)[1] = (GLdouble)0.5;
        (*_sigma)[2] = (GLdouble)0.5;

        RowMatrix<GLdouble> totalEnergies;

        QElapsedTimer timer;
        timer.start();
        switch(_current_running)
        {
        case CURVE:
            _bspline_curve_model = new (nothrow) ClassicBSplineCurve3();
            _bspline_curve_model->createBSplineCurve();
            break;
        case PATCH:
            _bspline_surface_model = new (nothrow) ClassicBSplineSurface3();
            _bspline_surface_model->createBSplinePatch();
            break;
        case CURVEPOINTCLOUD:
            _regression_curve_model = new (nothrow) GeneratedPointCloudAroundCurve();
            _regression_curve_model->createParametricCurve();
            _regression_curve_model->createPointCloudAroundCurve();
            _regression_curve_model->createBSplineCurve();

            totalEnergies = _regression_curve_model->get_energies();
            emit display_total_curvature(QString::number(totalEnergies[0]));
            emit display_length_of_curve(QString::number(totalEnergies[1]));
            emit display_kinetic_energy(QString::number(totalEnergies[2]));
            break;
        case SURFACEPOINTCLOUD:
            _regression_surface_model = new (nothrow) GeneratedPointCloudAroundSurface();

            _regression_surface_model->createParametricSurface();
            _regression_surface_model->createPointCloudAroundSurface();
            _regression_surface_model->createBSplinePatch();

            totalEnergies = _regression_surface_model->get_energies();
            emit(display_surface_area(QString::number(totalEnergies[1])));
            emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
            emit(display_mean_curvature(QString::number(totalEnergies[3])));
            emit(display_willmore_curvature(QString::number(totalEnergies[4])));
            emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
            emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
            emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
            emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
            emit(display_log_total_curvature(QString::number(totalEnergies[9])));

            break;
        case ALL:
            _models = new (nothrow) PointCloudsAndModels();
            break;
        }
        emit display_elapsed_time(timer.elapsed());
    }
    catch (Exception &e)
    {
        cout << e << endl;
        e.showReason();
    }
}

//-----------------------
// the rendering function
//-----------------------
void GLWidget::paintGL()
{
    // clears the color and depth buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glRotatef(_rotationAngle * 2, - _rotationAxis->x(),  - _rotationAxis->y(),- _rotationAxis->z());

    // stores/duplicates the original model view matrix
    glPushMatrix();

    // applying transformations
    glRotatef(_angle_x, 1.0, 0.0, 0.0);
    glRotatef(_angle_y, 0.0, 1.0, 0.0);
    glRotatef(_angle_z, 0.0, 0.0, 1.0);

    glTranslated(_trans_x, _trans_y, _trans_z);

    glScaled(_zoom, _zoom, _zoom);

    switch(_current_running)
    {
    case CURVE:
        _bspline_curve_model->renderBSplineCurve(_dark_mode);
        break;
    case PATCH:
        _bspline_surface_model->renderBSPlinePatch();
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->renderPointCloudAroundCurve(_dark_mode);
        break;
    case SURFACEPOINTCLOUD:
        _regression_surface_model->renderPointCloudAroundSurface(_dark_mode);
        break;
    case ALL:
        _models->renderOneVariablePointCloudAndRegressions(_dark_mode);
        _models->renderBSplineCurves(_dark_mode);
        _models->renderTwoVariablePointCloudAndRegressions(_dark_mode);
        _models->renderBSplineSurfaces(_dark_mode);
        break;
    }

    // pops the current matrix stack, replacing the current matrix with the one below it on the stack,
    // i.e., the original model view matrix is restored
    glPopMatrix();

}

//-----------
// destructor
//-----------
GLWidget::~GLWidget()
{
    switch(_current_running)
    {
    case CURVE:
        _bspline_curve_model->deleteBSplineCurve();
        break;
    case PATCH:
        _bspline_surface_model->deleteBSplinePatch();
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->deleteAllObject();
        break;
    case SURFACEPOINTCLOUD:
        _regression_surface_model->deleteAllObjects();
        break;
    case ALL:
        delete _models;
        _models = nullptr;
    }
    delete _sigma;
    _sigma = nullptr;

    delete _arcball;
    _arcball = nullptr;

    delete _rotationAxis;
    _rotationAxis = nullptr;
}

//----------------------------------------------------------------------------
// when the main window is resized one needs to redefine the projection matrix
//----------------------------------------------------------------------------
void GLWidget::resizeGL(int w, int h)
{
    // setting the new size of the rendering context
    glViewport(0, 0, w, h);

    // redefining the projection matrix
    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();

    _aspect = static_cast<double>(w) / static_cast<double>(h);
    _arcball->setWidthHeight(w, h);

    gluPerspective(_fovy, _aspect, _z_near, _z_far);

    // switching back to the model view matrix
    glMatrixMode(GL_MODELVIEW);

    update();
}

//----------------------------------------------------------------------------
// interacting with user - mouse control
//----------------------------------------------------------------------------
void GLWidget::mousePressEvent(QMouseEvent *event)
{
    makeCurrent();

    event->accept();

    if (event->button() == Qt::LeftButton)
    {
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);

        GLuint size;
        switch (_current_running)
        {
        case CURVE:
            size = 4 * _bspline_curve_model->get_n();
            break;
        case CURVEPOINTCLOUD:
            size = 4 * _regression_curve_model->get_n();
            break;
        case PATCH:
            size = 4 * _bspline_surface_model->get_size();
            break;
        case SURFACEPOINTCLOUD:
            size = 4 * _regression_surface_model->get_size();
            break;
        case ALL:
            size = 4 * (_models->get_named_objects_count_of_bspline_curves() +
                    _models->get_named_objects_count_of_bspline_surfaces() +
                    _models->get_named_objects_count_of_one_var_point_clouds() +
                    _models->get_named_objects_count_of_two_var_point_clouds());
            break;
        }

        GLuint *pick_buffer = new GLuint[size];
        glSelectBuffer(size, pick_buffer);

        glRenderMode(GL_SELECT);

        glInitNames();
        glPushName(0);

        GLfloat projection_matrix[16];
        glGetFloatv(GL_PROJECTION_MATRIX, projection_matrix);

        glMatrixMode(GL_PROJECTION);

        glPushMatrix();

        glLoadIdentity();
        gluPickMatrix((GLdouble)event->pos().x() * devicePixelRatio(), (GLdouble)(viewport[3] - event->pos().y() * devicePixelRatio()), 5.0, 5.0, viewport);

        glMultMatrixf(projection_matrix);

        glMatrixMode(GL_MODELVIEW);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glRotatef(_rotationAngle * 2, - _rotationAxis->x(),  - _rotationAxis->y(),- _rotationAxis->z());

        glPushMatrix();
        // one should use the same model-view matrix as in the paintGL method, therefore:

        // rotating around the coordinate axes
        glRotatef(_angle_x, 1.0, 0.0, 0.0);
        glRotatef(_angle_y, 0.0, 1.0, 0.0);
        glRotatef(_angle_z, 0.0, 0.0, 1.0);

        glTranslated(_trans_x, _trans_y, _trans_z);
        glScalef(_zoom, _zoom, _zoom);

        // render only the clickable geometries!
        switch(_current_running)
        {
        case CURVE:
            _bspline_curve_model->renderBSplineCurve(_dark_mode);
            break;
        case PATCH:
            _bspline_surface_model->set_shader_available(false);
            _bspline_surface_model->renderBSPlinePatch();
            _bspline_surface_model->set_shader_available(true);
            break;
        case CURVEPOINTCLOUD:
            _regression_curve_model->renderPointCloudAroundCurve(_dark_mode);
            break;
        case SURFACEPOINTCLOUD:
            _regression_surface_model->set_shader_available(false);
            _regression_surface_model->renderPointCloudAroundSurface(_dark_mode);
            _regression_surface_model->set_shader_available(true);
            break;
        case ALL:
            _models->set_shader_available(false);
            _models->renderOneVariablePointCloudAndRegressions(_dark_mode);
            _models->renderBSplineCurves(_dark_mode);
            _models->renderTwoVariablePointCloudAndRegressions(_dark_mode);
            _models->renderBSplineSurfaces(_dark_mode);
            _models->set_shader_available(true);
            break;
        }
        glPopMatrix();

        glMatrixMode(GL_PROJECTION);
        glPopMatrix();

        glMatrixMode(GL_MODELVIEW);

        // creating and analysing the selection buffer based on the detected hit counts
        int hit_count = glRenderMode(GL_RENDER);
        if (hit_count)
        {
            GLuint closest_selected = pick_buffer[3];
            GLuint closest_depth    = pick_buffer[1];

            for (GLuint i = 1; i < (unsigned int) hit_count; ++i)
            {
                GLuint offset = i * 4;
                if (pick_buffer[offset + 1] < closest_depth)
                {
                    closest_selected = pick_buffer[offset + 3];
                    closest_depth    = pick_buffer[offset + 1];
                }
            }

            switch (_current_running)
            {
            case CURVE:
                _column    = closest_selected;
                cout << "identifier/name = " << closest_selected << endl;
                break;
            case CURVEPOINTCLOUD:
                _column    = closest_selected;
                cout << "identifier/name = " << closest_selected << endl;
                break;
            case PATCH:
                _bspline_surface_model->calculate_indexes(closest_selected, _row, _column);
                cout << "identifier/name = " << closest_selected
                     << ", row = " << _row
                     << ", column = " << _column << endl;
                break;
            case SURFACEPOINTCLOUD:
                _regression_surface_model->calculate_indexes(closest_selected, _row, _column);

                cout << "identifier/name = " << closest_selected
                     << ", row = " << _row
                     << ", column = " << _column << endl;
                break;
            case ALL:
                _selected_weight_index = 0;
                emit display_index_of_derivativ(0);

                _clicked_point_cloud = _models->find_and_select_clicked_object(closest_selected, _row, _column);
                cout << "identifier/name = " << closest_selected
                     << "; row = " << _row
                     << "; column = " << _column
                     <<"; selected model = " << _models->_selected_model
                    <<"; type index = " << _models->_selected_type
                    << "; is selected models (true=1, false=0)= " << _clicked_point_cloud << endl;

                if (!_global_transformation_selected)
                {
                    emit display_trans_x(_models->get_trans_x());
                    emit display_trans_y(_models->get_trans_y());
                    emit display_trans_z(_models->get_trans_z());

                    emit display_scale(_models->get_scale_factor());

                    emit display_angle_x(_models->get_angle_x());
                    emit display_angle_y(_models->get_angle_y());
                    emit display_angle_z(_models->get_angle_z());
                }

                if (_models->_selected_type == PointCloudsAndModels::ONE_VARIABLE || _models->_selected_type == PointCloudsAndModels::CURVE)
                {
                    RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_selected_curve();
                    emit display_total_curvature(QString::number(totalEnergies[0]));
                    emit display_length_of_curve(QString::number(totalEnergies[1]));
                    emit display_kinetic_energy(QString::number(totalEnergies[2]));

                    emit display_surface_area("0.0");
                    emit display_gaussian_curvature("0.0");
                    emit display_mean_curvature("0.0");
                    emit display_willmore_curvature("0.0");
                    emit display_log_willmore_curvature("0.0");
                    emit display_umbilic_deviation("0.0");
                    emit display_log_umbilic_deviation("0.0");
                    emit display_total_curvature_surface("0.0");
                    emit display_log_total_curvature("0.0");

                    emit set_displayed_div_point_coint(_models->get_div_point_coint());

                    if (_models->_selected_type == PointCloudsAndModels::ONE_VARIABLE)
                    {
                        emit set_displayed_knotvector_type(_models->get_knotvector_type());
                        emit set_displayed_knotvector_order(_models->get_knotvector_order());
                        emit set_displayed_control_points(_models->get_control_points());

                        _weight.ResizeColumns(_models->get_weight_size());
                        _weight = _models->get_weight();
                        emit display_max_derivativ_num(_weight.GetColumnCount());

                        emit mask_has_changed(0b0001);
                    }
                    else
                    {
                        emit mask_has_changed(0b0100);
                    }
                }
                else
                {
                    RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_surface();
                    emit(display_surface_area(QString::number(totalEnergies[1])));
                    emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
                    emit(display_mean_curvature(QString::number(totalEnergies[3])));
                    emit(display_willmore_curvature(QString::number(totalEnergies[4])));
                    emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
                    emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
                    emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
                    emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
                    emit(display_log_total_curvature(QString::number(totalEnergies[9])));

                    emit display_total_curvature("0.0");
                    emit display_length_of_curve("0.0");
                    emit display_kinetic_energy("0.0");

                    if (_models->_selected_type == PointCloudsAndModels::TWO_VARIABLE)
                    {
                        emit set_displayed_knotvector_type_u(_models->get_knotvector_type_u());
                        emit set_displayed_knotvector_type_v(_models->get_knotvector_type_v());
                        emit set_displayed_knotvector_order_u(_models->get_knotvector_order_u());
                        emit set_displayed_knotvector_order_v(_models->get_knotvector_order_v());
                        emit set_displayed_control_points_u(_models->get_control_points_u());
                        emit set_displayed_control_points_v(_models->get_control_points_v());
                        emit set_displayed_div_point_coint_u(_models->get_div_point_coint_u());
                        emit set_displayed_div_point_coint_v(_models->get_div_point_coint_v());

                        _weight.ResizeColumns(_models->get_weight_size());
                        _weight = _models->get_weight();
                        emit display_max_derivativ_num(_weight.GetColumnCount());

                        emit mask_has_changed(0b0010);
                    }
                    else
                    {
                        emit mask_has_changed(0b1000);
                    }
                }

            }

            _named_object_clicked = GL_TRUE;
        }
        else
        {
            _named_object_clicked = GL_FALSE;
            _arcball->startRotation(event->position().x(),  event->position().y());
        }

        delete[] pick_buffer;

        update();
    }
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
    event->accept();

    if (_named_object_clicked)
    {
        DCoordinate3 *point;
        switch (_current_running)
        {
        case CURVE:
            point = _bspline_curve_model->get_control_point(_column);
            break;
        case CURVEPOINTCLOUD:
            point = _regression_curve_model->get_control_point(_column);
            break;
        case PATCH:
            point = _bspline_surface_model->get_control_point(_row, _column);
            break;
        case SURFACEPOINTCLOUD:
            point = _regression_surface_model->get_control_point(_row, _column);
            break;
        case ALL:
            if (!_clicked_point_cloud)
            {
                if (_models->_selected_type == PointCloudsAndModels::ONE_VARIABLE
                        || _models->_selected_type == PointCloudsAndModels::CURVE)
                {
                    point = _models->get_control_point_of_curve(_column);
                }
                else
                {
                    point = _models->get_control_point_of_surface(_row, _column);
                }
            }
            else
            {
                point = nullptr;
            }
            break;
        }

        GLdouble     &x     = (*point)[0];
        GLdouble     &y     = (*point)[1];
        GLdouble     &z     = (*point)[2];

        // wheel + Ctrl
        if (event->modifiers() & Qt::ControlModifier)
        {
            if (!_clicked_point_cloud)
            {
                x += event->angleDelta().y() / 120.0 * _reposition_unit;
            }
            else
            {
                if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::ONE_VARIABLE)
                {
                    _models->_one_var_point_clouds[_models->_selected_model]._trans_x +=
                            event->angleDelta().y() / 120.0 * _reposition_unit;
                }
                if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::CURVE)
                {
                    _models->_curves[_models->_selected_model]._trans_x +=
                            event->angleDelta().y() / 120.0 * _reposition_unit;
                }
                if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::TWO_VARIABLE)
                {
                    _models->_two_var_point_clouds[_models->_selected_model]._trans_x +=
                            event->angleDelta().y() / 120.0 * _reposition_unit;
                }
                if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::SURFACE)
                {
                    _models->_surfaces[_models->_selected_model]._trans_x +=
                            event->angleDelta().y() / 120.0 * _reposition_unit;
                }
             }
        }

        // wheel + Alt
        if (event->modifiers() & Qt::AltModifier)
        {
            if (!_clicked_point_cloud)
            {
                y += event->angleDelta().x() / 120.0 * _reposition_unit;
            }
            else
            {
                if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::ONE_VARIABLE)
                {
                    _models->_one_var_point_clouds[_models->_selected_model]._trans_y +=
                            event->angleDelta().x() / 120.0 * _reposition_unit;
                }
                if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::CURVE)
                {
                    _models->_curves[_models->_selected_model]._trans_y +=
                            event->angleDelta().x() / 120.0 * _reposition_unit;
                }
                if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::TWO_VARIABLE)
                {
                    _models->_two_var_point_clouds[_models->_selected_model]._trans_y +=
                            event->angleDelta().x() / 120.0 * _reposition_unit;
                }
                if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::SURFACE)
                {
                    _models->_surfaces[_models->_selected_model]._trans_y +=
                            event->angleDelta().x() / 120.0 * _reposition_unit;
                }
            }
        }

        // wheel + Shift
        if (event->modifiers() & Qt::ShiftModifier)
        {
            if (!_clicked_point_cloud)
            {
                z += event->angleDelta().y() / 120.0 * _reposition_unit;
            }
            else
            {
                if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::ONE_VARIABLE)
                {
                    _models->_one_var_point_clouds[_models->_selected_model]._trans_z +=
                            event->angleDelta().y() / 120.0 * _reposition_unit;
                }
                if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::CURVE)
                {
                    _models->_curves[_models->_selected_model]._trans_z +=
                            event->angleDelta().y() / 120.0 * _reposition_unit;
                }
                if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::TWO_VARIABLE)
                {
                    _models->_two_var_point_clouds[_models->_selected_model]._trans_z +=
                            event->angleDelta().y() / 120.0 * _reposition_unit;
                }
                if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::SURFACE)
                {
                    _models->_surfaces[_models->_selected_model]._trans_z +=
                            event->angleDelta().y() / 120.0 * _reposition_unit;
                }
            }
        }

        QElapsedTimer timer;
        timer.start();

        RowMatrix<GLdouble> totalEnergies;
        switch (_current_running)
        {
        case CURVE:
            _bspline_curve_model->updateCurveByOnePoint(_column);
            break;
        case CURVEPOINTCLOUD:
            _regression_curve_model->updateCurveByOnePoint(_column);
            totalEnergies = _regression_curve_model->get_energies();
            emit display_total_curvature(QString::number(totalEnergies[0]));
            emit display_length_of_curve(QString::number(totalEnergies[1]));
            emit display_kinetic_energy(QString::number(totalEnergies[2]));
            break;
        case PATCH:
            _bspline_surface_model->updatePatchByOnePoint(_row, _column);
            break;
        case SURFACEPOINTCLOUD:
            _regression_surface_model->updatePatchByOnePoint(_row, _column);
            totalEnergies = _regression_surface_model->get_energies();
            emit(display_surface_area(QString::number(totalEnergies[1])));
            emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
            emit(display_mean_curvature(QString::number(totalEnergies[3])));
            emit(display_willmore_curvature(QString::number(totalEnergies[4])));
            emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
            emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
            emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
            emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
            emit(display_log_total_curvature(QString::number(totalEnergies[9])));
            break;
        case ALL:
            if (!_clicked_point_cloud)
            {
                if (_models->_selected_type == PointCloudsAndModels::ONE_VARIABLE
                        || _models->_selected_type == PointCloudsAndModels::CURVE)
                {
                    _models->updateCurveByOnePoint(_column);
                    totalEnergies = _models->get_total_energies_of_selected_curve();
                    emit display_total_curvature(QString::number(totalEnergies[0]));
                    emit display_length_of_curve(QString::number(totalEnergies[1]));
                    emit display_kinetic_energy(QString::number(totalEnergies[2]));
                }
                else
                {
                    _models->updatePatchByOnePoint(_row, _column);
                    totalEnergies = _models->get_total_energies_of_surface();
                    emit(display_surface_area(QString::number(totalEnergies[1])));
                    emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
                    emit(display_mean_curvature(QString::number(totalEnergies[3])));
                    emit(display_willmore_curvature(QString::number(totalEnergies[4])));
                    emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
                    emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
                    emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
                    emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
                    emit(display_log_total_curvature(QString::number(totalEnergies[9])));
                }
            }
            emit display_scale(_models->get_scale_factor());

            emit display_angle_x(_models->get_angle_x());
            emit display_angle_y(_models->get_angle_y());
            emit display_angle_z(_models->get_angle_z());

            emit display_trans_x(_models->get_trans_x());
            emit display_trans_y(_models->get_trans_y());
            emit display_trans_z(_models->get_trans_z());

            break;
        }

        emit display_elapsed_time(timer.elapsed());
        update();
    }
    else
    {
        // zoom in or out
        _zoom += event->angleDelta().y() / 120.0 * _reposition_unit;
        emit(display_zoom_value(_zoom));
        update();
    }
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    event->accept();

    if (!_named_object_clicked)
    {
        _arcball->updateRotation(event->position().x(), event->position().y());
        _rotationAngle  = _arcball->stopRotation(_rotationAxis);
        _arcball->startRotation(event->position().x(),  event->position().y());
        update();
    }
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    event->accept();
    if (!_named_object_clicked)
    {
        _rotationAngle  = _arcball->stopRotation(_rotationAxis);
        update();
    }
}

//-----------------------------------
// implementation of the public slots
//-----------------------------------
void GLWidget::set_angle_x(int value)
{
    if (_global_transformation_selected)
    {
        if (_angle_x != value)
        {
            _angle_x = value;
            update();
        }
    }
    else if (_current_running == ALL)
    {
        if(_models->set_angle_x(value))
        {
            update();
        }
    }
}

void GLWidget::set_angle_y(int value)
{
    if (_global_transformation_selected)
    {
        if (_angle_y != value)
        {
            _angle_y = value;
            update();
        }
    }
    else if (_current_running == ALL)
    {
        if(_models->set_angle_y(value))
        {
            update();
        }
    }
}

void GLWidget::set_angle_z(int value)
{
    if (_global_transformation_selected)
    {
        if (_angle_z != value)
        {
            _angle_z = value;
            update();
        }
    }
    else if (_current_running == ALL)
    {
        if(_models->set_angle_z(value))
        {
            update();
        }
    }
}

void GLWidget::set_zoom_factor(double value)
{
    if (_zoom != value)
    {
        _zoom = value;
        update();
    }
}

void GLWidget::set_trans_x(double value)
{

    if (_global_transformation_selected)
    {
        if (_trans_x != value)
        {
            _trans_x = value;
            update();
        }
    }
    else if (_current_running == ALL)
    {
        if(_models->set_trans_x(value))
        {
            update();
        }
    }
}

void GLWidget::set_trans_y(double value)
{  
    if (_global_transformation_selected)
    {
        if (_trans_y != value)
        {
            _trans_y = value;
            update();
        }
    }
    else if (_current_running == ALL)
    {
        if(_models->set_trans_y(value))
        {
            update();
        }
    }
}

void GLWidget::set_trans_z(double value)
{
    if (_global_transformation_selected)
    {
        if (_trans_z != value)
        {
            _trans_z = value;
            update();
        }
    }
    else if (_current_running == ALL)
    {
        if(_models->set_trans_z(value))
        {
            update();
        }
    }
}

void GLWidget::set_scale(double value)
{
    if (!_global_transformation_selected && _current_running == ALL)
    {
        if(_models->set_scale_factor(value))
        {
            update();
        }
    }
}

void GLWidget::set_global_transformation(bool value)
{
    if (_global_transformation_selected != value)
    {
        _global_transformation_selected = value;

        if (_global_transformation_selected)
        {
            emit display_trans_x(_trans_x);
            emit display_trans_y(_trans_y);
            emit display_trans_z(_trans_z);

            emit display_scale(1.0);

            emit display_angle_x(_angle_x);
            emit display_angle_y(_angle_y);
            emit display_angle_z(_angle_z);

            update();
        }
        else
        {
            if (_current_running == ALL)
            {
                emit display_trans_x(_models->get_trans_x());
                emit display_trans_y(_models->get_trans_y());
                emit display_trans_z(_models->get_trans_z());

                emit display_scale(_models->get_scale_factor());

                emit display_angle_x(_models->get_angle_x());
                emit display_angle_y(_models->get_angle_y());
                emit display_angle_z(_models->get_angle_z());

                update();
            }
        }
    }
}

//-----------------------------------
// curve setters
//-----------------------------------
void GLWidget::set_knotvector_type(int index)
{
    QElapsedTimer timer;
    timer.start();

    RowMatrix<GLdouble> totalEnergies;
    switch (_current_running)
    {
    case CURVE:
        _bspline_curve_model->set_knotvector_type(index);
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->set_knotvector_type(index);
        totalEnergies = _regression_curve_model->get_energies();
        emit display_total_curvature(QString::number(totalEnergies[0]));
        emit display_length_of_curve(QString::number(totalEnergies[1]));
        emit display_kinetic_energy(QString::number(totalEnergies[2]));
        break;
    case PATCH:
        break;
    case SURFACEPOINTCLOUD:
        break;
    case ALL:
        if (_models->set_knotvector_type(index))
        {
            totalEnergies = _models->get_total_energies_of_selected_curve();
            emit display_total_curvature(QString::number(totalEnergies[0]));
            emit display_length_of_curve(QString::number(totalEnergies[1]));
            emit display_kinetic_energy(QString::number(totalEnergies[2]));
        }
        else
        {
            emit display_elapsed_time(timer.elapsed());
            return;
        }

        break;
    }


    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_knotvector_order(int value)
{
    QElapsedTimer timer;
    timer.start();

    RowMatrix<GLdouble> totalEnergies;
    switch (_current_running)
    {
    case CURVE:
        _bspline_curve_model->set_knotvector_order(value);
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->set_knotvector_order(value);
        totalEnergies = _regression_curve_model->get_energies();
        emit display_total_curvature(QString::number(totalEnergies[0]));
        emit display_length_of_curve(QString::number(totalEnergies[1]));
        emit display_kinetic_energy(QString::number(totalEnergies[2]));
        break;
    case PATCH:
        break;
    case SURFACEPOINTCLOUD:
        break;
    case ALL:
        if (_models->set_knotvector_order(value))
        {
            totalEnergies = _models->get_total_energies_of_selected_curve();
            emit display_total_curvature(QString::number(totalEnergies[0]));
            emit display_length_of_curve(QString::number(totalEnergies[1]));
            emit display_kinetic_energy(QString::number(totalEnergies[2]));
        }
        else
        {
            emit display_elapsed_time(timer.elapsed());
            return;
        }
        break;
    }

    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_control_points(int value)
{
    QElapsedTimer timer;
    timer.start();

    RowMatrix<GLdouble> totalEnergies;
    switch (_current_running)
    {
    case CURVE:
        _bspline_curve_model->set_control_points(value);
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->set_control_points(value);
        totalEnergies = _regression_curve_model->get_energies();
        emit display_total_curvature(QString::number(totalEnergies[0]));
        emit display_length_of_curve(QString::number(totalEnergies[1]));
        emit display_kinetic_energy(QString::number(totalEnergies[2]));
        break;
    case PATCH:
        break;
    case SURFACEPOINTCLOUD:
        break;
    case ALL:
        if (_models->set_control_points(value))
        {
            totalEnergies = _models->get_total_energies_of_selected_curve();
            emit display_total_curvature(QString::number(totalEnergies[0]));
            emit display_length_of_curve(QString::number(totalEnergies[1]));
            emit display_kinetic_energy(QString::number(totalEnergies[2]));
        }
        else
        {
            emit display_elapsed_time(timer.elapsed());
            return;
        }
        break;
    }

    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_div_point_coint(int value)
{
    QElapsedTimer timer;
    timer.start();

    RowMatrix<GLdouble> totalEnergies;
    switch (_current_running)
    {
    case CURVE:
        _bspline_curve_model->set_div_point_coint(value);
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->set_div_point_coint(value);
        totalEnergies = _regression_curve_model->get_energies();
        emit display_total_curvature(QString::number(totalEnergies[0]));
        emit display_length_of_curve(QString::number(totalEnergies[1]));
        emit display_kinetic_energy(QString::number(totalEnergies[2]));
        break;
    case PATCH:
        break;
    case SURFACEPOINTCLOUD:
        break;
    case ALL:
        if (_models->set_div_point_coint(value))
        {
            totalEnergies = _models->get_total_energies_of_selected_curve();
            emit display_total_curvature(QString::number(totalEnergies[0]));
            emit display_length_of_curve(QString::number(totalEnergies[1]));
            emit display_kinetic_energy(QString::number(totalEnergies[2]));
        }
        else
        {
            emit display_elapsed_time(timer.elapsed());
            return;
        }
        break;
    }

    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_scale_of_vectors(double value)
{
    switch (_current_running)
    {
    case CURVE:
         _bspline_curve_model->set_scale_of_vectors(value);
        break;
    case CURVEPOINTCLOUD:
         _regression_curve_model->set_scale_of_vectors(value);
        break;
    case PATCH:
        break;
    case SURFACEPOINTCLOUD:
        break;
    case ALL:
        if (!_models->set_scale_of_vectors(value))
            return;
        break;
    }

    update();
}

void GLWidget::set_scale_of_comb_vectors(double value)
{
    switch (_current_running)
    {
    case CURVE:
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->set_scale_of_comb_vectors(value);
        break;
    case PATCH:
        break;
    case SURFACEPOINTCLOUD:
        break;
    case ALL:
        if (!_models->set_scale_of_comb_vectors(value))
            return;
        break;
    }

    update();

}

void GLWidget::set_count_of_comb_vectors(int value)
{
    switch (_current_running)
    {
    case CURVE:
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->set_count_of_comb_vectors(value);
        break;
    case PATCH:
        break;
    case SURFACEPOINTCLOUD:
        break;
    case ALL:
        if (!_models->set_count_of_comb_vectors(value))
            return;
        break;
    }

    update();

}

//-----------------------------------
// patch setters
//-----------------------------------
void GLWidget::set_knotvector_type_u(int index)
{
    QElapsedTimer timer;
    timer.start();
    if (_current_running == PATCH){
        _bspline_surface_model->set_knotvector_type_u(index);
    }
    else if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->set_knotvector_type_u(index);
        RowMatrix<GLdouble> totalEnergies = _regression_surface_model->get_energies();
        emit(display_surface_area(QString::number(totalEnergies[1])));
        emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
        emit(display_mean_curvature(QString::number(totalEnergies[3])));
        emit(display_willmore_curvature(QString::number(totalEnergies[4])));
        emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
        emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
        emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
        emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
        emit(display_log_total_curvature(QString::number(totalEnergies[9])));
    }
    else if (_current_running == ALL)
    {
        if (_models->set_knotvector_type_u(index))
        {
            RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_surface();
            emit(display_surface_area(QString::number(totalEnergies[1])));
            emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
            emit(display_mean_curvature(QString::number(totalEnergies[3])));
            emit(display_willmore_curvature(QString::number(totalEnergies[4])));
            emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
            emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
            emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
            emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
            emit(display_log_total_curvature(QString::number(totalEnergies[9])));
        }
        else
        {
            emit display_elapsed_time(timer.elapsed());
            return;
        }
    }
    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_knotvector_type_v(int index)
{
    QElapsedTimer timer;
    timer.start();
    if (_current_running == PATCH){
        _bspline_surface_model->set_knotvector_type_v(index);
    }
    else if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->set_knotvector_type_v(index);
        RowMatrix<GLdouble> totalEnergies = _regression_surface_model->get_energies();
        emit(display_surface_area(QString::number(totalEnergies[1])));
        emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
        emit(display_mean_curvature(QString::number(totalEnergies[3])));
        emit(display_willmore_curvature(QString::number(totalEnergies[4])));
        emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
        emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
        emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
        emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
        emit(display_log_total_curvature(QString::number(totalEnergies[9])));
    }
    else if (_current_running == ALL)
    {
        if (_models->set_knotvector_type_v(index))
        {
            RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_surface();
            emit(display_surface_area(QString::number(totalEnergies[1])));
            emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
            emit(display_mean_curvature(QString::number(totalEnergies[3])));
            emit(display_willmore_curvature(QString::number(totalEnergies[4])));
            emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
            emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
            emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
            emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
            emit(display_log_total_curvature(QString::number(totalEnergies[9])));
        }
        else
        {
            emit display_elapsed_time(timer.elapsed());
            return;
        }
    }
    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_knotvector_order_u(int value)
{
    QElapsedTimer timer;
    timer.start();
    if  (_current_running == PATCH)
    {
        _bspline_surface_model->set_knotvector_order_u(value);
    }
    else if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->set_knotvector_order_u(value);
        RowMatrix<GLdouble> totalEnergies = _regression_surface_model->get_energies();
        emit(display_surface_area(QString::number(totalEnergies[1])));
        emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
        emit(display_mean_curvature(QString::number(totalEnergies[3])));
        emit(display_willmore_curvature(QString::number(totalEnergies[4])));
        emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
        emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
        emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
        emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
        emit(display_log_total_curvature(QString::number(totalEnergies[9])));
    }
    else if (_current_running == ALL)
    {
        if (_models->set_knotvector_order_u(value))
        {
            RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_surface();
            emit(display_surface_area(QString::number(totalEnergies[1])));
            emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
            emit(display_mean_curvature(QString::number(totalEnergies[3])));
            emit(display_willmore_curvature(QString::number(totalEnergies[4])));
            emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
            emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
            emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
            emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
            emit(display_log_total_curvature(QString::number(totalEnergies[9])));
        }
        else
        {
            emit display_elapsed_time(timer.elapsed());
            return;
        }
    }
    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_knotvector_order_v(int value)
{
    QElapsedTimer timer;
    timer.start();
    if  (_current_running == PATCH)
    {
        _bspline_surface_model->set_knotvector_order_v(value);
    }
    else if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->set_knotvector_order_v(value);
        RowMatrix<GLdouble> totalEnergies = _regression_surface_model->get_energies();
        emit(display_surface_area(QString::number(totalEnergies[1])));
        emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
        emit(display_mean_curvature(QString::number(totalEnergies[3])));
        emit(display_willmore_curvature(QString::number(totalEnergies[4])));
        emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
        emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
        emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
        emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
        emit(display_log_total_curvature(QString::number(totalEnergies[9])));
    }
    else if (_current_running == ALL)
    {
        if (_models->set_knotvector_order_v(value))
        {
            RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_surface();
            emit(display_surface_area(QString::number(totalEnergies[1])));
            emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
            emit(display_mean_curvature(QString::number(totalEnergies[3])));
            emit(display_willmore_curvature(QString::number(totalEnergies[4])));
            emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
            emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
            emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
            emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
            emit(display_log_total_curvature(QString::number(totalEnergies[9])));
        }
        else
        {
            emit display_elapsed_time(timer.elapsed());
            return;
        }
    }
    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_control_points_u(int value)
{
    QElapsedTimer timer;
    timer.start();
    if (_current_running == PATCH)
    {
        _bspline_surface_model->set_control_points_u(value);
    }
    else if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->set_control_points_u(value);
        RowMatrix<GLdouble> totalEnergies = _regression_surface_model->get_energies();
        emit(display_surface_area(QString::number(totalEnergies[1])));
        emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
        emit(display_mean_curvature(QString::number(totalEnergies[3])));
        emit(display_willmore_curvature(QString::number(totalEnergies[4])));
        emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
        emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
        emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
        emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
        emit(display_log_total_curvature(QString::number(totalEnergies[9])));
    }
    else if (_current_running == ALL)
    {
        if (_models->set_control_points_u(value))
        {
            RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_surface();
            emit(display_surface_area(QString::number(totalEnergies[1])));
            emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
            emit(display_mean_curvature(QString::number(totalEnergies[3])));
            emit(display_willmore_curvature(QString::number(totalEnergies[4])));
            emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
            emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
            emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
            emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
            emit(display_log_total_curvature(QString::number(totalEnergies[9])));
        }
        else
        {
            emit display_elapsed_time(timer.elapsed());
            return;
        }
    }
    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_control_points_v(int value)
{
    QElapsedTimer timer;
    timer.start();
    if (_current_running == PATCH)
    {
        _bspline_surface_model->set_control_points_v(value);
    }
    else if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->set_control_points_v(value);
        RowMatrix<GLdouble> totalEnergies = _regression_surface_model->get_energies();
        emit(display_surface_area(QString::number(totalEnergies[1])));
        emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
        emit(display_mean_curvature(QString::number(totalEnergies[3])));
        emit(display_willmore_curvature(QString::number(totalEnergies[4])));
        emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
        emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
        emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
        emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
        emit(display_log_total_curvature(QString::number(totalEnergies[9])));
    }
    else if (_current_running == ALL)
    {
        if (_models->set_control_points_v(value))
        {
            RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_surface();
            emit(display_surface_area(QString::number(totalEnergies[1])));
            emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
            emit(display_mean_curvature(QString::number(totalEnergies[3])));
            emit(display_willmore_curvature(QString::number(totalEnergies[4])));
            emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
            emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
            emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
            emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
            emit(display_log_total_curvature(QString::number(totalEnergies[9])));
        }
        else
        {
            emit display_elapsed_time(timer.elapsed());
            return;
        }
    }
    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_div_point_coint_u(int value)
{
    QElapsedTimer timer;
    timer.start();

    if (_current_running == PATCH)
    {
        _bspline_surface_model->set_div_point_coint_u(value);
    }
    else if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->set_div_point_coint_u(value);
    }
    else if (_current_running == ALL)
    {
        if (!_models->set_div_point_coint_u(value))
        {
            emit display_elapsed_time(timer.elapsed());
            return;
        }
    }
    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_div_point_coint_v(int value)
{
    QElapsedTimer timer;
    timer.start();
    if (_current_running == PATCH)
    {
        _bspline_surface_model->set_div_point_coint_v(value);
    }
    else if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->set_div_point_coint_v(value);
    }
    else if (_current_running == ALL)
    {
        if (!_models->set_div_point_coint_v(value))
        {
            emit display_elapsed_time(timer.elapsed());
            return;
        }
    }
    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_color_sheme(int index)
{
    QElapsedTimer timer;
    timer.start();
    if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->set_color_sheme(index);
    }
    else if (_current_running == ALL)
    {
        if (!_models->set_color_sheme(index))
        {
            emit display_elapsed_time(timer.elapsed());
            return;
        }
    }
    emit display_elapsed_time(timer.elapsed());
    update();
}

//-----------------------------------
// cloud setters
//-----------------------------------
void GLWidget::set_parametric_curve_type(int index)
{
    QElapsedTimer timer;
    timer.start();
    if (_current_running == CURVEPOINTCLOUD)
    {
        _regression_curve_model->set_parametric_curve_type(index);
        RowMatrix<GLdouble> totalEnergies = _regression_curve_model->get_energies();
        emit display_total_curvature(QString::number(totalEnergies[0]));
        emit display_length_of_curve(QString::number(totalEnergies[1]));
        emit display_kinetic_energy(QString::number(totalEnergies[2]));
    }

    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_parametric_surface_type(int index)
{
    QElapsedTimer timer;
    timer.start();
    if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->set_parametric_surface_type(index);
        RowMatrix<GLdouble> totalEnergies = _regression_surface_model->get_energies();
        emit(display_surface_area(QString::number(totalEnergies[1])));
        emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
        emit(display_mean_curvature(QString::number(totalEnergies[3])));
        emit(display_willmore_curvature(QString::number(totalEnergies[4])));
        emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
        emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
        emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
        emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
        emit(display_log_total_curvature(QString::number(totalEnergies[9])));
    }

    emit display_elapsed_time(timer.elapsed());
    update();
}

void GLWidget::set_sigma_x(double value)
{
    if (value != (*_sigma)[0])
    {
        (*_sigma)[0] = value;

        QElapsedTimer timer;
        timer.start();
        if (_current_running == CURVEPOINTCLOUD)
        {
            _regression_curve_model->set_sigma_x(value);
            RowMatrix<GLdouble> totalEnergies = _regression_curve_model->get_energies();
            emit display_total_curvature(QString::number(totalEnergies[0]));
            emit display_length_of_curve(QString::number(totalEnergies[1]));
            emit display_kinetic_energy(QString::number(totalEnergies[2]));
        }
        if (_current_running == SURFACEPOINTCLOUD)
        {
            _regression_surface_model->set_sigma_x(value);

            RowMatrix<GLdouble> totalEnergies = _regression_surface_model->get_energies();
            emit(display_surface_area(QString::number(totalEnergies[1])));
            emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
            emit(display_mean_curvature(QString::number(totalEnergies[3])));
            emit(display_willmore_curvature(QString::number(totalEnergies[4])));
            emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
            emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
            emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
            emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
            emit(display_log_total_curvature(QString::number(totalEnergies[9])));
        }
        emit display_elapsed_time(timer.elapsed());

        update();
    }
}

void GLWidget::set_sigma_y(double value)
{
    if (value != (*_sigma)[1])
    {
        (*_sigma)[1] = value;

        QElapsedTimer timer;
        timer.start();
        if (_current_running == CURVEPOINTCLOUD)
        {
            _regression_curve_model->set_sigma_y(value);
            RowMatrix<GLdouble> totalEnergies = _regression_curve_model->get_energies();
            emit display_total_curvature(QString::number(totalEnergies[0]));
            emit display_length_of_curve(QString::number(totalEnergies[1]));
            emit display_kinetic_energy(QString::number(totalEnergies[2]));
        }
        if (_current_running == SURFACEPOINTCLOUD)
        {
            _regression_surface_model->set_sigma_y(value);

            RowMatrix<GLdouble> totalEnergies = _regression_surface_model->get_energies();
            emit(display_surface_area(QString::number(totalEnergies[1])));
            emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
            emit(display_mean_curvature(QString::number(totalEnergies[3])));
            emit(display_willmore_curvature(QString::number(totalEnergies[4])));
            emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
            emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
            emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
            emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
            emit(display_log_total_curvature(QString::number(totalEnergies[9])));
        }
        emit display_elapsed_time(timer.elapsed());
        update();
    }
}

void GLWidget::set_sigma_z(double value)
{
    if (value != (*_sigma)[2])
    {
        (*_sigma)[2] = value;

        QElapsedTimer timer;
        timer.start();
        if (_current_running == CURVEPOINTCLOUD)
        {
            _regression_curve_model->set_sigma_z(value);
            RowMatrix<GLdouble> totalEnergies = _regression_curve_model->get_energies();
            emit display_total_curvature(QString::number(totalEnergies[0]));
            emit display_length_of_curve(QString::number(totalEnergies[1]));
            emit display_kinetic_energy(QString::number(totalEnergies[2]));
        }
        if (_current_running == SURFACEPOINTCLOUD)
        {
            _regression_surface_model->set_sigma_z(value);

            RowMatrix<GLdouble> totalEnergies = _regression_surface_model->get_energies();
            emit(display_surface_area(QString::number(totalEnergies[1])));
            emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
            emit(display_mean_curvature(QString::number(totalEnergies[3])));
            emit(display_willmore_curvature(QString::number(totalEnergies[4])));
            emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
            emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
            emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
            emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
            emit(display_log_total_curvature(QString::number(totalEnergies[9])));
        }
        emit display_elapsed_time(timer.elapsed());

        update();
    }
}

void GLWidget::set_weight_size(int value)
{
    if (value != _weight.GetColumnCount())
    {
        if (static_cast<int> (_selected_weight_index) > value)
            emit display_index_weight(0.0);
        _weight.ResizeColumns(value);
        if (_current_running == CURVEPOINTCLOUD)
        {
            _regression_curve_model->set_weight_size(value);
        }
        if (_current_running == SURFACEPOINTCLOUD)
        {
            _regression_surface_model->set_weight_size(value);
        }
        if (_current_running == ALL)
        {
            if (!_models->set_weight_size(value))
            {
                return;
            }
        }
        update();
    }
}

void GLWidget::set_weight_index(int value)
{
    if (value == 0)
    {
        _selected_weight_index = 0;
        emit display_index_weight(0.0);
        return;
    }
    if (value == static_cast<int>(_selected_weight_index))
        return;
    if (value <= static_cast<int>(_weight.GetColumnCount()))
    {
        _selected_weight_index = value;
        emit display_index_weight(_weight[value - 1]);
        update();
    }
    else
    {
        emit display_index_weight(0.0);
    }
}

void GLWidget::set_weight_value(double value)
{
    if (_selected_weight_index == 0)
        return;
    if (_weight[_selected_weight_index - 1] != value)
    {
        _weight[_selected_weight_index - 1] = value;

        QElapsedTimer timer;
        timer.start();

        emit optimization_done(false);

        if(_current_running == CURVEPOINTCLOUD)
        {
            _regression_curve_model->set_weight_value(_selected_weight_index - 1, value);
            RowMatrix<GLdouble> totalEnergies = _regression_curve_model->get_energies();
            emit display_total_curvature(QString::number(totalEnergies[0]));
            emit display_length_of_curve(QString::number(totalEnergies[1]));
            emit display_kinetic_energy(QString::number(totalEnergies[2]));
        }

        if (_current_running == SURFACEPOINTCLOUD)
        {
            _regression_surface_model->set_weight_value(_selected_weight_index - 1, value);

            RowMatrix<GLdouble> totalEnergies = _regression_surface_model->get_energies();
            emit(display_surface_area(QString::number(totalEnergies[1])));
            emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
            emit(display_mean_curvature(QString::number(totalEnergies[3])));
            emit(display_willmore_curvature(QString::number(totalEnergies[4])));
            emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
            emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
            emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
            emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
            emit(display_log_total_curvature(QString::number(totalEnergies[9])));
        }

        if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::ONE_VARIABLE)
        {
            _models->set_weight_value(_selected_weight_index - 1, value);
            RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_selected_curve();
            emit display_total_curvature(QString::number(totalEnergies[0]));
            emit display_length_of_curve(QString::number(totalEnergies[1]));
            emit display_kinetic_energy(QString::number(totalEnergies[2]));
        }

        if (_current_running == ALL && _models->_selected_type == PointCloudsAndModels::TWO_VARIABLE)
        {
            _models->set_weight_value(_selected_weight_index - 1, value);
            RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_surface();
            emit(display_surface_area(QString::number(totalEnergies[1])));
            emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
            emit(display_mean_curvature(QString::number(totalEnergies[3])));
            emit(display_willmore_curvature(QString::number(totalEnergies[4])));
            emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
            emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
            emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
            emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
            emit(display_log_total_curvature(QString::number(totalEnergies[9])));
        }

        emit optimization_done(true);
        emit set_focus();

        emit display_elapsed_time(timer.elapsed());

        update();
    }
}

void GLWidget::set_cloud_size(int value)
{

    QElapsedTimer timer;
    timer.start();
    if (_current_running == CURVEPOINTCLOUD)
    {
        _regression_curve_model->set_cloud_size(value);
        RowMatrix<GLdouble> totalEnergies = _regression_curve_model->get_energies();
        emit display_total_curvature(QString::number(totalEnergies[0]));
        emit display_length_of_curve(QString::number(totalEnergies[1]));
        emit display_kinetic_energy(QString::number(totalEnergies[2]));
    }

    emit display_elapsed_time(timer.elapsed());

    update();

}

void GLWidget::set_cloud_size_in_u_direction(int value)
{

    QElapsedTimer timer;
    timer.start();
    if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->set_cloud_size_in_u_direction(value);

        RowMatrix<GLdouble> totalEnergies = _regression_surface_model->get_energies();
        emit(display_surface_area(QString::number(totalEnergies[1])));
        emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
        emit(display_mean_curvature(QString::number(totalEnergies[3])));
        emit(display_willmore_curvature(QString::number(totalEnergies[4])));
        emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
        emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
        emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
        emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
        emit(display_log_total_curvature(QString::number(totalEnergies[9])));
    }

    emit display_elapsed_time(timer.elapsed());

    update();

}

void GLWidget::set_cloud_size_in_v_direction(int value)
{

    QElapsedTimer timer;
    timer.start();
    if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->set_cloud_size_in_v_direction(value);

        RowMatrix<GLdouble> totalEnergies = _regression_surface_model->get_energies();
        emit(display_surface_area(QString::number(totalEnergies[1])));
        emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
        emit(display_mean_curvature(QString::number(totalEnergies[3])));
        emit(display_willmore_curvature(QString::number(totalEnergies[4])));
        emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
        emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
        emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
        emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
        emit(display_log_total_curvature(QString::number(totalEnergies[9])));
    }

    emit display_elapsed_time(timer.elapsed());

    update();

}

//-----------------------------------
// show/hide
//-----------------------------------
void GLWidget::show_control_polygon(bool value)
{
    switch (_current_running)
    {
    case CURVE:
        _bspline_curve_model->show_control_polygon(value);
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->show_control_polygon(value);
        break;
    case PATCH:
        _bspline_surface_model->show_control_polygon(value);
        break;
    case SURFACEPOINTCLOUD:
        _regression_surface_model->show_control_polygon(value);
        break;
    case ALL:
        if (!_models->show_control_polygon(value))
            return;
        break;
    }

    update();

}

void GLWidget::show_cloud(bool value)
{
    switch (_current_running)
    {
    case CURVE:
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->show_cloud(value);
        break;
    case PATCH:
        break;
    case SURFACEPOINTCLOUD:
        _regression_surface_model->show_cloud(value);
        break;
    case ALL:
        if (!_models->show_cloud(value))
            return;
        break;
    }
    update();

}

void GLWidget::show_curve(bool value)
{
    switch (_current_running)
    {
    case CURVE:
        _bspline_curve_model->show_curve(value);
        break;
    case CURVEPOINTCLOUD:
         _regression_curve_model->show_curve(value);
        break;
    case PATCH:
        break;
    case SURFACEPOINTCLOUD:
        break;
    case ALL:
        if (!_models->show_curve(value))
            return;
        break;
    }

    update();
}

void GLWidget::show_arcs(bool value)
{
    switch (_current_running)
    {
    case CURVE:
        _bspline_curve_model->show_arcs(value);
        break;
    case CURVEPOINTCLOUD:
         _regression_curve_model->show_arcs(value);
        break;
    case PATCH:
        break;
    case SURFACEPOINTCLOUD:
        break;
    case ALL:
        if (!_models->show_arcs(value))
            return;
        break;
    }

    update();
}

void GLWidget::show_tangents(bool value)
{
    switch (_current_running)
    {
    case CURVE:
         _bspline_curve_model->show_tangents(value);
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->show_tangents(value);
        break;
    case PATCH:
        break;
    case SURFACEPOINTCLOUD:
        break;
    case ALL:
        if (!_models->show_tangents(value))
            return;
        break;
    }

    update();

}

void GLWidget::show_acceleration_vectors(bool value)
{
    switch (_current_running)
    {
    case CURVE:
        _bspline_curve_model->show_acceleration_vectors(value);
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->show_acceleration_vectors(value);
        break;
    case PATCH:
        break;
    case SURFACEPOINTCLOUD:
        break;
    case ALL:
        if (!_models->show_acceleration_vectors(value))
            return;
        break;
    }

    update();

}

void GLWidget::show_patches(bool value)
{
    if (_current_running == PATCH)
    {
        _bspline_surface_model->show_patches(value);
    }
    if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->show_patches(value);
    }
    if (_current_running == ALL && !_models->show_patches(value))
    {
        return;
    }
    update();
}

void GLWidget::show_patch(bool value)
{
    if (_current_running == PATCH)
    {
        _bspline_surface_model->show_patch(value);
    }
    if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->show_patch(value);
    }
    if (_current_running == ALL && !_models->show_patch(value))
    {
        return;
    }
    update();
}

void GLWidget::show_comb(bool value)
{
    if (_current_running == CURVEPOINTCLOUD)
    {
        _regression_curve_model->show_comb(value);
    }
    else if (_current_running == ALL && !_models->show_comb(value))
    {
        return;
    }
    update();
}

void GLWidget::show_heat_map(bool value)
{
    if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->show_heat_map(value);
    }
    if (_current_running == ALL && !_models->show_heat_map(value))
    {
        return;
    }
    update();
}

//-----------------------------------
// other
//-----------------------------------
void GLWidget::set_control_point_size(double value)
{
    switch (_current_running)
    {
    case CURVE:
        _bspline_curve_model->set_control_point_size(value);
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->set_control_point_size(value);
        break;
    case PATCH:
        _bspline_surface_model->set_control_point_size(value);
        break;
    case SURFACEPOINTCLOUD:
        _regression_surface_model->set_control_point_size(value);
        break;
    case ALL:
        if (!_models->set_control_point_size(value))
        {
            return;
        }
        break;
    }

    update();

}

void GLWidget::set_cloud_point_size(double value)
{
    switch (_current_running)
    {
    case CURVE:
        break;
    case CURVEPOINTCLOUD:
        _regression_curve_model->set_cloud_point_size(value);
        break;
    case PATCH:
        break;
    case SURFACEPOINTCLOUD:
        _regression_surface_model->set_cloud_point_size(value);
        break;
    case ALL:
        if (!_models->set_cloud_point_size(value))
        {
            return;
        }
        break;
    }

    update();

}

void GLWidget::set_dark_light_mode(bool value)
{
    makeCurrent();

    if (_dark_mode != value)
    {
        _dark_mode = value;
        if (!_dark_mode)
        {
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        }
        else
        {
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        }
        update();
    }
}

void GLWidget::save_point_cloud_around_curve()
{
    string text = "Curve point cloud (*.1v)";
    if (_current_running != CURVEPOINTCLOUD && _current_running != ALL)
    {
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save one variable point cloud"), "Point clouds",
                                                    tr(text.c_str()));

    QFile file(fileName);

    if (!file.open(QIODevice::WriteOnly))
    {
        QMessageBox::information(this, tr("Unable to open file"),
                                 file.errorString());
        return;
    }

    QTextStream out(&file);
    if (_current_running == CURVEPOINTCLOUD)
        out << (*_regression_curve_model);
    if (_current_running == ALL)
        _models->saveOneVariablePointCloud(out);
    file.flush();
    file.close();
}

void GLWidget::load_point_cloud_around_curve()
{
    string text = "";
    if (_current_running == CURVEPOINTCLOUD)
    {
        text = "Curve point cloud (*.1v)";
    }
    else
    {
        text = "Surface point cloud (*.2v)";
    }
    if (_current_running == ALL)
    {
        text = "Curve point cloud (*.1v)";
    }

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Load one variable point cloud"), "Point clouds",
                                                    tr(text.c_str()));

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::information(this, tr("Unable to open file"),
                                 file.errorString());
        return;
    }

    QTextStream in(&file);

    if (in.atEnd())
    {
        QMessageBox::information(this, tr("Empty file"),
                                 tr("The file is empty!"));
        return;
    }

    if (_current_running == CURVEPOINTCLOUD)
    {
        in >> (*_regression_curve_model);
    }
    else if (_current_running == SURFACEPOINTCLOUD)
    {
        in >> (*_regression_surface_model);
    }
    else if (_current_running == ALL)
    {
        QElapsedTimer timer;
        timer.start();

        _selected_weight_index = 0;
        emit display_index_of_derivativ(0);

        _models->loadOneVariablePointCloud(in);

        RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_selected_curve();
        emit display_total_curvature(QString::number(totalEnergies[0]));
        emit display_length_of_curve(QString::number(totalEnergies[1]));
        emit display_kinetic_energy(QString::number(totalEnergies[2]));

        emit display_surface_area("0.0");
        emit display_gaussian_curvature("0.0");
        emit display_mean_curvature("0.0");
        emit display_willmore_curvature("0.0");
        emit display_log_willmore_curvature("0.0");
        emit display_umbilic_deviation("0.0");
        emit display_log_umbilic_deviation("0.0");
        emit display_total_curvature_surface("0.0");
        emit display_log_total_curvature("0.0");

        emit set_displayed_knotvector_type(_models->get_knotvector_type());
        emit set_displayed_knotvector_order(_models->get_knotvector_order());
        emit set_displayed_control_points(_models->get_control_points());
        emit set_displayed_div_point_coint(_models->get_div_point_coint());

        _weight.ResizeColumns(0);
        emit display_max_derivativ_num(0);

        emit mask_has_changed(0b0001);


        if (!_global_transformation_selected)
        {
            emit display_trans_x(_models->get_trans_x());
            emit display_trans_y(_models->get_trans_y());
            emit display_trans_z(_models->get_trans_z());

            emit display_scale(_models->get_scale_factor());

            emit display_angle_x(_models->get_angle_x());
            emit display_angle_y(_models->get_angle_y());
            emit display_angle_z(_models->get_angle_z());
        }
        emit display_elapsed_time(timer.elapsed());
    }

    file.close();
    update();
}

void GLWidget::save_point_cloud_around_surface()
{
    string text = "Surface point cloud (*.2v)";
    if (_current_running != SURFACEPOINTCLOUD && _current_running != ALL)
    {
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save two variable point cloud"), "Point clouds",
                                                    tr(text.c_str()));

    QFile file(fileName);

    if (!file.open(QIODevice::WriteOnly))
    {
        QMessageBox::information(this, tr("Unable to open file"),
                                 file.errorString());
        return;
    }
    QTextStream out(&file);
    if (_current_running == SURFACEPOINTCLOUD)
        out << (*_regression_surface_model);
    if (_current_running == ALL)
        _models->saveTwoVariablePointCloud(out);

    file.flush();
    file.close();
}

void GLWidget::load_point_cloud_around_surface()
{
    string text = "";
    if (_current_running == CURVEPOINTCLOUD)
    {
        text = "Curve point cloud (*.1v)";
    }
    else
    {
        text = "Surface point cloud (*.2v)";
    }
    if (_current_running == ALL)
    {
        text = "Surface point cloud (*.2v)";
    }
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Load two variable point cloud"), "Point clouds",
                                                    tr(text.c_str()));

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::information(this, tr("Unable to open file"),
                                 file.errorString());
        return;
    }

    QTextStream in(&file);

    if (in.atEnd())
    {
        QMessageBox::information(this, tr("Empty file"),
                                 tr("The file is empty!"));
        return;
    }

    if (_current_running == CURVEPOINTCLOUD)
    {
        in >> (*_regression_curve_model);
    }
    else if (_current_running == SURFACEPOINTCLOUD)
    {
        in >> (*_regression_surface_model);
    }
    else if (_current_running == ALL)
    {
        QElapsedTimer timer;
        timer.start();

        _selected_weight_index = 0;
        emit display_index_of_derivativ(0);

        _models->loadTwoVariablePointCloud(in);

        RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_surface();
        emit(display_surface_area(QString::number(totalEnergies[1])));
        emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
        emit(display_mean_curvature(QString::number(totalEnergies[3])));
        emit(display_willmore_curvature(QString::number(totalEnergies[4])));
        emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
        emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
        emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
        emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
        emit(display_log_total_curvature(QString::number(totalEnergies[9])));

        emit display_total_curvature("0.0");
        emit display_length_of_curve("0.0");
        emit display_kinetic_energy("0.0");

        emit set_displayed_knotvector_type_u(_models->get_knotvector_type_u());
        emit set_displayed_knotvector_type_v(_models->get_knotvector_type_v());
        emit set_displayed_knotvector_order_u(_models->get_knotvector_order_u());
        emit set_displayed_knotvector_order_v(_models->get_knotvector_order_v());
        emit set_displayed_control_points_u(_models->get_control_points_u());
        emit set_displayed_control_points_v(_models->get_control_points_v());
        emit set_displayed_div_point_coint_u(_models->get_div_point_coint_u());
        emit set_displayed_div_point_coint_v(_models->get_div_point_coint_v());

        _weight.ResizeColumns(0);
        emit display_max_derivativ_num(0);

        emit mask_has_changed(0b0010);


        if (!_global_transformation_selected)
        {
            emit display_trans_x(_models->get_trans_x());
            emit display_trans_y(_models->get_trans_y());
            emit display_trans_z(_models->get_trans_z());

            emit display_scale(_models->get_scale_factor());

            emit display_angle_x(_models->get_angle_x());
            emit display_angle_y(_models->get_angle_y());
            emit display_angle_z(_models->get_angle_z());
        }

        emit display_elapsed_time(timer.elapsed());
    }
    file.close();
    update();
}

void GLWidget::save_bspline_curve()
{
    string text = "B-spline curve (*.bc)";
    if (_current_running != CURVE && _current_running != ALL)
    {
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save B-spline curve"), "B-spline models",
                                                    tr(text.c_str()));

    QFile file(fileName);

    if (!file.open(QIODevice::WriteOnly))
    {
        QMessageBox::information(this, tr("Unable to open file"),
                                 file.errorString());
        return;
    }

    QTextStream out(&file);
    if (_current_running == CURVE)
    {
        out << (*_bspline_curve_model);
    }
    if (_current_running == ALL)
    {
        _models->saveBSplineCurve(out);
    }


    file.flush();
    file.close();
}

void GLWidget::load_bspline_curve()
{
    string text = "B-spline curve (*.bc)";
    if (_current_running != CURVE && _current_running != ALL)
    {
        return;
    }

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Save B-spline curve"), "B-spline models",
                                                    tr(text.c_str()));

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::information(this, tr("Unable to open file"),
                                 file.errorString());
        return;
    }

    QTextStream in(&file);

    if (in.atEnd())
    {
        QMessageBox::information(this, tr("Empty file"),
                                 tr("The file is empty!"));
        return;
    }

    if (_current_running == CURVE)
    {
        in >> (*_bspline_curve_model);
    }

    if (_current_running == ALL)
    {
        QElapsedTimer timer;
        timer.start();

        _selected_weight_index = 0;
        emit display_index_of_derivativ(0);

        _models->loadBSplineCurve(in);

        RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_selected_curve();
        emit display_total_curvature(QString::number(totalEnergies[0]));
        emit display_length_of_curve(QString::number(totalEnergies[1]));
        emit display_kinetic_energy(QString::number(totalEnergies[2]));

        emit display_surface_area("0.0");
        emit display_gaussian_curvature("0.0");
        emit display_mean_curvature("0.0");
        emit display_willmore_curvature("0.0");
        emit display_log_willmore_curvature("0.0");
        emit display_umbilic_deviation("0.0");
        emit display_log_umbilic_deviation("0.0");
        emit display_total_curvature_surface("0.0");
        emit display_log_total_curvature("0.0");

        _weight.ResizeColumns(0);
        emit display_max_derivativ_num(0);
        emit set_displayed_div_point_coint(_models->get_div_point_coint());

        emit mask_has_changed(0b0100);


        if (!_global_transformation_selected)
        {
            emit display_trans_x(_models->get_trans_x());
            emit display_trans_y(_models->get_trans_y());
            emit display_trans_z(_models->get_trans_z());

            emit display_scale(_models->get_scale_factor());

            emit display_angle_x(_models->get_angle_x());
            emit display_angle_y(_models->get_angle_y());
            emit display_angle_z(_models->get_angle_z());
        }
        emit display_elapsed_time(timer.elapsed());
    }

    file.close();
    update();
}

void GLWidget::save_bspline_surface()
{
    string text = "B-spline surface (*.bs)";
    if (_current_running != PATCH && _current_running != SURFACEPOINTCLOUD && _current_running != ALL)
    {
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save B-spline surface"), "B-spline models",
                                                    tr(text.c_str()));

    QFile file(fileName);

    if (!file.open(QIODevice::WriteOnly))
    {
        QMessageBox::information(this, tr("Unable to open file"),
                                 file.errorString());
        return;
    }

    QTextStream out(&file);
    if (_current_running == PATCH)
    {
        out << (*_bspline_surface_model);
    }
    if (_current_running == SURFACEPOINTCLOUD)
    {
        _regression_surface_model->OutSurface(out);
    }
    if (_current_running == ALL)
    {
        _models->saveBSplineSurface(out);
    }

    file.flush();
    file.close();
}

void GLWidget::load_bspline_surface()
{
    string text = "B-spline surface (*.bs)";
    if (_current_running != PATCH  && _current_running != ALL)
    {
        return;
    }

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Save B-spline surface"), "B-spline models",
                                                    tr(text.c_str()));

    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly))
    {
        QMessageBox::information(this, tr("Unable to open file"),
                                 file.errorString());
        return;
    }

    QTextStream in(&file);

    if (in.atEnd())
    {
        QMessageBox::information(this, tr("Empty file"),
                                 tr("The file is empty!"));
        return;
    }

    if (_current_running == PATCH)
    {
        in >> (*_bspline_surface_model);
    }

    if (_current_running == ALL)
    {
        QElapsedTimer timer;
        timer.start();

        _selected_weight_index = 0;
        emit display_index_of_derivativ(0);

        _models->loadBSplineSurface(in);

        RowMatrix<GLdouble> totalEnergies = _models->get_total_energies_of_surface();
        emit(display_surface_area(QString::number(totalEnergies[1])));
        emit(display_gaussian_curvature(QString::number(totalEnergies[2])));
        emit(display_mean_curvature(QString::number(totalEnergies[3])));
        emit(display_willmore_curvature(QString::number(totalEnergies[4])));
        emit(display_log_willmore_curvature(QString::number(totalEnergies[5])));
        emit(display_umbilic_deviation(QString::number(totalEnergies[6])));
        emit(display_log_umbilic_deviation(QString::number(totalEnergies[7])));
        emit(display_total_curvature_surface(QString::number(totalEnergies[8])));
        emit(display_log_total_curvature(QString::number(totalEnergies[9])));

        emit display_total_curvature("0.0");
        emit display_length_of_curve("0.0");
        emit display_kinetic_energy("0.0");

        _weight.ResizeColumns(0);
        emit display_max_derivativ_num(0);

        emit mask_has_changed(0b1000);


        if (!_global_transformation_selected)
        {
            emit display_trans_x(_models->get_trans_x());
            emit display_trans_y(_models->get_trans_y());
            emit display_trans_z(_models->get_trans_z());

            emit display_scale(_models->get_scale_factor());

            emit display_angle_x(_models->get_angle_x());
            emit display_angle_y(_models->get_angle_y());
            emit display_angle_z(_models->get_angle_z());
        }
        emit display_elapsed_time(timer.elapsed());
    }

    file.close();
    update();
}

}
