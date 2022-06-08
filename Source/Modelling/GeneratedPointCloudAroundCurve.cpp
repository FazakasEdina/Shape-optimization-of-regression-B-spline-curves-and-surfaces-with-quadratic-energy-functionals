#include "GeneratedPointCloudAroundCurve.h"

namespace cagd
{
GeneratedPointCloudAroundCurve::GeneratedPointCloudAroundCurve():ClassicBSplineCurve3()
{
    _weight.ResizeColumns(1);
    _weight[0] = 0.0;

    _sigma = new (nothrow) RowMatrix<GLdouble>(3);
    (*_sigma)[0] = (GLdouble)0.5;
    (*_sigma)[1] = (GLdouble)0.5;
    (*_sigma)[2] = (GLdouble)0.5;

}

bool GeneratedPointCloudAroundCurve::createParametricCurve()
{
    RowMatrix<ParametricCurve3::Derivative> d(3);
    switch(_selected_pc)
    {
    case 0:
        d[0] = circle::d0;
        d[1] = circle::d1;
        d[2] = circle::d2;

        _curve_u_min = circle::u_min;
        _curve_u_max = circle::u_max;
        break;
    case 1:
        d[0] = spiral_on_cone::d0;
        d[1] = spiral_on_cone::d1;
        d[2] = spiral_on_cone::d2;

        _curve_u_min = spiral_on_cone::u_min;
        _curve_u_max = spiral_on_cone::u_max;
        break;
    case 2:
        d[0] = spiral_on_cylinder::d0;
        d[1] = spiral_on_cylinder::d1;
        d[2] = spiral_on_cylinder::d2;

        _curve_u_min = spiral_on_cylinder::u_min;
        _curve_u_max = spiral_on_cylinder::u_max;
        break;
    case 3:
        d[0] = hypotrochoid_curve::d0;
        d[1] = hypotrochoid_curve::d1;
        d[2] = hypotrochoid_curve::d2;

        _curve_u_min = hypotrochoid_curve::u_min;
        _curve_u_max = hypotrochoid_curve::u_max;
        break;
    case 4:
        d[0] = vivians_curve::d0;
        d[1] = vivians_curve::d1;
        d[2] = vivians_curve::d2;

        _curve_u_min = vivians_curve::u_min;
        _curve_u_max = vivians_curve::u_max;
        break;
    case 5:
        d[0] = rose_curve::d0;
        d[1] = rose_curve::d1;
        d[2] = rose_curve::d2;

        _curve_u_min = rose_curve::u_min;
        _curve_u_max = rose_curve::u_max;
        break;
    case 6:
        d[0] = epitrochoid::d0;
        d[1] = epitrochoid::d1;
        d[2] = epitrochoid::d2;

        _curve_u_min = epitrochoid::u_min;
        _curve_u_max = epitrochoid::u_max;
        break;
    }

    _pc = new (nothrow) ParametricCurve3(d, _curve_u_min, _curve_u_max);
    if (!_pc)
    {
        deleteParametricCurve();
        return false;
    }

    _img_of_pc = _pc->GenerateImage(_div_point_count);
    if (!_img_of_pc || !_img_of_pc->UpdateVertexBufferObjects(_scale_of_vectors))
    {
        deleteParametricCurve();
        return false;
    }

    return true;
}

bool GeneratedPointCloudAroundCurve::createPointCloudAroundCurve()
{
    _curve_cloud = new (nothrow) PointCloudAroundCurve3();
    if (!_curve_cloud)
    {
        deletePointCloudAroundCurve();
        deleteParametricCurve();
        throw Exception("Could not create point cloud!");
    }

    _curve_cloud->GeneratePointCloudAroundParametricCurve(*_pc, (*_sigma), _point_cloud_size);
    return true;
}

bool GeneratedPointCloudAroundCurve::createBSplineCurve()
{
    _bs = _curve_cloud->GenerateRegressionCurve(_type, _k, _n, _weight, _curve_u_min, _curve_u_max);
    _total_energies = _bs->TotalEnergies(300);
    // cout << totalEnergies[3] << endl << totalEnergies[4] << endl;

    if (!_bs)
    {
        deleteAllObject();
        throw Exception("Could not create the bspline curve");
    }

    if (!_bs->UpdateVertexBufferObjectsOfData())
    {
        deleteAllObject();
        throw Exception("Could not update the VBO's pff the bspline curve's control polygon");
    }

    _img_bs = _bs->GenerateImage(2, _div_point_count);

    if (!_img_bs)
    {
        deleteAllObject();
        throw Exception("Could not create image of bspline curve");
    }

    if (!_img_bs->UpdateVertexBufferObjects(_scale_of_vectors))
    {
        deleteAllObject();
        throw Exception("Could not update the VBO's of the bspline curve's imgae!");
    }

    _arcs = _bs->GenerateImageOfArcs(2, _div_point_count);

    if (!_arcs)
    {
        deleteAllObject();
        throw Exception("Could not generate the arcs of the B-spline curve!");
    }

    for (GLuint i = 0; i < _arcs->GetColumnCount(); i++)
    {
        if (!(*_arcs)[i]->UpdateVertexBufferObjects(_scale_of_vectors))
        {
            deleteAllObject();
            throw Exception("Could not update the VBO of all arcs of the B-spline curve!");
        }
    }
    return true;
}

bool GeneratedPointCloudAroundCurve::renderPointCloudAroundCurve(bool dark_mode)
{
    if (!_bs)
    {
        return false;
    }

    glLineWidth(2);
    if (_show_comb)
    {
        glColor3f(0.6f, 1.0f, 1.0f);
        _bs->RenderCurvatureComb(_count_of_comb_vectors, _scale_of_comb_vectors);
    }

    if (_show_control_polygon)
    {
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_NORMALIZE);
        for (GLuint i = 0; i <= _n; i++)
        {
            DCoordinate3 &point = (*_bs)[i];

            // provide a unique name (i.e., identifier) for each sphere
            glLoadName(i);

            glPushMatrix();
                glTranslated(point[0], point[1], point[2]);
                glScalef(_point_size, _point_size, _point_size);
                MatFBTurquoise.Apply();
                _unit_sphere.Render();
            glPopMatrix();
        }

        glDisable(GL_LIGHTING);

        glColor3f(0.6f, 0.0f, 0.6f);
        _bs->RenderData(_type == KnotVector::PERIODIC ? GL_LINE_LOOP : GL_LINE_STRIP);
    }

    if (_show_cloud)
    {
        _curve_cloud->RenderPointCloud(&_unit_sphere, _cloud_point_size, dark_mode);
    }

    if (_show_curve)
    {
        glColor3d(0.0, 1.0, 0.0);
        glLineWidth(2);
        _img_bs->RenderDerivatives(0, GL_LINE_STRIP);
    }

    if (_show_tangents)
    {
        glColor3d(0.0, 1.0, 0.0);
        _img_bs->RenderDerivatives(1, GL_LINES);
    }

    if (_show_acceleration_vectors)
    {
        glColor3d(0.0, 1.0, 1.0);
        _img_bs->RenderDerivatives(2, GL_LINES);
    }

    if (_show_arcs)
    {
        if (!_arcs)
        {
            return false;
        }

        for (GLuint i = 0; i < _arcs->GetColumnCount(); i++)
        {
            if (!(*_arcs)[i])
            {
                return false;
            }

            if (i % 2)
            {
                glColor3d(1.0, 0.0, 0.0);
            }
            else
            {
                if (dark_mode)
                {
                    glColor3d(1.0, 0.8, 0.0);
                }
                else
                {
                    glColor3d(0.0, 1.0, 0.0);
                }
            }

            (*_arcs)[i]->RenderDerivatives(0, GL_LINE_STRIP);
        }
    }

    return true;
}

void GeneratedPointCloudAroundCurve::deleteParametricCurve()
{
    if (_pc)
    {
        delete _pc;
        _pc = nullptr;
    }

    if(_img_of_pc)
    {
        delete _img_of_pc;
        _img_of_pc = nullptr;
    }
}

void GeneratedPointCloudAroundCurve::deletePointCloudAroundCurve()
{
    if (_curve_cloud)
    {
        delete _curve_cloud;
        _curve_cloud = nullptr;
    }
}

void GeneratedPointCloudAroundCurve::deleteAllObject()
{
    deleteParametricCurve();
    deletePointCloudAroundCurve();
    deleteBSplineCurve();
}

void GeneratedPointCloudAroundCurve::deleteBSplineCurve()
{
    ClassicBSplineCurve3::deleteBSplineCurve();
}

void GeneratedPointCloudAroundCurve::updateCurveByOnePoint(int index)
{
    ClassicBSplineCurve3::updateCurveByOnePoint(index);
    _total_energies = _bs->TotalEnergies(300);
}

bool GeneratedPointCloudAroundCurve::set_control_points(int value)
{
    return ClassicBSplineCurve3::set_control_points(value);
}

bool GeneratedPointCloudAroundCurve::set_knotvector_type(int index)
{
    return ClassicBSplineCurve3::set_knotvector_type(index);
}

bool GeneratedPointCloudAroundCurve::set_knotvector_order(int value)
{
    return ClassicBSplineCurve3::set_knotvector_order(value);
}

bool GeneratedPointCloudAroundCurve::set_div_point_coint(int value)
{
    return ClassicBSplineCurve3::set_div_point_coint(value);
}

bool GeneratedPointCloudAroundCurve::set_scale_of_vectors(double value)
{
    return ClassicBSplineCurve3::set_scale_of_vectors(value);
}

bool GeneratedPointCloudAroundCurve::set_control_point_size(double value)
{
    return ClassicBSplineCurve3::set_control_point_size(value);
}

bool GeneratedPointCloudAroundCurve::show_control_polygon(bool value)
{
    return ClassicBSplineCurve3::show_control_polygon(value);
}

bool GeneratedPointCloudAroundCurve::show_curve(bool value)
{
    return ClassicBSplineCurve3::show_curve(value);
}

bool GeneratedPointCloudAroundCurve::show_arcs(bool value)
{
    return ClassicBSplineCurve3::show_arcs(value);
}

bool GeneratedPointCloudAroundCurve::show_tangents(bool value)
{
    return ClassicBSplineCurve3::show_tangents(value);
}

bool GeneratedPointCloudAroundCurve::show_acceleration_vectors(bool value)
{
    return ClassicBSplineCurve3::show_acceleration_vectors(value);
}

GLuint GeneratedPointCloudAroundCurve::get_n()
{
    return ClassicBSplineCurve3::get_n();
}

DCoordinate3* GeneratedPointCloudAroundCurve::get_control_point(int index)
{
    return ClassicBSplineCurve3::get_control_point(index);
}

// ---------------------------------------
RowMatrix<GLdouble> GeneratedPointCloudAroundCurve::get_energies()
{
    return _total_energies;
}

bool GeneratedPointCloudAroundCurve::set_parametric_curve_type(int index)
{
    if (index == _selected_pc)
        return false;

    _selected_pc = index;
    deleteAllObject();

    createParametricCurve();
    createPointCloudAroundCurve();
    createBSplineCurve();
    return true;
}

bool GeneratedPointCloudAroundCurve::set_sigma_x(double value)
{
    if (value == (*_sigma)[0])
        return false;
    (*_sigma)[0] = value;

    deletePointCloudAroundCurve();
    deleteBSplineCurve();

    createPointCloudAroundCurve();
    createBSplineCurve();
    return true;
}

bool GeneratedPointCloudAroundCurve::set_sigma_y(double value)
{
    if (value == (*_sigma)[1])
        return false;
    (*_sigma)[1] = value;

    deletePointCloudAroundCurve();
    deleteBSplineCurve();

    createPointCloudAroundCurve();
    createBSplineCurve();
    return true;
}

bool GeneratedPointCloudAroundCurve::set_sigma_z(double value)
{
    if (value == (*_sigma)[2])
        return false;
    (*_sigma)[2] = value;

    deletePointCloudAroundCurve();
    deleteBSplineCurve();

    createPointCloudAroundCurve();
    createBSplineCurve();
    return true;
}

bool GeneratedPointCloudAroundCurve::set_weight_size(int value)
{
    if (value == _weight.GetColumnCount())
        return false;
    _weight.ResizeColumns(value);
    return true;
}

bool GeneratedPointCloudAroundCurve::set_weight_value(int index, double value)
{
    if (_weight[index] == value)
        return false;
    _weight[index] = value;

    deleteBSplineCurve();
    createBSplineCurve();
    return true;
}

bool GeneratedPointCloudAroundCurve::set_cloud_size(int value)
{
    if (_point_cloud_size == value)
        return false;
    _point_cloud_size = value;

    deletePointCloudAroundCurve();
    deleteBSplineCurve();

    createPointCloudAroundCurve();
    createBSplineCurve();
    return true;
}

bool GeneratedPointCloudAroundCurve::set_scale_of_comb_vectors(double value)
{
    if (value == _scale_of_comb_vectors)
        return false;
    _scale_of_comb_vectors = value;
    return true;
}

bool GeneratedPointCloudAroundCurve::set_count_of_comb_vectors(int value)
{
    if (value == _count_of_comb_vectors)
        return false;
    _count_of_comb_vectors = value;
    return true;
}

bool GeneratedPointCloudAroundCurve::show_cloud(bool value)
{
    if (_show_cloud == value)
        return false;
    _show_cloud = value;
    return true;
}

bool GeneratedPointCloudAroundCurve::show_comb(bool value)
{
    if (_show_comb == value)
        return false;
    _show_comb = value;
    return true;
}

bool GeneratedPointCloudAroundCurve::set_cloud_point_size(double value)
{
    if (_cloud_point_size == value)
        return false;

    _cloud_point_size = value;
    return true;
}

}
