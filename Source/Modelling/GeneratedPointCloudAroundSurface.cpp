#include "GeneratedPointCloudAroundSurface.h"
#include <iostream>

using namespace std;

namespace cagd
{
GeneratedPointCloudAroundSurface::GeneratedPointCloudAroundSurface():
    ClassicBSplineSurface3()
{
    _weight.ResizeColumns(1);
    _weight[0] = 0.0;

    _sigma = new (nothrow) RowMatrix<GLdouble>(3);
    (*_sigma)[0] = (GLdouble)0.5;
    (*_sigma)[1] = (GLdouble)0.5;
    (*_sigma)[2] = (GLdouble)0.5;

    cout << "twosided_color shader: ";
    if (!_twosided_color.InstallShaders("Shaders/twosided_color.vert", "Shaders/twosided_color.frag", _loging_is_enabled))
    {
        throw Exception ("Could not install the twosided_color shader");
    }
}

bool GeneratedPointCloudAroundSurface::createParametricSurface()
{
    TriangularMatrix<ParametricSurface3::PartialDerivative> pd(2);

    switch(_selected_ps)
    {
    case 0:
        pd(0,0) = torus::d00;
        pd(1,0) = torus::d10;
        pd(1,1) = torus::d01;

        _surface_u_min = torus::u_min;
        _surface_u_max = torus::u_max;
        _surface_v_min = torus::v_min;
        _surface_v_max = torus::v_max;
        break;
    case 1:
        pd(0,0) = sphere::d00;
        pd(1,0) = sphere::d10;
        pd(1,1) = sphere::d01;

        _surface_u_min = sphere::u_min;
        _surface_u_max = sphere::u_max;
        _surface_v_min = sphere::v_min;
        _surface_v_max = sphere::v_max;
        break;
    case 2:
        pd(0,0) = cylinder::d00;
        pd(1,0) = cylinder::d10;
        pd(1,1) = cylinder::d01;

        _surface_u_min = cylinder::u_min;
        _surface_u_max = cylinder::u_max;
        _surface_v_min = cylinder::v_min;
        _surface_v_max = cylinder::v_max;
        break;
    case 3:
        pd(0,0) = cone::d00;
        pd(1,0) = cone::d10;
        pd(1,1) = cone::d01;

        _surface_u_min = cone::u_min;
        _surface_u_max = cone::u_max;
        _surface_v_min = cone::v_min;
        _surface_v_max = cone::v_max;
        break;
    case 4:

        pd(0,0) = pseudosphere::d00;
        pd(1,0) = pseudosphere::d10;
        pd(1,1) = pseudosphere::d01;

        _surface_u_min = pseudosphere::u_min;
        _surface_u_max = pseudosphere::u_max;
        _surface_v_min = pseudosphere::v_min;
        _surface_v_max = pseudosphere::v_max;
        break;
    }
    _ps = new (nothrow) ParametricSurface3
            (pd, _surface_u_min, _surface_u_max, _surface_v_min, _surface_v_max);

    if (!_ps)
    {
        throw Exception("Could not create parametric surface!");
        deleteParametricSurface();
    }

    return true;
}

bool GeneratedPointCloudAroundSurface::createPointCloudAroundSurface()
{
    _surface_cloud = new (nothrow) PointCloudAroundSurface3();

    if (!_surface_cloud)
    {
        deletePointCloudAroundSurface();
        deleteParametricSurface();
        throw Exception("Could not create point cloud around surface!");
    }
    _surface_cloud->GeneratePointCloudAroundParametricSurface(*_ps, *_sigma, _point_cloud_size_u, _point_cloud_size_v);

    return true;
}

bool GeneratedPointCloudAroundSurface::createBSplinePatch()
{
    _patch = _surface_cloud->GenerateRegressionSurface(_weight, _type_u, _type_v,
                                                       _k_u, _k_v, _n_u, _n_v,
                                                       _surface_u_min, _surface_u_max,
                                                       _surface_v_min, _surface_v_max);

    if (!_patch)
    {
        deleteAllObjects();
        throw Exception ("Could not create the patch");
    }

    if (!_patch->UpdateVertexBufferObjectsOfData())
    {
        deleteAllObjects();
        throw Exception ("Could not create the image of patch");
    }
    _img_patch = _patch->GenerateImage(_div_point_count_u, _div_point_count_v, _total_energies, _selected_color_sheme);

    if (!_img_patch || !_img_patch->UpdateVertexBufferObjects())
    {
        deleteAllObjects();
        throw Exception("error");
    }

    _patches = _patch->GenerateImageOfPatches(_div_point_count_u, _div_point_count_v, _selected_color_sheme);
    if (!_patches)
    {
        deleteAllObjects();
        throw Exception("Could not generate the patches of the B-spline patch!");
    }

    for (GLuint i = 0; i < _patches->GetRowCount(); i++)
    {
        for (GLuint j = 0; j < _patches->GetColumnCount(); j++)
        {
            if (!(*_patches)(i, j)->UpdateVertexBufferObjects())
            {
                deleteAllObjects();
                throw Exception("Could not update the VBO of all patches of the B-spline patch!");
            }
        }
    }

    return true;
}

void GeneratedPointCloudAroundSurface::updatePatchByOnePoint(int row, int column)
{
    ClassicBSplineSurface3::updatePatchByOnePoint(row, column);
}

bool GeneratedPointCloudAroundSurface::renderPointCloudAroundSurface(bool dark_mode)
{
    if (!_patch)
    {
        return false;
    }

    if (_show_cloud)
    {
        _surface_cloud->RenderPointCloud(&_unit_sphere, _cloud_point_size, dark_mode);
    }

    glEnable(GL_LIGHT0);
    if (_show_control_polygon)
    {
        glEnable(GL_LIGHTING);
        for (GLuint i = 0; i <= _n_u; i++)
        {
            for (GLuint j = 0; j <= _n_v; j++)
            {
                DCoordinate3 &point = (*_patch)(i, j);

                // provide a unique name (i.e., identifier) for each sphere
                glLoadName(i * (_n_v + 1) + j);

                glPushMatrix();
                    glTranslated(point[0], point[1], point[2]);
                    glScalef(_point_size, _point_size, _point_size);
                    MatFBTurquoise.Apply();
                    _unit_sphere.Render();
                glPopMatrix();
            }
        }
        glDisable(GL_LIGHTING);

        glColor3f(0.6f, 0.0f, 0.6f);
        _patch->RenderData(GL_LINE_STRIP);
    }

    if (_show_patch)
    {
        glDisable(GL_LIGHTING);

        if (!_no_shader)
        {
            if (_show_heat_map)
            {
                _twosided_color.Enable();
            }
            else
            {
                _two_sided_lighting_shader.Enable();
                _two_sided_lighting_shader.SetUniformVariable1i("is_enabled[0]", true);
            }
        }
        MatFBBrass.Apply();
        _img_patch->Render();
        if (!_no_shader)
        {
            if (_show_heat_map)
            {
                _twosided_color.Disable();
            }
            else
            {
                _two_sided_lighting_shader.Disable();
            }
        }
    }

    if (_show_patches)
    {
        if (!_patches)
        {
            return false;
        }

        glEnable(GL_LIGHTING);
        for (GLuint i = 0; i < _patches->GetRowCount(); i++)
        {
            for (GLuint j = 0; j < _patches->GetColumnCount(); j++)
            {
                if (!(*_patches)(i, j))
                {
                    return false;
                }
                if (i % 2)
                {
                    if (j % 2)
                    {
                        MatFBRuby.Apply();
                    }
                    else
                    {
                        MatFBGold.Apply();
                    }
                }
                else
                {
                    if (j % 2)
                    {
                        MatFBGold.Apply();
                    }
                    else
                    {
                        MatFBRuby.Apply();
                    }
                }
                if (!_no_shader)
                {
                    if (_show_heat_map)
                    {
                        _twosided_color.Enable();
                    }
                    else
                    {
                        _two_sided_lighting_shader.Enable();
                        _two_sided_lighting_shader.SetUniformVariable1i("is_enabled[0]", true);
                    }
                }
                (*_patches)(i, j)->Render();
                if (!_no_shader)
                {
                    if (_show_heat_map)
                    {
                        _twosided_color.Disable();
                    }
                    else
                    {
                        _two_sided_lighting_shader.Disable();
                    }
                }
            }

        }
       glDisable(GL_LIGHTING);
    }

    return true;
}

void GeneratedPointCloudAroundSurface::deleteParametricSurface()
{
    if (_ps)
    {
        delete _ps;
        _ps = nullptr;
    }
}

void GeneratedPointCloudAroundSurface::deletePointCloudAroundSurface()
{
    if (_surface_cloud)
    {
        delete _surface_cloud;
        _surface_cloud = nullptr;
    }
}

void GeneratedPointCloudAroundSurface::deleteAllObjects()
{
    deleteParametricSurface();
    deletePointCloudAroundSurface();
    deleteBSplinePatch();
}

void GeneratedPointCloudAroundSurface::deleteBSplinePatch()
{
    ClassicBSplineSurface3::deleteBSplinePatch();
}
// methods from classic patch
bool GeneratedPointCloudAroundSurface::set_knotvector_type_u(int index)
{
    return ClassicBSplineSurface3::set_knotvector_type_u(index);
}

bool GeneratedPointCloudAroundSurface::set_knotvector_type_v(int index)
{
    return ClassicBSplineSurface3::set_knotvector_type_v(index);
}

bool GeneratedPointCloudAroundSurface::set_knotvector_order_u(int value)
{
    return ClassicBSplineSurface3::set_knotvector_order_u(value);
}

bool GeneratedPointCloudAroundSurface::set_knotvector_order_v(int value)
{
    return ClassicBSplineSurface3::set_knotvector_order_v(value);
}

bool GeneratedPointCloudAroundSurface::set_control_points_u(int value)
{
    return ClassicBSplineSurface3::set_control_points_u(value);
}

bool GeneratedPointCloudAroundSurface::set_control_points_v(int value)
{
    return ClassicBSplineSurface3::set_control_points_v(value);
}

bool GeneratedPointCloudAroundSurface::set_div_point_coint_u(int value)
{
    return ClassicBSplineSurface3::set_div_point_coint_u(value);
}

bool GeneratedPointCloudAroundSurface::set_div_point_coint_v(int value)
{
    return ClassicBSplineSurface3::set_div_point_coint_v(value);
}

bool GeneratedPointCloudAroundSurface::show_patches(bool value)
{
    return ClassicBSplineSurface3::show_patches(value);
}

bool GeneratedPointCloudAroundSurface::show_patch(bool value)
{
    return ClassicBSplineSurface3::show_patch(value);
}

bool GeneratedPointCloudAroundSurface::show_control_polygon(bool value)
{
    return ClassicBSplineSurface3::show_control_polygon(value);
}

bool GeneratedPointCloudAroundSurface::set_control_point_size(double value)
{
    return ClassicBSplineSurface3::set_control_point_size(value);
}

GLuint GeneratedPointCloudAroundSurface::get_size()
{
    return ClassicBSplineSurface3::get_size();
}

DCoordinate3* GeneratedPointCloudAroundSurface::get_control_point(int row, int column)
{
    return ClassicBSplineSurface3::get_control_point(row, column);
}

void GeneratedPointCloudAroundSurface::calculate_indexes(GLuint name, GLint &row, GLint &column)
{
    ClassicBSplineSurface3::calculate_indexes(name, row, column);
}

void GeneratedPointCloudAroundSurface::set_shader_available(bool value)
{
    ClassicBSplineSurface3::set_shader_available(value);
}

// methods for cloud and regression
bool GeneratedPointCloudAroundSurface::set_color_sheme(int index)
{
    switch(index)
    {
    case 0:
        if (_selected_color_sheme == TensorProductSurface3::ImageColorScheme::DEFAULT_NULL_FRAGMENT)
            return false;
        _selected_color_sheme = TensorProductSurface3::ImageColorScheme::DEFAULT_NULL_FRAGMENT;
        break;
    case 1:
        if (_selected_color_sheme == TensorProductSurface3::ImageColorScheme::NORMAL_LENGTH_FRAGMENT)
            return false;
        _selected_color_sheme = TensorProductSurface3::ImageColorScheme::NORMAL_LENGTH_FRAGMENT;
        break;
    case 2:
        if (_selected_color_sheme == TensorProductSurface3::ImageColorScheme::GAUSSIAN_CURVATURE_FRAGMENT)
            return false;
        _selected_color_sheme = TensorProductSurface3::ImageColorScheme::GAUSSIAN_CURVATURE_FRAGMENT;
        break;
    case 3:
        if (_selected_color_sheme == TensorProductSurface3::ImageColorScheme::MEAN_CURVATURE_FRAGMENT)
            return false;
        _selected_color_sheme = TensorProductSurface3::ImageColorScheme::MEAN_CURVATURE_FRAGMENT;
        break;
    case 4:
        if (_selected_color_sheme == TensorProductSurface3::ImageColorScheme::WILLMORE_ENERGY_FRAGMENT)
            return false;
        _selected_color_sheme = TensorProductSurface3::ImageColorScheme::WILLMORE_ENERGY_FRAGMENT;
        break;
    case 5:
        if (_selected_color_sheme == TensorProductSurface3::ImageColorScheme::LOG_WILLMORE_ENERGY_FRAGMENT)
            return false;
        _selected_color_sheme = TensorProductSurface3::ImageColorScheme::LOG_WILLMORE_ENERGY_FRAGMENT;
        break;
    case 6:
        if (_selected_color_sheme == TensorProductSurface3::ImageColorScheme::UMBILIC_DEVIATION_ENERGY_FRAGMENT)
            return false;
        _selected_color_sheme = TensorProductSurface3::ImageColorScheme::UMBILIC_DEVIATION_ENERGY_FRAGMENT;
        break;
    case 7:
        if (_selected_color_sheme == TensorProductSurface3::ImageColorScheme::LOG_UMBILIC_DEVIATION_ENERGY_FRAGMENT)
            return false;
        _selected_color_sheme = TensorProductSurface3::ImageColorScheme::LOG_UMBILIC_DEVIATION_ENERGY_FRAGMENT;
        break;
    case 8:
        if (_selected_color_sheme == TensorProductSurface3::ImageColorScheme::TOTAL_CURVATURE_ENERGY_FRAGMENT)
            return false;
        _selected_color_sheme = TensorProductSurface3::ImageColorScheme::TOTAL_CURVATURE_ENERGY_FRAGMENT;
        break;
    case 9:
        if (_selected_color_sheme == TensorProductSurface3::ImageColorScheme::LOG_TOTAL_CURVATURE_ENERGY_FRAGMENT)
            return false;
        _selected_color_sheme = TensorProductSurface3::ImageColorScheme::LOG_TOTAL_CURVATURE_ENERGY_FRAGMENT;
        break;
    }
    _patch->UpdateColorShemeOfImage(*_img_patch, _selected_color_sheme);

    if (!_img_patch || !_img_patch->UpdateVertexBufferObjects())
    {
        deleteAllObjects();
        throw Exception("error");
    }

    _patches = _patch->GenerateImageOfPatches(_div_point_count_u, _div_point_count_v, _selected_color_sheme);
    if (!_patches)
    {
        deleteAllObjects();
        throw Exception("Could not generate the patches of the B-spline patch!");
    }

    for (GLuint i = 0; i < _patches->GetRowCount(); i++)
    {
        for (GLuint j = 0; j < _patches->GetColumnCount(); j++)
        {
            if (!(*_patches)(i, j)->UpdateVertexBufferObjects())
            {
                deleteAllObjects();
                throw Exception("Could not update the VBO of all patches of the B-spline patch!");
            }
        }
    }

    return true;
}

bool GeneratedPointCloudAroundSurface::set_parametric_surface_type(int index)
{
    if (index == _selected_ps)
        return false;
    _selected_ps = index;

    deleteAllObjects();

    createParametricSurface();
    createPointCloudAroundSurface();
    createBSplinePatch();

    return true;
}

bool GeneratedPointCloudAroundSurface::set_sigma_x(double value)
{
    if (value == (*_sigma)[0])
        return false;
    (*_sigma)[0] = value;

    deletePointCloudAroundSurface();
    deleteBSplinePatch();

    createPointCloudAroundSurface();
    createBSplinePatch();
    return true;
}

bool GeneratedPointCloudAroundSurface::set_sigma_y(double value)
{
    if (value == (*_sigma)[1])
        return false;
    (*_sigma)[1] = value;

    deletePointCloudAroundSurface();
    deleteBSplinePatch();

    createPointCloudAroundSurface();
    createBSplinePatch();
    return true;
}

bool GeneratedPointCloudAroundSurface::set_sigma_z(double value)
{
    if (value == (*_sigma)[2])
        return false;
    (*_sigma)[2] = value;

    deletePointCloudAroundSurface();
    deleteBSplinePatch();

    createPointCloudAroundSurface();
    createBSplinePatch();
    return true;
}

bool GeneratedPointCloudAroundSurface::set_weight_size(int value)
{
    if (value == _weight.GetColumnCount())
        return false;
    _weight.ResizeColumns(value);
    return true;
}

bool GeneratedPointCloudAroundSurface::set_weight_value(int index, double value)
{
    if (_weight[index] == value)
        return false;
    _weight[index] = value;

    deleteBSplinePatch();

    createBSplinePatch();
    return true;
}

bool GeneratedPointCloudAroundSurface::set_cloud_size_in_u_direction(int value)
{
    if (_point_cloud_size_u == value)
        return false;
    _point_cloud_size_u = value;

    deletePointCloudAroundSurface();
    deleteBSplinePatch();

    createPointCloudAroundSurface();
    createBSplinePatch();
    return true;
}

bool GeneratedPointCloudAroundSurface::set_cloud_size_in_v_direction(int value)
{
    if (_point_cloud_size_v == value)
        return false;
    _point_cloud_size_v = value;

    deletePointCloudAroundSurface();
    deleteBSplinePatch();

    createPointCloudAroundSurface();
    createBSplinePatch();
    return true;
}

bool GeneratedPointCloudAroundSurface::show_cloud(bool value)
{
    if (_show_cloud == value)
        return false;
    _show_cloud = value;
    return true;
}

bool GeneratedPointCloudAroundSurface::show_heat_map(bool value)
{
    if (_show_heat_map == value)
        return false;
    _show_heat_map = value;
    return true;
}

bool GeneratedPointCloudAroundSurface::set_cloud_point_size(double value)
{
    if (_cloud_point_size == value)
        return false;

    _cloud_point_size = value;
    return true;
}

RowMatrix<GLdouble> GeneratedPointCloudAroundSurface::get_energies()
{
    return _total_energies;
}

}
