#include "Modelling/ClassicBSplineSurface3.h"

#include <Core/Constants.h>
#include <Test/TestFunctions.h>

#include <iostream>
using namespace std;

namespace cagd
{
ClassicBSplineSurface3::ClassicBSplineSurface3()
{
    if (!_unit_sphere.LoadFromOFF("Models/sphere.off"))
    {
        throw Exception("Could not load the model file: sphere.off!");
    }

    if (!_unit_sphere.UpdateVertexBufferObjects(GL_DYNAMIC_DRAW))
    {
        throw Exception("Could not update the VBOs of the sphere's triangulated mesh!");
    }

    cout << "two_sided_lighting shader: ";
    if (!_two_sided_lighting_shader.InstallShaders("Shaders/two_sided_lighting.vert", "Shaders/two_sided_lighting.frag", _loging_is_enabled))
    {
        throw Exception ("Could not install the two_sided_lighting shader");
    }
    glEnable(GL_LIGHT0);
}

bool ClassicBSplineSurface3::createBSplinePatch()
{

    _patch = new (nothrow) BSplinePatch3(_type_u, _type_v, _k_u, _k_v, _n_u, _n_v);//, cylinder::u_min, cylinder::u_max, cylinder::v_min, cylinder::v_max);

    if (!_patch)
    {
        deleteBSplinePatch();
        throw Exception ("Could not create the classic B-spline patch");
    }
    GLdouble  u, v;


    //cylinder
    GLdouble  r = 0.5;
    GLdouble  a = cylinder::u_min, b = cylinder::u_max;
    for (GLuint i = 0; i <= _n_u; i++)
    {
        u = a + i * (b - a) / (_n_u);
        for (GLuint j = 0; j <= _n_v; j++)
        {
            v = 0.0;
            v = ( (-2.0) * j * PI) / (_n_v + 1);

            // calculate the control polygon points
            _patch->SetData(i, j, r * cos(v), r * sin(v), u);
        }
    }
//        for (GLuint j = 0; j <= _n_v; j++)
//        {
//            v = a + j * (b - a) / (_n_v);
//            for (GLuint i = 0; i <= _n_u; i++)
//            {
//                u = i * TWO_PI / (_n_u + 1);

//                   // calculate the control polygon points
//                   _patch->SetData(i, j, cylinder::d00(u, v));
//            }
//        }

    // simple surface
//    for (GLuint i = 0; i <= _n_u; i ++)
//    {
//        u = 0.25 * i;
//        for (GLuint j = 0; j <= _n_v; j++)
//        {
//            v = 0.25 * j;
//            _patch->SetData(i, j, u, v, 0.0);
//        }
//    }



        // torus
//        GLdouble R = 2.0, r = 1.0;
//        for (GLuint i = 0; i <= _n_u; i++)
//        {
//            u =  2 * i * PI / (_n_u + 1);
//            for (GLuint j = 0; j <= _n_v; j++)
//            {
//                   v = 2 * j * PI / (_n_v + 1);

//                   // calculate the control polygon points
//                    _patch->SetData(i, j,(R + r * sin(u)) * cos(v), (R + r * sin(u)) * sin(v), r * cos(u) );
//            }
//        }


    if (!_patch->UpdateVertexBufferObjectsOfData())
    {
        deleteBSplinePatch();
        throw Exception ("Could not create the image of the classic B-spline patch.");
    }

    RowMatrix<GLdouble> _energies;
    _img_patch = _patch->GenerateImage(_div_point_count_u, _div_point_count_v, _energies, TensorProductSurface3::DEFAULT_NULL_FRAGMENT);

    if (!_img_patch || !_img_patch->UpdateVertexBufferObjects())
    {
        deleteBSplinePatch();
        throw Exception("Could not create the VBO's of the classic B-spline patch.");
    }

    _patches = _patch->GenerateImageOfPatches(_div_point_count_u, _div_point_count_v, TensorProductSurface3::DEFAULT_NULL_FRAGMENT);
    if (!_patches)
    {
        deleteBSplinePatch();
        throw Exception("Could not generate the patches of the classic B-spline patch!");
    }

    for (GLuint i = 0; i < _patches->GetRowCount(); i++)
    {
        for (GLuint j = 0; j < _patches->GetColumnCount(); j++)
        {
            if (!(*_patches)(i, j)->UpdateVertexBufferObjects())
            {
                deleteBSplinePatch();
                throw Exception("Could not update the VBO of all patches of the classic B-spline patch!");
            }
        }
    }
    return true;
}

bool ClassicBSplineSurface3::renderBSPlinePatch()
{
    if(!_patch)
    {
        return false;
    }

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
        glLineWidth(2);
        _patch->RenderData(GL_LINE_STRIP);
    }

    if (_show_patch)
    {
        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);

        if (!_no_shader)
        {

            _two_sided_lighting_shader.Enable();
            _two_sided_lighting_shader.SetUniformVariable1i("is_enabled[0]", true);

        }
        MatFBBrass.Apply();
        _img_patch->Render();
        if (!_no_shader)
        {

            _two_sided_lighting_shader.Disable();

        }
    }

    if (_show_patches)
    {
        if (!_patches)
        {
            return false;
        }

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);

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
                    _two_sided_lighting_shader.Enable();
                    _two_sided_lighting_shader.SetUniformVariable1i("is_enabled[0]", true);
                }

                (*_patches)(i, j)->Render();

                if (!_no_shader)
                {
                    _two_sided_lighting_shader.Disable();
                }
            }
        }
       glDisable(GL_LIGHTING);
    }
    return true;
}

void ClassicBSplineSurface3::deleteBSplinePatch()
{
    if (_patch)
    {
        delete _patch;
        _patch = nullptr;
    }

    if (_img_patch)
    {
        delete _img_patch;
        _img_patch = nullptr;
    }

    for (GLuint i = 0; i < _patches->GetRowCount(); i++)
    {
        for (GLuint j = 0; j < _patches->GetColumnCount(); j++)
        {
            if ((*_patches)(i, j))
            {
                delete (*_patches)(i, j);
                (*_patches)(i, j) = nullptr;
            }
        }
    }
}

void ClassicBSplineSurface3::updatePatchByOnePoint(int row, int column)
{
    if (!_patch->UpdateVertexBufferObjectsOfData())
    {
        deleteBSplinePatch();
        throw Exception ("Could not create the image of patch");
    }
    RowMatrix<GLdouble> _energies;
    _img_patch = _patch->GenerateImage(_div_point_count_u, _div_point_count_v, _energies, TensorProductSurface3::DEFAULT_NULL_FRAGMENT);
    _patch->UpdateColorShemeOfImage(*_img_patch, TensorProductSurface3::DEFAULT_NULL_FRAGMENT);

    if (!_img_patch || !_img_patch->UpdateVertexBufferObjects())
    {
        deleteBSplinePatch();
        throw Exception("Cann't update vertex buffer objects of image of patch");
    }

    int offset_u = _k_u - 1;
    int offset_v = _k_v - 1;
    int start_index_u, final_index_u, start_index_v, final_index_v;

    if (_type_u != KnotVector::PERIODIC && _type_v != KnotVector::PERIODIC)
    {
        start_index_u = max(row, (int)_k_u - 1);
        start_index_v = max(column, (int)_k_v - 1);
        final_index_u = min(row + offset_u, (int)_n_u);
        final_index_v = min(column + offset_v, (int)_n_v);

        for (int i = start_index_u; i <= final_index_u; i++)
        {
            for (int j = start_index_v; j <= final_index_v; j++)
            {
                updateOnePatchByIndexes(i - offset_u, j - offset_v, i, j);
            }
        }
    }

    if (_type_u == KnotVector::PERIODIC && _type_v == KnotVector::PERIODIC)
    {
        start_index_u = max(row, (int)_k_u - 1);
        start_index_v = max(column, (int)_k_v - 1);
        final_index_u = row + offset_u;
        final_index_v = column + offset_v;

        for (int i = start_index_u; i <= final_index_u; i++)
        {
            for (int j = start_index_v; j <= final_index_v; j++)
            {
                updateOnePatchByIndexes(i - offset_u, j - offset_v, i, j);
            }
        }

        if (row >=0 && row <= (int)_k_u - 2)
        {
            for (GLuint r = _n_u + 1; r <= _n_u + _k_u - 1 - row; r++)
            {
                GLuint ii = row + r;

                for (int j = start_index_v; j <= final_index_v; j++)
                {
                    updateOnePatchByIndexes(ii - offset_u, j - offset_v, ii, j);
                }
            }
        }

        if (column >=0 && column <= (int)_k_u - 2)
        {
            for (int i = start_index_u; i <= final_index_u; i++)
            {
                for (GLuint r = _n_v + 1; r <= _n_v + _k_v - 1 - column; r++)
                {
                    GLuint jj = column + r;
                    updateOnePatchByIndexes(i - offset_u, jj - offset_v, i, jj);
                }
            }
        }

        if (row >=0 && row <= (int)_k_u - 2 && column >=0 && column <= (int)_k_u - 2)
        {
            for (GLuint r = _n_u + 1; r <= _n_u + _k_u - 1 - row; r++)
            {
                GLuint ii = row + r;
                for (GLuint p = _n_v + 1; p <= _n_v + _k_v - 1 - column; p++)
                {
                    GLuint jj = column + p;
                    updateOnePatchByIndexes(ii - offset_u, jj - offset_v, ii, jj);
                }
            }
        }
    }

    if (_type_u == KnotVector::PERIODIC && _type_v != KnotVector::PERIODIC)
    {
        start_index_u = max(row, (int)_k_u - 1);
        start_index_v = max(column, (int)_k_v - 1);
        final_index_u = row + offset_u;
        final_index_v = min(column + offset_v, (int)_n_v);

        for (int i = start_index_u; i <= final_index_u; i++)
        {
            for (int j = start_index_v; j <= final_index_v; j++)
            {
                updateOnePatchByIndexes(i - offset_u, j - offset_v, i, j);
            }
        }

        if (row >=0 && row <= (int)_k_u - 2)
        {
            for (GLuint r = _n_u + 1; r <= _n_u + _k_u - 1 - row; r++)
            {
                GLuint ii = row + r;

                for (int j = start_index_v; j <= final_index_v; j++)
                {
                    updateOnePatchByIndexes(ii - offset_u, j - offset_v, ii, j);
                }
            }
        }
    }

    if (_type_u != KnotVector::PERIODIC && _type_v == KnotVector::PERIODIC)
    {
        start_index_u = max(row, (int)_k_u - 1);
        start_index_v = max(column, (int)_k_v - 1);
        final_index_u = min(row + offset_u, (int)_n_u);
        final_index_v = column + offset_v;

        for (int i = start_index_u; i <= final_index_u; i++)
        {
            for (int j = start_index_v; j <= final_index_v; j++)
            {
                updateOnePatchByIndexes(i - offset_u, j - offset_v, i, j);
            }
        }

        if (column >=0 && column <= (int)_k_u - 2)
        {
            for (int i = start_index_u; i <= final_index_u; i++)
            {
                for (GLuint r = _n_v + 1; r <= _n_v + _k_v - 1 - column; r++)
                {
                    GLuint jj = column + r;
                    updateOnePatchByIndexes(i - offset_u, jj - offset_v, i, jj);
                }
            }
        }
    }
}

void ClassicBSplineSurface3::updateOnePatchByIndexes(int row, int column, int index_1, int index_2)
{
    if ((*_patches)(row, column))
    {
        delete (*_patches)(row, column);
        (*_patches)(row, column) = nullptr;
    }

    (*_patches)(row, column) = _patch->GenerateImageOfAnPatch(index_1, index_2, _div_point_count_u, _div_point_count_v, TensorProductSurface3::DEFAULT_NULL_FRAGMENT);

    if (!(*_patches)(row, column))
    {
        deleteBSplinePatch();
        throw Exception("Could not generate patch of surface!");
    }

    if (!(*_patches)(row, column)->UpdateVertexBufferObjects())
    {
        deleteBSplinePatch();
        throw Exception("Could not update the VBO of all patches of the B-spline patch!");
    }
}

bool ClassicBSplineSurface3::set_knotvector_type_u(int index)
{
    switch(index)
    {
    case 0:
        if (_type_u == KnotVector::PERIODIC)
            return false;
        _type_u = KnotVector::PERIODIC;
        break;
    case 1:
        if (_type_u == KnotVector::CLAMPED)
            return false;
        _type_u = KnotVector::CLAMPED;
        break;
    case 2:
        if (_type_u == KnotVector::UNCLAMPED)
            return false;
        _type_u = KnotVector::UNCLAMPED;
        break;
    }

    deleteBSplinePatch();
    createBSplinePatch();
    return true;
}

bool ClassicBSplineSurface3::set_knotvector_type_v(int index)
{
    switch(index)
    {
    case 0:
        if (_type_v == KnotVector::PERIODIC)
            return false;
        _type_v = KnotVector::PERIODIC;
        break;
    case 1:
        if (_type_v == KnotVector::CLAMPED)
            return false;
        _type_v = KnotVector::CLAMPED;
        break;
    case 2:
        if (_type_v == KnotVector::UNCLAMPED)
            return false;
        _type_v = KnotVector::UNCLAMPED;
        break;
    }

    deleteBSplinePatch();
    createBSplinePatch();
    return true;
}

bool ClassicBSplineSurface3::set_knotvector_order_u(int value)
{
    if (_k_u == value)
        return false;
    if (_type_u != KnotVector::PERIODIC && value > static_cast<int> (_n_u + 1))
        return false;
    _k_u = value;

    deleteBSplinePatch();
    createBSplinePatch();
    return true;
}

bool ClassicBSplineSurface3::set_knotvector_order_v(int value)
{
    if (_k_v == value)
        return false;
    if (_type_v != KnotVector::PERIODIC && value > static_cast<int> (_n_v + 1))
        return false;
    _k_v = value;

    deleteBSplinePatch();
    createBSplinePatch();
    return true;
}

bool ClassicBSplineSurface3::set_control_points_u(int value)
{
    if (_n_u == value)
        return false;
    _n_u = value;
    deleteBSplinePatch();
    createBSplinePatch();

    return true;
}

bool ClassicBSplineSurface3::set_control_points_v(int value)
{
    if (_n_v == value)
        return false;
    _n_v = value;
    deleteBSplinePatch();
    createBSplinePatch();

    return true;
}

bool ClassicBSplineSurface3::set_div_point_coint_u(int value)
{
    if (_div_point_count_u == value)
        return false;
    _div_point_count_u = value;

    deleteBSplinePatch();
    createBSplinePatch();
    return true;
}

bool ClassicBSplineSurface3::set_div_point_coint_v(int value)
{
    if (_div_point_count_v == value)
        return false;
    _div_point_count_v = value;

    deleteBSplinePatch();
    createBSplinePatch();
    return true;
}

bool ClassicBSplineSurface3::show_patches(bool value)
{
    if (_show_patches == value)
        return false;
    _show_patches = value;
    return true;
}

bool ClassicBSplineSurface3::show_patch(bool value)
{
    if (_show_patch == value)
        return false;
    _show_patch = value;
    return true;
}

bool ClassicBSplineSurface3::show_control_polygon(bool value)
{
    if (_show_control_polygon == value)
        return false;
    _show_control_polygon = value;
    return true;
}

bool ClassicBSplineSurface3::set_control_point_size(double value)
{
    if (_point_size == value)
        return false;

    _point_size = value;
     return true;
}

GLuint ClassicBSplineSurface3::get_size()
{
    return (_n_u + 1) * (_n_v + 1);
}

DCoordinate3* ClassicBSplineSurface3::get_control_point(int row, int column)
{
    return &(*_patch)(row, column);
}

void ClassicBSplineSurface3::calculate_indexes(GLuint name, GLint &row, GLint &column)
{
    row = name / (_n_v + 1);
    column = name % (_n_v + 1);
}

void ClassicBSplineSurface3::set_shader_available(bool value)
{
    _no_shader = !value;
}

}
