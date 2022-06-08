#include "PointCloudsAndModels.h"

namespace cagd
{
    PointCloudsAndModels::PointCloudsAndModels()
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

        cout << "twosided_color shader: ";
        if (!_twosided_color.InstallShaders("Shaders/twosided_color.vert", "Shaders/twosided_color.frag", _loging_is_enabled))
        {
            throw Exception ("Could not install the twosided_color shader");
        }
        glEnable(GL_LIGHT0);

        _one_var_point_clouds.ResizeColumns(0);
        _curves.ResizeColumns(0);
        _two_var_point_clouds.ResizeColumns(0);
        _surfaces.ResizeColumns(0);
    }

    bool PointCloudsAndModels::createOneVariablePointCloudRegression(int index)
    {
        _one_var_point_clouds[index]._bs = _one_var_point_clouds[index]._cloud->GenerateRegressionCurve(
                    _one_var_point_clouds[index]._type, _one_var_point_clouds[index]._k, _one_var_point_clouds[index]._n,
                    _one_var_point_clouds[index]._weight, _one_var_point_clouds[index]._curve_u_min, _one_var_point_clouds[index]._curve_u_max);

        if (!_one_var_point_clouds[index]._bs)
        {
            deleteAllOneVariablePointCloudRegressions();
            throw Exception("Could not create the B-spline curve regression!");
        }


        if (!_one_var_point_clouds[index]._bs->UpdateVertexBufferObjectsOfData())
        {
            deleteAllOneVariablePointCloudRegressions();
            throw Exception("Could not update the VBO's of the B-spline curve regression's control polygon!");
        }

        _one_var_point_clouds[index]._img_bs = _one_var_point_clouds[index]._bs->GenerateImage(2, _one_var_point_clouds[index]._div_point_count);

        if (!_one_var_point_clouds[index]._img_bs)
        {
            deleteAllOneVariablePointCloudRegressions();
            throw Exception("Could not create image of B-spline curve regression!");
        }

        if (!_one_var_point_clouds[index]._img_bs->UpdateVertexBufferObjects(_scale_of_vectors))
        {
            deleteAllOneVariablePointCloudRegressions();
            throw Exception("Could not update the VBO's of the B-spline curve regression's image!");
        }

        _one_var_point_clouds[index]._arcs = _one_var_point_clouds[index]._bs->GenerateImageOfArcs(2, _one_var_point_clouds[index]._div_point_count);

        if (!_one_var_point_clouds[index]._arcs)
        {
            deleteAllOneVariablePointCloudRegressions();
            throw Exception("Could not generate the arcs of the B-spline curve regression!");
        }

        for (GLuint i = 0; i < _one_var_point_clouds[index]._arcs->GetColumnCount(); i++)
        {
            if (!(*_one_var_point_clouds[index]._arcs)[i]->UpdateVertexBufferObjects(_scale_of_vectors))
            {
                deleteAllOneVariablePointCloudRegressions();
                throw Exception("Could not update the VBO of all arcs of the B-spline curve regression!");
            }
        }

        return true;
    }

    bool PointCloudsAndModels::createImageOfBSplineCurve(int index)
    {
        if (!_curves[index]._bs)
        {
            deleteAllBSplineCurves();
            throw Exception("Could not create the B-spline curve!");
        }
        if (!_curves[index]._bs->UpdateVertexBufferObjectsOfData())
        {
            deleteAllBSplineCurves();
            throw Exception("Could not update the VBO's off the classic B-spline curve's control polygon!");
        }

        _curves[index]._img_bs = _curves[index]._bs->GenerateImage(2, _curves[index]._div_point_count);

        if (!_curves[index]._img_bs)
        {
            deleteAllBSplineCurves();
            throw Exception("Could not create image of classic B-spline curve!");
        }

        if (!_curves[index]._img_bs->UpdateVertexBufferObjects(_scale_of_vectors))
        {
            deleteAllBSplineCurves();
            throw Exception("Could not update the VBO's of the classic B-spline curve's image!");
        }

        _curves[index]._arcs = _curves[index]._bs->GenerateImageOfArcs(2, _curves[index]._div_point_count);

        if (!_curves[index]._arcs)
        {
            deleteAllBSplineCurves();
            throw Exception("Could not generate the arcs of the classic B-spline curve!");
        }

        for (GLuint i = 0; i <_curves[index]._arcs->GetColumnCount(); i++)
        {
            if (!(*_curves[index]._arcs)[i]->UpdateVertexBufferObjects(_scale_of_vectors))
            {
                deleteAllBSplineCurves();
                throw Exception("Could not update the VBO of all arcs of the classic B-spline curve!");
            }
        }
        return true;
    }

    bool PointCloudsAndModels::createTwoVariablePointCloudRegression(int index)
    {
        _two_var_point_clouds[index]._patch = _two_var_point_clouds[index]._cloud->GenerateRegressionSurface(
                    _two_var_point_clouds[index]._weight, _two_var_point_clouds[index]._type_u, _two_var_point_clouds[index]._type_v,
                    _two_var_point_clouds[index]._k_u, _two_var_point_clouds[index]._k_v,_two_var_point_clouds[index]._n_u, _two_var_point_clouds[index]._n_v,
                    _two_var_point_clouds[index]._surface_u_min, _two_var_point_clouds[index]._surface_u_max,
                    _two_var_point_clouds[index]._surface_v_min, _two_var_point_clouds[index]._surface_v_max);

        if (!_two_var_point_clouds[index]._patch)
        {
            deleteAllBSplineSurfaces();
            throw Exception ("Could not create the patch");
        }

        if (!_two_var_point_clouds[index]._patch->UpdateVertexBufferObjectsOfData())
        {
            deleteAllBSplineSurfaces();
            throw Exception ("Could not create the image of patch");
        }
        _two_var_point_clouds[index]._img_patch = _two_var_point_clouds[index]._patch->GenerateImage(
                    _two_var_point_clouds[index]._div_point_count_u,
                    _two_var_point_clouds[index]._div_point_count_v, _two_var_point_clouds[index]._total_energies, _selected_color_sheme);

        if (!_two_var_point_clouds[index]._img_patch || !_two_var_point_clouds[index]._img_patch->UpdateVertexBufferObjects())
        {
            deleteAllBSplineSurfaces();
            throw Exception("error");
        }

        _two_var_point_clouds[index]._patches = _two_var_point_clouds[index]._patch->GenerateImageOfPatches(
                    _two_var_point_clouds[index]._div_point_count_u, _two_var_point_clouds[index]._div_point_count_v, _selected_color_sheme);
        if (!_two_var_point_clouds[index]._patches)
        {
            deleteAllBSplineSurfaces();
            throw Exception("Could not generate the patches of the B-spline patch!");
        }

        for (GLuint i = 0; i < _two_var_point_clouds[index]._patches->GetRowCount(); i++)
        {
            for (GLuint j = 0; j < _two_var_point_clouds[index]._patches->GetColumnCount(); j++)
            {
                if (!(*_two_var_point_clouds[index]._patches)(i, j)->UpdateVertexBufferObjects())
                {
                    deleteAllBSplineSurfaces();
                    throw Exception("Could not update the VBO of all patches of the B-spline patch!");
                }
            }
        }

        return true;
    }

    bool PointCloudsAndModels::createImageOfBSplineSurface(int index)
    {
        if (!_surfaces[index]._patch->UpdateVertexBufferObjectsOfData())
        {
            deleteAllBSplineSurfaces();
            throw Exception ("Could not create the image of the classic B-spline patch.");
        }

        _surfaces[index]._img_patch = _surfaces[index]._patch->GenerateImage(
                    _surfaces[index]._div_point_count_u, _surfaces[index]._div_point_count_v,
                    _surfaces[index]._total_energies, _selected_color_sheme);

        if (!_surfaces[index]._img_patch || !_surfaces[index]._img_patch->UpdateVertexBufferObjects())
        {
            deleteAllBSplineSurfaces();
            throw Exception("Could not create the VBO's of the classic B-spline patch.");
        }

        _surfaces[index]._patches = _surfaces[index]._patch->GenerateImageOfPatches(
                    _surfaces[index]._div_point_count_u, _surfaces[index]._div_point_count_v, _selected_color_sheme);

        if (!_surfaces[index]._patches)
        {
            deleteAllBSplineSurfaces();
            throw Exception("Could not generate the patches of the classic B-spline patch!");
        }

        for (GLuint i = 0; i < _surfaces[index]._patches->GetRowCount(); i++)
        {
            for (GLuint j = 0; j < _surfaces[index]._patches->GetColumnCount(); j++)
            {
                if (!(*_surfaces[index]._patches)(i, j)->UpdateVertexBufferObjects())
                {
                    deleteAllBSplineSurfaces();
                    throw Exception("Could not update the VBO of all patches of the classic B-spline patch!");
                }
            }
        }
        return true;
    }

    bool PointCloudsAndModels::renderOneVariablePointCloudAndRegressions(bool dark_mode)
    {
        GLuint offset = 0;

        for (GLuint pc = 0; pc < _one_var_point_clouds.GetColumnCount(); pc++)
        {
            glLineWidth(2);

            glPushMatrix();

            // transformations
            glTranslated(_one_var_point_clouds[pc]._trans_x, _one_var_point_clouds[pc]._trans_y, _one_var_point_clouds[pc]._trans_z);
            glScaled(_one_var_point_clouds[pc]._scale, _one_var_point_clouds[pc]._scale, _one_var_point_clouds[pc]._scale);
            glRotatef(_one_var_point_clouds[pc]._angle_x, 1.0, 0.0, 0.0);
            glRotatef(_one_var_point_clouds[pc]._angle_y, 0.0, 1.0, 0.0);
            glRotatef(_one_var_point_clouds[pc]._angle_z, 0.0, 0.0, 1.0);

            if (!_one_var_point_clouds[pc]._bs)
            {
                return false;
            }

            if (_show_control_polygon)
            {
                glEnable(GL_LIGHTING);
                glEnable(GL_LIGHT0);
                glEnable(GL_NORMALIZE);
                for (GLuint i = 0; i <= _one_var_point_clouds[pc]._n; i++)
                {
                    DCoordinate3 &point = (*_one_var_point_clouds[pc]._bs)[i];

                    glLoadName(offset + i);
//                    cout << "Reg Curve Cp " << offset + i << endl;

                    glPushMatrix();
                    glTranslated(point[0], point[1], point[2]);
                    glScalef(_control_point_size, _control_point_size, _control_point_size);
                    MatFBTurquoise.Apply();
                    _unit_sphere.Render();
                    glPopMatrix();
                }

                glDisable(GL_LIGHTING);

                glColor3f(0.6f, 0.0f, 0.6f);
                _one_var_point_clouds[pc]._bs->RenderData(_one_var_point_clouds[pc]._type == KnotVector::PERIODIC ? GL_LINE_LOOP : GL_LINE_STRIP);
            }
            glPopMatrix();

            glLoadName(offset + _one_var_point_clouds[pc]._n + 1);
//            cout << "Reg Curve model " << offset + _one_var_point_clouds[pc]._n + 1 << endl;
            glPushMatrix();

            // transformations
            glTranslated(_one_var_point_clouds[pc]._trans_x, _one_var_point_clouds[pc]._trans_y, _one_var_point_clouds[pc]._trans_z);
            glScaled(_one_var_point_clouds[pc]._scale, _one_var_point_clouds[pc]._scale, _one_var_point_clouds[pc]._scale);
            glRotatef(_one_var_point_clouds[pc]._angle_x, 1.0, 0.0, 0.0);
            glRotatef(_one_var_point_clouds[pc]._angle_y, 0.0, 1.0, 0.0);
            glRotatef(_one_var_point_clouds[pc]._angle_z, 0.0, 0.0, 1.0);

            if (_show_comb)
            {
                glColor3f(0.6f, 1.0f, 1.0f);
                _one_var_point_clouds[pc]._bs->RenderCurvatureComb(_count_of_comb_vectors, _scale_of_comb_vectors);
            }

            if (_show_cloud)
            {
                if (_selected_type == ONE_VARIABLE && pc == _selected_model)
                {
                    _one_var_point_clouds[pc]._cloud->RenderPointCloud(&_unit_sphere, _cloud_point_size, dark_mode, false);
                }
                else
                {
                    _one_var_point_clouds[pc]._cloud->RenderPointCloud(&_unit_sphere, _cloud_point_size, dark_mode, true);
                }
            }

            if (_show_curve)
            {
                if (_selected_type == ONE_VARIABLE && pc == _selected_model)
                {
                    glColor3d(0.0, 0.2, 1.0);
                }
                else
                {
                    glColor3d(0.0, 1.0, 0.0);
                }
                glLineWidth(2);
                _one_var_point_clouds[pc]._img_bs->RenderDerivatives(0, GL_LINE_STRIP);
            }

            if (_show_tangents)
            {
                glColor3d(0.0, 1.0, 0.0);
                _one_var_point_clouds[pc]._img_bs->RenderDerivatives(1, GL_LINES);
            }

            if (_show_acceleration_vectors)
            {
                glColor3d(0.0, 1.0, 1.0);
                _one_var_point_clouds[pc]._img_bs->RenderDerivatives(2, GL_LINES);
            }

            if (_show_arcs)
            {
                if (!_one_var_point_clouds[pc]._arcs)
                {
                    return false;
                }

                for (GLuint i = 0; i < _one_var_point_clouds[pc]._arcs->GetColumnCount(); i++)
                {
                    if (!(*_one_var_point_clouds[pc]._arcs)[i])
                    {
                        return false;
                    }

                    if (i % 2)
                    {
                        if (_selected_type == ONE_VARIABLE && pc == _selected_model)
                        {
                            glColor3d(0.5, 0.5, 0.5);
                        }
                        else
                        {
                            glColor3d(1.0, 0.0, 0.0);
                        }
                    }
                    else
                    {
                        if (_selected_type == ONE_VARIABLE && pc == _selected_model)
                        {
                            glColor3d(0.0, 0.2, 1.0);
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
                    }

                    (*_one_var_point_clouds[pc]._arcs)[i]->RenderDerivatives(0, GL_LINE_STRIP);
                }
            }
            glPopMatrix();
            offset += _one_var_point_clouds[pc]._n + 2;
        }
        return true;
    }

    bool PointCloudsAndModels::renderBSplineCurves(bool dark_mode)
    {
        GLuint offset = get_named_objects_count_of_one_var_point_clouds();

        for(GLuint c = 0; c < _curves.GetColumnCount(); c++)
        {
            glLineWidth(2);

            glPushMatrix();

            // transformations
            glTranslated(_curves[c]._trans_x, _curves[c]._trans_y, _curves[c]._trans_z);
            glScaled(_curves[c]._scale, _curves[c]._scale, _curves[c]._scale);
            glRotatef(_curves[c]._angle_x, 1.0, 0.0, 0.0);
            glRotatef(_curves[c]._angle_y, 0.0, 1.0, 0.0);
            glRotatef(_curves[c]._angle_z, 0.0, 0.0, 1.0);

            if (!_curves[c]._bs)
            {
                return false;
            }

            if (_show_control_polygon)
            {
                glEnable(GL_LIGHTING);
                glEnable(GL_LIGHT0);
                glEnable(GL_NORMALIZE);

                for (GLuint i = 0; i <= _curves[c]._n; i++)
                {
                    DCoordinate3 &point = (*_curves[c]._bs)[i];

                    glLoadName(offset + i);
//                    cout << "Curve " << offset + i << endl;

                    glPushMatrix();
                    glTranslated(point[0], point[1], point[2]);
                    glScalef(_control_point_size, _control_point_size, _control_point_size);
                    MatFBTurquoise.Apply();
                    _unit_sphere.Render();
                    glPopMatrix();
                }

                glDisable(GL_LIGHTING);

                glColor3f(0.6f, 0.0f, 0.6f);
                _curves[c]._bs->RenderData(_curves[c]._type == KnotVector::PERIODIC ? GL_LINE_LOOP : GL_LINE_STRIP);
            }
            glPopMatrix();

            glLoadName(offset + _curves[c]._n + 1);
//            cout << "Curve model " << offset + _curves[c]._n + 1 << endl;
            glPushMatrix();

            // transformations
            glTranslated(_curves[c]._trans_x, _curves[c]._trans_y, _curves[c]._trans_z);
            glScaled(_curves[c]._scale, _curves[c]._scale, _curves[c]._scale);
            glRotatef(_curves[c]._angle_x, 1.0, 0.0, 0.0);
            glRotatef(_curves[c]._angle_y, 0.0, 1.0, 0.0);
            glRotatef(_curves[c]._angle_z, 0.0, 0.0, 1.0);

            if (_show_curve)
            {
                if (_selected_type == CURVE && c == _selected_model)
                {
                    glColor3d(0.0, 0.2, 1.0);
                }
                else
                {
                    glColor3d(0.0, 1.0, 0.0);
                }
                glLineWidth(2);
                _curves[c]._img_bs->RenderDerivatives(0, GL_LINE_STRIP);
            }


            if (_show_comb)
            {
                glColor3f(0.6f, 1.0f, 1.0f);
                _curves[c]._bs->RenderCurvatureComb(_count_of_comb_vectors, _scale_of_comb_vectors);
            }

            if (_show_tangents)
            {
                glColor3d(0.0, 1.0, 0.0);
                _curves[c]._img_bs->RenderDerivatives(1, GL_LINES);
            }

            if (_show_acceleration_vectors)
            {
                glColor3d(0.0, 1.0, 1.0);
                _curves[c]._img_bs->RenderDerivatives(2, GL_LINES);
            }

            if (_show_arcs)
            {
                if (!_curves[c]._arcs)
                {
                    return false;
                }

                for (GLuint i = 0; i < _curves[c]._arcs->GetColumnCount(); i++)
                {
                    if (!(*_curves[c]._arcs)[i])
                    {
                        return false;
                    }

                    if (i % 2)
                    {
                        if (_selected_type == CURVE && c == _selected_model)
                        {
                            glColor3d(0.5, 0.5, 0.5);
                        }
                        else
                        {
                            glColor3d(1.0, 0.0, 0.0);
                        }
                    }
                    else
                    {
                        if (_selected_type == CURVE && c == _selected_model)
                        {
                            glColor3d(0.0, 0.2, 1.0);
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
                    }

                    (*_curves[c]._arcs)[i]->RenderDerivatives(0, GL_LINE_STRIP);
                }
            }
            glPopMatrix();
            offset += _curves[c]._n + 2;
        }
        return true;
    }

    bool PointCloudsAndModels::renderTwoVariablePointCloudAndRegressions(bool dark_mode)
    {
        GLuint offset = get_named_objects_count_of_one_var_point_clouds() + get_named_objects_count_of_bspline_curves();
        for (GLuint pc = 0; pc < _two_var_point_clouds.GetColumnCount(); pc++)
        {
            glPushMatrix();

            // transformations
            glTranslated(_two_var_point_clouds[pc]._trans_x, _two_var_point_clouds[pc]._trans_y, _two_var_point_clouds[pc]._trans_z);
            glScaled(_two_var_point_clouds[pc]._scale, _two_var_point_clouds[pc]._scale, _two_var_point_clouds[pc]._scale);
            glRotatef(_two_var_point_clouds[pc]._angle_x, 1.0, 0.0, 0.0);
            glRotatef(_two_var_point_clouds[pc]._angle_y, 0.0, 1.0, 0.0);
            glRotatef(_two_var_point_clouds[pc]._angle_z, 0.0, 0.0, 1.0);

            if (!_two_var_point_clouds[pc]._patch)
            {
                return false;
            }

            if (_show_control_polygon)
            {
                glEnable(GL_LIGHTING);
                for (GLuint i = 0; i <= _two_var_point_clouds[pc]._n_u; i++)
                {
                    for (GLuint j = 0; j <= _two_var_point_clouds[pc]._n_v; j++)
                    {
                        DCoordinate3 &point = (*_two_var_point_clouds[pc]._patch)(i, j);

                        // provide a unique name (i.e., identifier) for each sphere
                        glLoadName(offset + i * (_two_var_point_clouds[pc]._n_v + 1) + j);
//                        cout << "Surface cloud " << offset + i * (_two_var_point_clouds[pc]._n_v + 1) + j << endl;

                        glPushMatrix();
                            glTranslated(point[0], point[1], point[2]);
                            glScalef(_control_point_size, _control_point_size, _control_point_size);
                            MatFBTurquoise.Apply();
                            _unit_sphere.Render();
                        glPopMatrix();
                    }
                }
                glDisable(GL_LIGHTING);

                glColor3f(0.6f, 0.0f, 0.6f);
                _two_var_point_clouds[pc]._patch->RenderData(GL_LINE_STRIP);
            }
            glPopMatrix();

            glLoadName(offset + (_two_var_point_clouds[pc]._n_u + 1)*(_two_var_point_clouds[pc]._n_v + 1));
//            cout << "Surface cloud model" << offset + (_two_var_point_clouds[pc]._n_u + 1)*(_two_var_point_clouds[pc]._n_v + 1) << endl;
            glPushMatrix();

            // transformations
            glTranslated(_two_var_point_clouds[pc]._trans_x, _two_var_point_clouds[pc]._trans_y, _two_var_point_clouds[pc]._trans_z);
            glScaled(_two_var_point_clouds[pc]._scale, _two_var_point_clouds[pc]._scale, _two_var_point_clouds[pc]._scale);
            glRotatef(_two_var_point_clouds[pc]._angle_x, 1.0, 0.0, 0.0);
            glRotatef(_two_var_point_clouds[pc]._angle_y, 0.0, 1.0, 0.0);
            glRotatef(_two_var_point_clouds[pc]._angle_z, 0.0, 0.0, 1.0);
            if (_show_cloud)
            {
                if (_selected_type == TWO_VARIABLE && pc == _selected_model)
                {
                    _two_var_point_clouds[pc]._cloud->RenderPointCloud(&_unit_sphere, _cloud_point_size, dark_mode, false);
                }
                else
                {
                    _two_var_point_clouds[pc]._cloud->RenderPointCloud(&_unit_sphere, _cloud_point_size, dark_mode, true);
                }
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
                if (_selected_type == TWO_VARIABLE && pc == _selected_model)
                {
                    MatFBSilver.Apply();
                }
                else
                {
                    MatFBBrass.Apply();
                }
                _two_var_point_clouds[pc]._img_patch->Render();
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
                if (!_two_var_point_clouds[pc]._patches)
                {
                    return false;
                }

                glEnable(GL_LIGHTING);
                for (GLuint i = 0; i < _two_var_point_clouds[pc]._patches->GetRowCount(); i++)
                {
                    for (GLuint j = 0; j < _two_var_point_clouds[pc]._patches->GetColumnCount(); j++)
                    {
                        if (!(*_two_var_point_clouds[pc]._patches)(i, j))
                        {
                            return false;
                        }
                        if (i % 2)
                        {
                            if (j % 2)
                            {
                                if (_selected_type == TWO_VARIABLE && pc == _selected_model)
                                {
                                    MatFBSilver.Apply();
                                }
                                else
                                {
                                    MatFBRuby.Apply();
                                }

                            }
                            else
                            {
                                if (_selected_type == TWO_VARIABLE && pc == _selected_model)
                                {
                                    MatFBEmerald.Apply();
                                }
                                else
                                {
                                    MatFBGold.Apply();
                                }
                            }
                        }
                        else
                        {
                            if (j % 2)
                            {
                                if (_selected_type == TWO_VARIABLE && pc == _selected_model)
                                {
                                    MatFBEmerald.Apply();
                                }
                                else
                                {
                                    MatFBGold.Apply();
                                }
                            }
                            else
                            {
                                if (_selected_type == TWO_VARIABLE && pc == _selected_model)
                                {
                                    MatFBSilver.Apply();
                                }
                                else
                                {
                                    MatFBRuby.Apply();
                                }
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

                        (*_two_var_point_clouds[pc]._patches)(i, j)->Render();

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
            glPopMatrix();
            offset += (_two_var_point_clouds[pc]._n_u + 1)*(_two_var_point_clouds[pc]._n_v + 1) + 1;
        }
        return true;
    }

    bool PointCloudsAndModels::renderBSplineSurfaces(bool)
    {
        GLuint offset = get_named_objects_count_of_one_var_point_clouds() + get_named_objects_count_of_bspline_curves()
                + get_named_objects_count_of_two_var_point_clouds();
        for (GLuint s = 0; s < _surfaces.GetColumnCount(); s++)
        {
            glPushMatrix();

            // transformations
            glTranslated(_surfaces[s]._trans_x, _surfaces[s]._trans_y, _surfaces[s]._trans_z);
            glScaled(_surfaces[s]._scale, _surfaces[s]._scale, _surfaces[s]._scale);
            glRotatef(_surfaces[s]._angle_x, 1.0, 0.0, 0.0);
            glRotatef(_surfaces[s]._angle_y, 0.0, 1.0, 0.0);
            glRotatef(_surfaces[s]._angle_z, 0.0, 0.0, 1.0);

            if(!_surfaces[s]._patch)
            {
                return false;
            }

            if (_show_control_polygon)
            {
                glEnable(GL_LIGHTING);
                for (GLuint i = 0; i <= _surfaces[s]._n_u; i++)
                {
                    for (GLuint j = 0; j <= _surfaces[s]._n_v; j++)
                    {
                        DCoordinate3 &point = (*_surfaces[s]._patch)(i, j);

                        // provide a unique name (i.e., identifier) for each sphere
                        glLoadName(offset + i * (_surfaces[s]._n_v + 1) + j);
//                        cout << "Surface " << offset + i * (_surfaces[s]._n_v + 1) + j << endl;

                        glPushMatrix();
                        glTranslated(point[0], point[1], point[2]);
                        glScalef(_control_point_size, _control_point_size, _control_point_size);
                        MatFBTurquoise.Apply();
                        _unit_sphere.Render();
                        glPopMatrix();
                    }
                }
                glDisable(GL_LIGHTING);

                glColor3f(0.6f, 0.0f, 0.6f);
                glLineWidth(2);
                _surfaces[s]._patch->RenderData(GL_LINE_STRIP);
            }
            glPopMatrix();

            glLoadName(offset + (_surfaces[s]._n_u + 1)*(_surfaces[s]._n_v + 1));
//            cout << "Surface model " << offset + (_surfaces[s]._n_u + 1)*(_surfaces[s]._n_v + 1) << endl;
            glPushMatrix();

            // transformations
            glTranslated(_surfaces[s]._trans_x, _surfaces[s]._trans_y, _surfaces[s]._trans_z);
            glScaled(_surfaces[s]._scale, _surfaces[s]._scale, _surfaces[s]._scale);
            glRotatef(_surfaces[s]._angle_x, 1.0, 0.0, 0.0);
            glRotatef(_surfaces[s]._angle_y, 0.0, 1.0, 0.0);
            glRotatef(_surfaces[s]._angle_z, 0.0, 0.0, 1.0);

            if (_show_patch)
            {
                glEnable(GL_LIGHTING);
                glEnable(GL_LIGHT0);

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

                if (_selected_type == SURFACE && s == _selected_model)
                {
                    MatFBSilver.Apply();
                }
                else
                {
                    MatFBBrass.Apply();
                }

                _surfaces[s]._img_patch->Render();

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
                if (!_surfaces[s]._patches)
                {
                    return false;
                }

                glEnable(GL_LIGHTING);
                glEnable(GL_LIGHT0);

                for (GLuint i = 0; i < _surfaces[s]._patches->GetRowCount(); i++)
                {
                    for (GLuint j = 0; j < _surfaces[s]._patches->GetColumnCount(); j++)
                    {
                        if (!(*_surfaces[s]._patches)(i, j))
                        {
                            return false;
                        }
                        if (i % 2)
                        {
                            if (j % 2)
                            {
                                if (_selected_type == SURFACE && s == _selected_model)
                                {
                                    MatFBSilver.Apply();
                                }
                                else
                                {
                                    MatFBRuby.Apply();
                                }

                            }
                            else
                            {
                                if (_selected_type == SURFACE && s == _selected_model)
                                {
                                    MatFBEmerald.Apply();
                                }
                                else
                                {
                                    MatFBGold.Apply();
                                }
                            }
                        }
                        else
                        {
                            if (j % 2)
                            {
                                if (_selected_type == SURFACE && s == _selected_model)
                                {
                                    MatFBEmerald.Apply();
                                }
                                else
                                {
                                    MatFBGold.Apply();
                                }
                            }
                            else
                            {
                                if (_selected_type == SURFACE && s == _selected_model)
                                {
                                    MatFBSilver.Apply();
                                }
                                else
                                {
                                    MatFBRuby.Apply();
                                }
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

                        (*_surfaces[s]._patches)(i, j)->Render();

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
            glPopMatrix();
            offset += (_surfaces[s]._n_u + 1)*(_surfaces[s]._n_v + 1) + 1;
        }
        return true;
    }

    void PointCloudsAndModels::deleteOneVariablePointCloudRegression(int index)
    {
        if (_one_var_point_clouds[index]._bs)
        {
            delete _one_var_point_clouds[index]._bs;
            _one_var_point_clouds[index]._bs = nullptr;
        }

        if (_one_var_point_clouds[index]._img_bs)
        {
            delete _one_var_point_clouds[index]._img_bs;
            _one_var_point_clouds[index]._img_bs = nullptr;
        }

        for (GLuint i = 0; i < _one_var_point_clouds[index]._arcs->GetColumnCount(); i++)
        {
            if ((*_one_var_point_clouds[index]._arcs)[i])
            {
                delete (*_one_var_point_clouds[index]._arcs)[i];
                (*_one_var_point_clouds[index]._arcs)[i]=nullptr;
            }
        }
    }

    void PointCloudsAndModels::deleteBSplineCurve(int index)
    {
//        if (_curves[index]._bs)
//        {
//            delete _curves[index]._bs;
//            _curves[index]._bs = nullptr;
//        }

        if (_curves[index]._img_bs)
        {
            delete _curves[index]._img_bs;
            _curves[index]._img_bs = nullptr;
        }

        for (GLuint i = 0; i < _curves[index]._arcs->GetColumnCount(); i++)
        {
            if ((*_curves[index]._arcs)[i])
            {
                delete (*_curves[index]._arcs)[i];
                (*_curves[index]._arcs)[i]=nullptr;
            }
        }
    }

    void PointCloudsAndModels::deleteTwoVariablePointCloudRegression(int index)
    {
        if (_two_var_point_clouds[index]._patch)
        {
            delete _two_var_point_clouds[index]._patch;
            _two_var_point_clouds[index]._patch = nullptr;
        }

        if (_two_var_point_clouds[index]._img_patch)
        {
            delete _two_var_point_clouds[index]._img_patch;
            _two_var_point_clouds[index]._img_patch = nullptr;
        }

        for (GLuint i = 0; i < _two_var_point_clouds[index]._patches->GetRowCount(); i++)
        {
            for (GLuint j = 0; j < _two_var_point_clouds[index]._patches->GetColumnCount(); j++)
            {
                if ((*_two_var_point_clouds[index]._patches)(i, j))
                {
                    delete (*_two_var_point_clouds[index]._patches)(i, j);
                    (*_two_var_point_clouds[index]._patches)(i, j) = nullptr;
                }
            }
        }
    }

    void PointCloudsAndModels::deleteBSplineSurface(int index)
    {
//        if (_surfaces[index]._patch)
//        {
//            delete _surfaces[index]._patch;
//            _surfaces[index]._patch = nullptr;
//        }

        if (_surfaces[index]._img_patch)
        {
            delete _surfaces[index]._img_patch;
            _surfaces[index]._img_patch = nullptr;
        }

        for (GLuint i = 0; i < _surfaces[index]._patches->GetRowCount(); i++)
        {
            for (GLuint j = 0; j < _surfaces[index]._patches->GetColumnCount(); j++)
            {
                if ((*_surfaces[index]._patches)(i, j))
                {
                    delete (*_surfaces[index]._patches)(i, j);
                    (*_surfaces[index]._patches)(i, j) = nullptr;
                }
            }
        }
    }

    void PointCloudsAndModels::deleteAllOneVariablePointCloudRegressions()
    {
        for (GLuint index = 0; index < _one_var_point_clouds.GetColumnCount(); index ++)
        {
            if (_one_var_point_clouds[index]._bs)
            {
                delete _one_var_point_clouds[index]._bs;
                _one_var_point_clouds[index]._bs = nullptr;
            }

            if (_one_var_point_clouds[index]._img_bs)
            {
                delete _one_var_point_clouds[index]._img_bs;
                _one_var_point_clouds[index]._img_bs = nullptr;
            }

            for (GLuint i = 0; i < _one_var_point_clouds[index]._arcs->GetColumnCount(); i++)
            {
                if ((*_one_var_point_clouds[index]._arcs)[i])
                {
                    delete (*_one_var_point_clouds[index]._arcs)[i];
                    (*_one_var_point_clouds[index]._arcs)[i]=nullptr;
                }
            }

            delete _one_var_point_clouds[index]._cloud;
            _one_var_point_clouds[index]._cloud = nullptr;
        }
    }

    void PointCloudsAndModels::deleteAllBSplineCurves()
    {
        for(GLuint index = 0; index < _curves.GetColumnCount(); index++)
        {
            if (_curves[index]._bs)
            {
                delete _curves[index]._bs;
                _curves[index]._bs = nullptr;
            }

            if (_curves[index]._img_bs)
            {
                delete _curves[index]._img_bs;
                _curves[index]._img_bs = nullptr;
            }

            for (GLuint i = 0; i < _curves[index]._arcs->GetColumnCount(); i++)
            {
                if ((*_curves[index]._arcs)[i])
                {
                    delete (*_curves[index]._arcs)[i];
                    (*_curves[index]._arcs)[i]=nullptr;
                }
            }
        }
    }

    void PointCloudsAndModels::deleteAllTwoVariablePointCloudRegressions()
    {
        for (GLuint pc = 0; pc < _two_var_point_clouds.GetColumnCount(); pc++)
        {
            if (_two_var_point_clouds[pc]._patch)
            {
                delete _two_var_point_clouds[pc]._patch;
                _two_var_point_clouds[pc]._patch = nullptr;
            }

            if (_two_var_point_clouds[pc]._img_patch)
            {
                delete _two_var_point_clouds[pc]._img_patch;
                _two_var_point_clouds[pc]._img_patch = nullptr;
            }

            for (GLuint i = 0; i < _two_var_point_clouds[pc]._patches->GetRowCount(); i++)
            {
                for (GLuint j = 0; j < _two_var_point_clouds[pc]._patches->GetColumnCount(); j++)
                {
                    if ((*_two_var_point_clouds[pc]._patches)(i, j))
                    {
                        delete (*_two_var_point_clouds[pc]._patches)(i, j);
                        (*_two_var_point_clouds[pc]._patches)(i, j) = nullptr;
                    }
                }
            }

            delete _two_var_point_clouds[pc]._cloud;
            _two_var_point_clouds[pc]._cloud = nullptr;
        }
    }

    void PointCloudsAndModels::deleteAllBSplineSurfaces()
    {
        for (GLuint s = 0; s < _surfaces.GetColumnCount(); s++)
        {
            if (_surfaces[s]._patch)
            {
                delete _surfaces[s]._patch;
                _surfaces[s]._patch = nullptr;
            }

            if (_surfaces[s]._img_patch)
            {
                delete _surfaces[s]._img_patch;
                _surfaces[s]._img_patch = nullptr;
            }

            for (GLuint i = 0; i < _surfaces[s]._patches->GetRowCount(); i++)
            {
                for (GLuint j = 0; j < _surfaces[s]._patches->GetColumnCount(); j++)
                {
                    if ((*_surfaces[s]._patches)(i, j))
                    {
                        delete (*_surfaces[s]._patches)(i, j);
                        (*_surfaces[s]._patches)(i, j) = nullptr;
                    }
                }
            }
        }
    }

    // -------
    // Methods for regression and simple curves
    // -------
    void PointCloudsAndModels::updateCurveByOnePoint(int point_index)
    {
        if (_selected_type != ONE_VARIABLE && _selected_type != CURVE)
        {
            return;
        }

        BSplineCurve3 *bs = _selected_type == ONE_VARIABLE ? _one_var_point_clouds[_selected_model]._bs : _curves[_selected_model]._bs;
        GenericCurve3 *img_bs = _selected_type == ONE_VARIABLE ? _one_var_point_clouds[_selected_model]._img_bs : _curves[_selected_model]._img_bs;
        RowMatrix<GenericCurve3*> *arcs = _selected_type == ONE_VARIABLE ? _one_var_point_clouds[_selected_model]._arcs : _curves[_selected_model]._arcs;
        GLuint k = _selected_type == ONE_VARIABLE ? _one_var_point_clouds[_selected_model]._k : _curves[_selected_model]._k;
        KnotVector::Type type = _selected_type == ONE_VARIABLE ? _one_var_point_clouds[_selected_model]._type : _curves[_selected_model]._type;
        GLuint div_point_count = _selected_type == ONE_VARIABLE ? _one_var_point_clouds[_selected_model]._div_point_count : _curves[_selected_model]._div_point_count;
        GLuint n = _selected_type == ONE_VARIABLE ? _one_var_point_clouds[_selected_model]._n : _curves[_selected_model]._n;

        if (!bs->UpdateVertexBufferObjectsOfData())
        {
            if (_selected_model == ONE_VARIABLE)
            {
                deleteAllOneVariablePointCloudRegressions();
            }
            else
            {
                deleteAllBSplineCurves();
            }
            throw Exception("Could not update the VBO's pff the bspline curve's control polygon");
        }

        img_bs = bs->GenerateImage(2, div_point_count);

        if (!img_bs || !img_bs->UpdateVertexBufferObjects(_scale_of_vectors))
        {
            if (_selected_model == ONE_VARIABLE)
            {
                deleteAllOneVariablePointCloudRegressions();
            }
            else
            {
                deleteAllBSplineCurves();
            }
            throw Exception("Could not create image of bspline curve");
        }

        int offset = k - 1;

        if (type != KnotVector::PERIODIC)
        {
            int start_index = max(point_index, (int)k - 1);
            int final_index = min(point_index + offset, (int)n);
            for (int i = start_index; i <= final_index; i++)
            {
                if ((*arcs)[i - offset])
                {
                    delete (*arcs)[i - offset];
                    (*arcs)[i - offset] = nullptr;
                }

                (*arcs)[i - offset] = bs->GenerateImageOfAnArc(i, 2, div_point_count);

                if (!(*arcs)[i - offset] || !(*arcs)[i - offset]->UpdateVertexBufferObjects(_scale_of_vectors))
                {
                    if (_selected_model == ONE_VARIABLE)
                    {
                        deleteAllOneVariablePointCloudRegressions();
                    }
                    else
                    {
                        deleteAllBSplineCurves();
                    }
                    throw Exception("Could not generate the arcs of the B-spline curve!");
                }
            }
        }
        else
        {
            int start_index = max(point_index, (int)k - 1);
            int final_index = point_index + offset;
            for (int i = start_index; i <= final_index; i++)
            {
                if ((*arcs)[i - offset])
                {
                    delete (*arcs)[i - offset];
                    (*arcs)[i - offset] = nullptr;
                }

                (*arcs)[i - offset] = bs->GenerateImageOfAnArc(i, 2, div_point_count);

                if (!(*arcs)[i - offset] || !(*arcs)[i - offset]->UpdateVertexBufferObjects(_scale_of_vectors))
                {
                    if (_selected_model == ONE_VARIABLE)
                    {
                        deleteAllOneVariablePointCloudRegressions();
                    }
                    else
                    {
                        deleteAllBSplineCurves();
                    }
                    throw Exception("Could not generate the arcs of the B-spline curve!");
                }
            }
            if (point_index >=0 && point_index <= (int)k - 2)
            {
                for (GLuint r = n + 1; r <= n + k - 1 - point_index; r++)
                {
                    GLuint ii = point_index + r;
                    if ((*arcs)[ii - offset])
                    {
                        delete (*arcs)[ii - offset];
                        (*arcs)[ii - offset] = nullptr;
                    }

                    (*arcs)[ii - offset] = bs->GenerateImageOfAnArc(ii, 2, div_point_count);

                    if (!(*arcs)[ii - offset] || !(*arcs)[ii - offset]->UpdateVertexBufferObjects(_scale_of_vectors))
                    {
                        if (_selected_model == ONE_VARIABLE)
                        {
                            deleteAllOneVariablePointCloudRegressions();
                        }
                        else
                        {
                            deleteAllBSplineCurves();
                        }
                        throw Exception("Could not generate the arcs of the B-spline curve!");
                    }
                }
            }
        }
    }

    bool PointCloudsAndModels::set_control_points(int value)
    {
        if (_selected_type != ONE_VARIABLE)
        {
            return false;
        }

        if (_one_var_point_clouds[_selected_model]._n == value)
            return false;
        _one_var_point_clouds[_selected_model]._n = value;

        deleteOneVariablePointCloudRegression(_selected_model);
        createOneVariablePointCloudRegression(_selected_model);

        return true;
    }

    bool PointCloudsAndModels::set_knotvector_type(int index)
    {
        if (_selected_type != ONE_VARIABLE)
        {
            return false;
        }

        KnotVector::Type &_type = _one_var_point_clouds[_selected_model]._type;
        switch(index)
        {
        case 0:
            if (_type == KnotVector::PERIODIC)
                return false;
            _type = KnotVector::PERIODIC;
            break;
        case 1:
            if (_type == KnotVector::CLAMPED)
                return false;
            _type = KnotVector::CLAMPED;
            break;
        case 2:
            if (_type == KnotVector::UNCLAMPED)
                return false;
            _type = KnotVector::UNCLAMPED;
            break;
        }

        deleteOneVariablePointCloudRegression(_selected_model);
        createOneVariablePointCloudRegression(_selected_model);

        return true;
    }

    bool PointCloudsAndModels::set_knotvector_order(int value)
    {
        if (_selected_type != ONE_VARIABLE)
        {
            return false;
        }

        GLuint &_k = _one_var_point_clouds[_selected_model]._k;
        if (_k == value)
            return false;
        if (_one_var_point_clouds[_selected_model]._type != KnotVector::PERIODIC
                && value > static_cast<int> (_one_var_point_clouds[_selected_model]._n + 1))
            return false;

        _k = value;

        deleteOneVariablePointCloudRegression(_selected_model);
        createOneVariablePointCloudRegression(_selected_model);

        return true;
    }

    bool PointCloudsAndModels::set_div_point_coint(int value)
    {
        if (_selected_type == ONE_VARIABLE)
        {
            if (_one_var_point_clouds[_selected_model]._div_point_count == value)
                return false;
            _one_var_point_clouds[_selected_model]._div_point_count = value;

            deleteOneVariablePointCloudRegression(_selected_model);
            createOneVariablePointCloudRegression(_selected_model);

            return true;
        }

        if (_selected_type == CURVE)
        {
            if (_curves[_selected_model]._div_point_count == value)
                return false;
            _curves[_selected_model]._div_point_count = value;

            deleteBSplineCurve(_selected_model);
            createImageOfBSplineCurve(_selected_model);

            return true;
        }

        return false;
    }

    bool PointCloudsAndModels::set_scale_of_vectors(double value)
    {
        if (value == _scale_of_vectors)
        {
            return false;
        }
        _scale_of_vectors = value;

        for (GLuint c = 0; c < _one_var_point_clouds.GetColumnCount(); c++)
        {

            if (!_one_var_point_clouds[c]._img_bs->UpdateVertexBufferObjects(_scale_of_vectors))
            {
                deleteAllOneVariablePointCloudRegressions();
                throw Exception("Could not update the VBO of the B-spline curve regression!");
            }

            for (GLuint i = 0; i < _one_var_point_clouds[c]._arcs->GetColumnCount(); i++)
            {
                if (!(*_one_var_point_clouds[c]._arcs)[i]->UpdateVertexBufferObjects(_scale_of_vectors))
                {
                    deleteAllOneVariablePointCloudRegressions();
                    throw Exception("Could not update the VBO of all arcs of the B-spline curve regression!");
                }
            }
        }

        for (GLuint c = 0; c < _curves.GetColumnCount(); c++)
        {

            if (!_curves[c]._img_bs->UpdateVertexBufferObjects(_scale_of_vectors))
            {
                deleteAllBSplineCurves();
                throw Exception("Could not update the VBO of the classic B-spline curve!");
            }

            for (GLuint i = 0; i < _curves[c]._arcs->GetColumnCount(); i++)
            {
                if (!(*_curves[c]._arcs)[i]->UpdateVertexBufferObjects(_scale_of_vectors))
                {
                    deleteAllBSplineCurves();
                    throw Exception("Could not update the VBO of all arcs of the classic B-spline curve!");
                }
            }
        }
        return true;
    }

    bool PointCloudsAndModels::set_scale_of_comb_vectors(double value)
    {
        if (value == _scale_of_comb_vectors)
            return false;
        _scale_of_comb_vectors = value;
        return true;
    }

    bool PointCloudsAndModels::set_count_of_comb_vectors(int value)
    {
        if (value == _count_of_comb_vectors)
            return false;
        _count_of_comb_vectors = value;
        return true;
    }

    DCoordinate3* PointCloudsAndModels::get_control_point_of_curve(int index)
    {
        if (_selected_type != ONE_VARIABLE && _selected_type != CURVE)
        {
            return new DCoordinate3(0.0, 0.0, 0.0);
        }

        return _selected_type == ONE_VARIABLE ? &(*_one_var_point_clouds[_selected_model]._bs)[index] : &(*_curves[_selected_model]._bs)[index];
    }

    RowMatrix<GLdouble> PointCloudsAndModels::get_total_energies_of_selected_curve()
    {
        if (_selected_type != ONE_VARIABLE && _selected_type != CURVE)
        {
            return RowMatrix<GLdouble>(0);
        }

        return _selected_type == ONE_VARIABLE ? _one_var_point_clouds[_selected_model]._bs->TotalEnergies(300) :
                                                _curves[_selected_model]._bs->TotalEnergies(300);
    }

    bool PointCloudsAndModels::show_curve(bool value)
    {
        if (_show_curve == value)
            return false;
        _show_curve = value;

        return true;
    }

    bool PointCloudsAndModels::show_arcs(bool value)
    {
        if (_show_arcs == value)
            return false;
        _show_arcs = value;

        return true;
    }

    bool PointCloudsAndModels::show_tangents(bool value)
    {
        if (_show_tangents == value)
            return false;
        _show_tangents = value;

        return true;
    }

    bool PointCloudsAndModels::show_acceleration_vectors(bool value)
    {
        if (_show_acceleration_vectors == value)
            return false;
        _show_acceleration_vectors = value;

        return true;
    }

    bool PointCloudsAndModels::show_comb(bool value)
    {
        if (_show_comb == value)
            return false;
        _show_comb = value;
        return true;
    }

    // -------
    // Methods for regression and simple surfaces
    // -------
    void PointCloudsAndModels::updatePatchByOnePoint(int row, int column)
    {
        if (_selected_type == TWO_VARIABLE)
        {
            TwoVariablePointCloudAndItsRegression& sf = _two_var_point_clouds[_selected_model];
            if (!sf._patch->UpdateVertexBufferObjectsOfData())
            {
                deleteAllBSplineSurfaces();
                throw Exception ("Could not create the image of patch");
            }

            sf._img_patch = sf._patch->GenerateImage(sf._div_point_count_u, sf._div_point_count_v, sf._total_energies, _selected_color_sheme);
            sf._patch->UpdateColorShemeOfImage(*sf._img_patch, _selected_color_sheme);

            if (!sf._img_patch || !sf._img_patch->UpdateVertexBufferObjects())
            {
                deleteAllBSplineSurfaces();
                throw Exception("Cann't update vertex buffer objects of image of patch");
            }

            int offset_u = sf._k_u - 1;
            int offset_v = sf._k_v - 1;
            int start_index_u, final_index_u, start_index_v, final_index_v;

            if (sf._type_u != KnotVector::PERIODIC && sf._type_v != KnotVector::PERIODIC)
            {
                start_index_u = max(row, (int)sf._k_u - 1);
                start_index_v = max(column, (int)sf._k_v - 1);
                final_index_u = min(row + offset_u, (int)sf._n_u);
                final_index_v = min(column + offset_v, (int)sf._n_v);

                for (int i = start_index_u; i <= final_index_u; i++)
                {
                    for (int j = start_index_v; j <= final_index_v; j++)
                    {
                        updateOnePatchByIndexes(i - offset_u, j - offset_v, i, j);
                    }
                }
            }

            if (sf._type_u == KnotVector::PERIODIC && sf._type_v == KnotVector::PERIODIC)
            {
                start_index_u = max(row, (int)sf._k_u - 1);
                start_index_v = max(column, (int)sf._k_v - 1);
                final_index_u = row + offset_u;
                final_index_v = column + offset_v;

                for (int i = start_index_u; i <= final_index_u; i++)
                {
                    for (int j = start_index_v; j <= final_index_v; j++)
                    {
                        updateOnePatchByIndexes(i - offset_u, j - offset_v, i, j);
                    }
                }

                if (row >=0 && row <= (int)sf._k_u - 2)
                {
                    for (GLuint r = sf._n_u + 1; r <= sf._n_u + sf._k_u - 1 - row; r++)
                    {
                        GLuint ii = row + r;

                        for (int j = start_index_v; j <= final_index_v; j++)
                        {
                            updateOnePatchByIndexes(ii - offset_u, j - offset_v, ii, j);
                        }
                    }
                }

                if (column >=0 && column <= (int)sf._k_u - 2)
                {
                    for (int i = start_index_u; i <= final_index_u; i++)
                    {
                        for (GLuint r = sf._n_v + 1; r <= sf._n_v + sf._k_v - 1 - column; r++)
                        {
                            GLuint jj = column + r;
                            updateOnePatchByIndexes(i - offset_u, jj - offset_v, i, jj);
                        }
                    }
                }

                if (row >=0 && row <= (int)sf._k_u - 2 && column >=0 && column <= (int)sf._k_u - 2)
                {
                    for (GLuint r = sf._n_u + 1; r <= sf._n_u + sf._k_u - 1 - row; r++)
                    {
                        GLuint ii = row + r;
                        for (GLuint p = sf._n_v + 1; p <= sf._n_v + sf._k_v - 1 - column; p++)
                        {
                            GLuint jj = column + p;
                            updateOnePatchByIndexes(ii - offset_u, jj - offset_v, ii, jj);
                        }
                    }
                }
            }

            if (sf._type_u == KnotVector::PERIODIC && sf._type_v != KnotVector::PERIODIC)
            {
                start_index_u = max(row, (int)sf._k_u - 1);
                start_index_v = max(column, (int)sf._k_v - 1);
                final_index_u = row + offset_u;
                final_index_v = min(column + offset_v, (int)sf._n_v);

                for (int i = start_index_u; i <= final_index_u; i++)
                {
                    for (int j = start_index_v; j <= final_index_v; j++)
                    {
                        updateOnePatchByIndexes(i - offset_u, j - offset_v, i, j);
                    }
                }

                if (row >=0 && row <= (int)sf._k_u - 2)
                {
                    for (GLuint r = sf._n_u + 1; r <= sf._n_u + sf._k_u - 1 - row; r++)
                    {
                        GLuint ii = row + r;

                        for (int j = start_index_v; j <= final_index_v; j++)
                        {
                            updateOnePatchByIndexes(ii - offset_u, j - offset_v, ii, j);
                        }
                    }
                }
            }

            if (sf._type_u != KnotVector::PERIODIC && sf._type_v == KnotVector::PERIODIC)
            {
                start_index_u = max(row, (int)sf._k_u - 1);
                start_index_v = max(column, (int)sf._k_v - 1);
                final_index_u = min(row + offset_u, (int)sf._n_u);
                final_index_v = column + offset_v;

                for (int i = start_index_u; i <= final_index_u; i++)
                {
                    for (int j = start_index_v; j <= final_index_v; j++)
                    {
                        updateOnePatchByIndexes(i - offset_u, j - offset_v, i, j);
                    }
                }

                if (column >=0 && column <= (int)sf._k_u - 2)
                {
                    for (int i = start_index_u; i <= final_index_u; i++)
                    {
                        for (GLuint r = sf._n_v + 1; r <= sf._n_v + sf._k_v - 1 - column; r++)
                        {
                            GLuint jj = column + r;
                            updateOnePatchByIndexes(i - offset_u, jj - offset_v, i, jj);
                        }
                    }
                }
            }
        }

        if(_selected_type == SURFACE)
        {
            BSplineSurface& sf = _surfaces[_selected_model];
            if (!sf._patch->UpdateVertexBufferObjectsOfData())
            {
                deleteAllBSplineSurfaces();
                throw Exception ("Could not create the image of patch");
            }

            sf._img_patch = sf._patch->GenerateImage(sf._div_point_count_u, sf._div_point_count_v, sf._total_energies, _selected_color_sheme);
            sf._patch->UpdateColorShemeOfImage(*sf._img_patch, _selected_color_sheme);

            if (!sf._img_patch || !sf._img_patch->UpdateVertexBufferObjects())
            {
                deleteAllBSplineSurfaces();
                throw Exception("Cann't update vertex buffer objects of image of patch");
            }

            int offset_u = sf._k_u - 1;
            int offset_v = sf._k_v - 1;
            int start_index_u, final_index_u, start_index_v, final_index_v;

            if (sf._type_u != KnotVector::PERIODIC && sf._type_v != KnotVector::PERIODIC)
            {
                start_index_u = max(row, (int)sf._k_u - 1);
                start_index_v = max(column, (int)sf._k_v - 1);
                final_index_u = min(row + offset_u, (int)sf._n_u);
                final_index_v = min(column + offset_v, (int)sf._n_v);

                for (int i = start_index_u; i <= final_index_u; i++)
                {
                    for (int j = start_index_v; j <= final_index_v; j++)
                    {
                        updateOnePatchByIndexes(i - offset_u, j - offset_v, i, j);
                    }
                }
            }

            if (sf._type_u == KnotVector::PERIODIC && sf._type_v == KnotVector::PERIODIC)
            {
                start_index_u = max(row, (int)sf._k_u - 1);
                start_index_v = max(column, (int)sf._k_v - 1);
                final_index_u = row + offset_u;
                final_index_v = column + offset_v;

                for (int i = start_index_u; i <= final_index_u; i++)
                {
                    for (int j = start_index_v; j <= final_index_v; j++)
                    {
                        updateOnePatchByIndexes(i - offset_u, j - offset_v, i, j);
                    }
                }

                if (row >=0 && row <= (int)sf._k_u - 2)
                {
                    for (GLuint r = sf._n_u + 1; r <= sf._n_u + sf._k_u - 1 - row; r++)
                    {
                        GLuint ii = row + r;

                        for (int j = start_index_v; j <= final_index_v; j++)
                        {
                            updateOnePatchByIndexes(ii - offset_u, j - offset_v, ii, j);
                        }
                    }
                }

                if (column >=0 && column <= (int)sf._k_u - 2)
                {
                    for (int i = start_index_u; i <= final_index_u; i++)
                    {
                        for (GLuint r = sf._n_v + 1; r <= sf._n_v + sf._k_v - 1 - column; r++)
                        {
                            GLuint jj = column + r;
                            updateOnePatchByIndexes(i - offset_u, jj - offset_v, i, jj);
                        }
                    }
                }

                if (row >=0 && row <= (int)sf._k_u - 2 && column >=0 && column <= (int)sf._k_u - 2)
                {
                    for (GLuint r = sf._n_u + 1; r <= sf._n_u + sf._k_u - 1 - row; r++)
                    {
                        GLuint ii = row + r;
                        for (GLuint p = sf._n_v + 1; p <= sf._n_v + sf._k_v - 1 - column; p++)
                        {
                            GLuint jj = column + p;
                            updateOnePatchByIndexes(ii - offset_u, jj - offset_v, ii, jj);
                        }
                    }
                }
            }

            if (sf._type_u == KnotVector::PERIODIC && sf._type_v != KnotVector::PERIODIC)
            {
                start_index_u = max(row, (int)sf._k_u - 1);
                start_index_v = max(column, (int)sf._k_v - 1);
                final_index_u = row + offset_u;
                final_index_v = min(column + offset_v, (int)sf._n_v);

                for (int i = start_index_u; i <= final_index_u; i++)
                {
                    for (int j = start_index_v; j <= final_index_v; j++)
                    {
                        updateOnePatchByIndexes(i - offset_u, j - offset_v, i, j);
                    }
                }

                if (row >=0 && row <= (int)sf._k_u - 2)
                {
                    for (GLuint r = sf._n_u + 1; r <= sf._n_u + sf._k_u - 1 - row; r++)
                    {
                        GLuint ii = row + r;

                        for (int j = start_index_v; j <= final_index_v; j++)
                        {
                            updateOnePatchByIndexes(ii - offset_u, j - offset_v, ii, j);
                        }
                    }
                }
            }

            if (sf._type_u != KnotVector::PERIODIC && sf._type_v == KnotVector::PERIODIC)
            {
                start_index_u = max(row, (int)sf._k_u - 1);
                start_index_v = max(column, (int)sf._k_v - 1);
                final_index_u = min(row + offset_u, (int)sf._n_u);
                final_index_v = column + offset_v;

                for (int i = start_index_u; i <= final_index_u; i++)
                {
                    for (int j = start_index_v; j <= final_index_v; j++)
                    {
                        updateOnePatchByIndexes(i - offset_u, j - offset_v, i, j);
                    }
                }

                if (column >=0 && column <= (int)sf._k_u - 2)
                {
                    for (int i = start_index_u; i <= final_index_u; i++)
                    {
                        for (GLuint r = sf._n_v + 1; r <= sf._n_v + sf._k_v - 1 - column; r++)
                        {
                            GLuint jj = column + r;
                            updateOnePatchByIndexes(i - offset_u, jj - offset_v, i, jj);
                        }
                    }
                }
            }
        }
    }

    void PointCloudsAndModels::updateOnePatchByIndexes(int row, int column, int index_1, int index_2)
    {
        BSplinePatch3 *patch = _selected_model == TWO_VARIABLE ? _two_var_point_clouds[_selected_model]._patch :
                                                                 _surfaces[_selected_model]._patch;
        Matrix<TriangulatedMesh3*> *patches = _selected_model == TWO_VARIABLE ? _two_var_point_clouds[_selected_model]._patches :
                                                                 _surfaces[_selected_model]._patches;
        GLuint div_point_count_u = _selected_model == TWO_VARIABLE ? _two_var_point_clouds[_selected_model]._div_point_count_u :
                                                                     _surfaces[_selected_model]._div_point_count_u;
        GLuint div_point_count_v = _selected_type == TWO_VARIABLE ? _two_var_point_clouds[_selected_model]._div_point_count_v :
                                                                     _surfaces[_selected_model]._div_point_count_v;
        if ((*patches)(row, column))
        {
            delete (*patches)(row, column);
            (*patches)(row, column) = nullptr;
        }

        (*patches)(row, column) = patch->GenerateImageOfAnPatch(index_1, index_2, div_point_count_u, div_point_count_v, _selected_color_sheme);

        if (!(*patches)(row, column))
        {
            if (_selected_type == TWO_VARIABLE)
            {
                deleteAllTwoVariablePointCloudRegressions();
            }
            if (_selected_type == SURFACE)
            {
                deleteAllBSplineSurfaces();
            }
            throw Exception("Could not generate patch of surface!");
        }

        if (!(*patches)(row, column)->UpdateVertexBufferObjects())
        {
            if (_selected_type == TWO_VARIABLE)
            {
                deleteAllTwoVariablePointCloudRegressions();
            }
            if (_selected_type == SURFACE)
            {
                deleteAllBSplineSurfaces();
            }
            throw Exception("Could not update the VBO of all patches of the B-spline patch!");
        }
    }

    bool PointCloudsAndModels::set_knotvector_type_u(int index)
    {
        if (_selected_type != TWO_VARIABLE)
        {
            return false;
        }
        switch(index)
        {
        case 0:
            if (_two_var_point_clouds[_selected_model]._type_u == KnotVector::PERIODIC)
                return false;
            _two_var_point_clouds[_selected_model]._type_u = KnotVector::PERIODIC;
            break;
        case 1:
            if (_two_var_point_clouds[_selected_model]._type_u == KnotVector::CLAMPED)
                return false;
            _two_var_point_clouds[_selected_model]._type_u = KnotVector::CLAMPED;
            break;
        case 2:
            if (_two_var_point_clouds[_selected_model]._type_u == KnotVector::UNCLAMPED)
                return false;
            _two_var_point_clouds[_selected_model]._type_u = KnotVector::UNCLAMPED;
            break;
        }

        deleteTwoVariablePointCloudRegression(_selected_model);
        createTwoVariablePointCloudRegression(_selected_model);
        return true;
    }

    bool PointCloudsAndModels::set_knotvector_type_v(int index)
    {
        if (_selected_type != TWO_VARIABLE)
        {
            return false;
        }

        switch(index)
        {
        case 0:
            if (_two_var_point_clouds[_selected_model]._type_v == KnotVector::PERIODIC)
                return false;
            _two_var_point_clouds[_selected_model]._type_v = KnotVector::PERIODIC;
            break;
        case 1:
            if (_two_var_point_clouds[_selected_model]._type_v == KnotVector::CLAMPED)
                return false;
            _two_var_point_clouds[_selected_model]._type_v = KnotVector::CLAMPED;
            break;
        case 2:
            if (_two_var_point_clouds[_selected_model]._type_v == KnotVector::UNCLAMPED)
                return false;
            _two_var_point_clouds[_selected_model]._type_v = KnotVector::UNCLAMPED;
            break;
        }

        deleteTwoVariablePointCloudRegression(_selected_model);
        createTwoVariablePointCloudRegression(_selected_model);
        return true;
    }

    bool PointCloudsAndModels::set_knotvector_order_u(int value)
    {
        if (_selected_type != TWO_VARIABLE)
        {
            return false;
        }

        TwoVariablePointCloudAndItsRegression& sf = _two_var_point_clouds[_selected_model];
        if (sf._k_u == value)
            return false;
        if (sf._type_u != KnotVector::PERIODIC && value > static_cast<int> (sf._n_u + 1))
            return false;
        sf._k_u = value;

        deleteTwoVariablePointCloudRegression(_selected_model);
        createTwoVariablePointCloudRegression(_selected_model);
        return true;
    }

    bool PointCloudsAndModels::set_knotvector_order_v(int value)
    {
        if (_selected_type != TWO_VARIABLE)
        {
            return false;
        }

        TwoVariablePointCloudAndItsRegression& sf = _two_var_point_clouds[_selected_model];
        if (sf._k_v == value)
            return false;
        if (sf._type_v != KnotVector::PERIODIC && value > static_cast<int> (sf._n_v + 1))
            return false;
        sf._k_v = value;

        deleteTwoVariablePointCloudRegression(_selected_model);
        createTwoVariablePointCloudRegression(_selected_model);
        return true;
    }

    bool PointCloudsAndModels::set_control_points_u(int value)
    {
        if (_selected_type != TWO_VARIABLE)
        {
            return false;
        }

        TwoVariablePointCloudAndItsRegression& sf = _two_var_point_clouds[_selected_model];
        if (sf._n_u == value)
            return false;
        sf._n_u = value;

        deleteTwoVariablePointCloudRegression(_selected_model);
        createTwoVariablePointCloudRegression(_selected_model);
        return true;
    }

    bool PointCloudsAndModels::set_control_points_v(int value)
    {
        if (_selected_type != TWO_VARIABLE)
        {
            return false;
        }

        TwoVariablePointCloudAndItsRegression& sf = _two_var_point_clouds[_selected_model];
        if (sf._n_v == value)
            return false;
        sf._n_v = value;

        deleteTwoVariablePointCloudRegression(_selected_model);
        createTwoVariablePointCloudRegression(_selected_model);
        return true;
    }

    bool PointCloudsAndModels::set_div_point_coint_u(int value)
    {
        if (_selected_type == TWO_VARIABLE)
        {
            TwoVariablePointCloudAndItsRegression& sf = _two_var_point_clouds[_selected_model];
            if (sf._div_point_count_u == value)
                return false;
            sf._div_point_count_u = value;

            deleteTwoVariablePointCloudRegression(_selected_model);
            createTwoVariablePointCloudRegression(_selected_model);
            return true;
        }

        if (_selected_type == SURFACE)
        {
            BSplineSurface& sf = _surfaces[_selected_model];
            if (sf._div_point_count_u == value)
                return false;
            sf._div_point_count_u = value;

            deleteBSplineSurface(_selected_model);
            createImageOfBSplineSurface(_selected_model);
            return true;
        }

        return false;
    }

    bool PointCloudsAndModels::set_div_point_coint_v(int value)
    {if (_selected_type == TWO_VARIABLE)
        {
            TwoVariablePointCloudAndItsRegression& sf = _two_var_point_clouds[_selected_model];
            if (sf._div_point_count_v == value)
                return false;
            sf._div_point_count_v = value;

            deleteTwoVariablePointCloudRegression(_selected_model);
            createTwoVariablePointCloudRegression(_selected_model);
            return true;
        }

        if (_selected_type == SURFACE)
        {
            BSplineSurface& sf = _surfaces[_selected_model];
            if (sf._div_point_count_v == value)
                return false;
            sf._div_point_count_v = value;

            deleteBSplineSurface(_selected_model);
            createImageOfBSplineSurface(_selected_model);
            return true;
        }

        return false;
    }

    void PointCloudsAndModels::set_shader_available(bool value)
    {
        _no_shader = !value;
    }

    bool PointCloudsAndModels::set_color_sheme(int index)
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

        for (GLuint pc = 0; pc < _two_var_point_clouds.GetColumnCount(); pc++)
        {
            _two_var_point_clouds[pc]._patch->UpdateColorShemeOfImage(*_two_var_point_clouds[pc]._img_patch, _selected_color_sheme);

            if (!_two_var_point_clouds[pc]._img_patch || !_two_var_point_clouds[pc]._img_patch->UpdateVertexBufferObjects())
            {
                deleteAllTwoVariablePointCloudRegressions();
                throw Exception("Could not create image of B-spline patch!");
            }

            _two_var_point_clouds[pc]._patches = _two_var_point_clouds[pc]._patch->GenerateImageOfPatches(_two_var_point_clouds[pc]._div_point_count_u, _two_var_point_clouds[pc]._div_point_count_v, _selected_color_sheme);
            if (!_two_var_point_clouds[pc]._patches)
            {
                deleteAllTwoVariablePointCloudRegressions();
                throw Exception("Could not generate the patches of the B-spline patch!");
            }

            for (GLuint i = 0; i < _two_var_point_clouds[pc]._patches->GetRowCount(); i++)
            {
                for (GLuint j = 0; j < _two_var_point_clouds[pc]._patches->GetColumnCount(); j++)
                {
                    if (!(*_two_var_point_clouds[pc]._patches)(i, j)->UpdateVertexBufferObjects())
                    {
                        deleteAllTwoVariablePointCloudRegressions();
                        throw Exception("Could not update the VBO of all patches of the B-spline patch!");
                    }
                }
            }
        }

        for (GLuint pc = 0; pc < _surfaces.GetColumnCount(); pc++)
        {
            _surfaces[pc]._patch->UpdateColorShemeOfImage(*_surfaces[pc]._img_patch, _selected_color_sheme);

            if (!_surfaces[pc]._img_patch || !_surfaces[pc]._img_patch->UpdateVertexBufferObjects())
            {
                deleteAllBSplineSurfaces();
                throw Exception("Could not create image of B-spline patch!");
            }

            _surfaces[pc]._patches = _surfaces[pc]._patch->GenerateImageOfPatches(_surfaces[pc]._div_point_count_u, _surfaces[pc]._div_point_count_v, _selected_color_sheme);
            if (!_surfaces[pc]._patches)
            {
                deleteAllBSplineSurfaces();
                throw Exception("Could not generate the patches of the B-spline patch!");
            }

            for (GLuint i = 0; i < _surfaces[pc]._patches->GetRowCount(); i++)
            {
                for (GLuint j = 0; j < _surfaces[pc]._patches->GetColumnCount(); j++)
                {
                    if (!(*_surfaces[pc]._patches)(i, j)->UpdateVertexBufferObjects())
                    {
                        deleteAllBSplineSurfaces();
                        throw Exception("Could not update the VBO of all patches of the B-spline patch!");
                    }
                }
            }
        }

        return true;
    }

    DCoordinate3* PointCloudsAndModels::get_control_point_of_surface(int row, int column)
    {
        if (_selected_type != TWO_VARIABLE && _selected_type != SURFACE)
        {
            return new DCoordinate3(0.0, 0.0, 0.0);
        }

        return _selected_type == TWO_VARIABLE ? &(*_two_var_point_clouds[_selected_model]._patch)(row, column) : &(*_surfaces[_selected_model]._patch)(row, column);
    }

    RowMatrix<GLdouble> PointCloudsAndModels::get_total_energies_of_surface()
    {
        if (_selected_type != TWO_VARIABLE && _selected_type != SURFACE)
        {
            return RowMatrix<GLdouble>(0);
        }

        return _selected_type == TWO_VARIABLE ? _two_var_point_clouds[_selected_model]._total_energies : _surfaces[_selected_model]._total_energies ;
    }

    bool PointCloudsAndModels::show_heat_map(bool value)
    {
        if (_show_heat_map == value)
            return false;
        _show_heat_map = value;
        return true;
    }

    bool PointCloudsAndModels::show_patches(bool value)
    {
        if (_show_patches == value)
            return false;
        _show_patches = value;
        return true;
    }

    bool PointCloudsAndModels::show_patch(bool value)
    {
        if (_show_patch == value)
            return false;
        _show_patch = value;
        return true;
    }

    // -------
    // Common methods
    // -------
    bool PointCloudsAndModels::set_weight_size(int value)
    {
        if (_selected_type == ONE_VARIABLE)
        {
            if (value == _one_var_point_clouds[_selected_model]._weight.GetColumnCount())
                return false;
            _one_var_point_clouds[_selected_model]._weight.ResizeColumns(value);
            return true;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            if (value == _two_var_point_clouds[_selected_model]._weight.GetColumnCount())
                return false;
            _two_var_point_clouds[_selected_model]._weight.ResizeColumns(value);
            return true;
        }
        return false;
    }

    bool PointCloudsAndModels::set_weight_value(int index, double value)
    {
        if (_selected_type == ONE_VARIABLE)
        {
            if (_one_var_point_clouds[_selected_model]._weight[index] == value)
                return false;
            _one_var_point_clouds[_selected_model]._weight[index] = value;

            deleteOneVariablePointCloudRegression(_selected_model);
            createOneVariablePointCloudRegression(_selected_model);
            return true;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            if (_two_var_point_clouds[_selected_model]._weight[index] == value)
                return false;
            _two_var_point_clouds[_selected_model]._weight[index] = value;

            deleteTwoVariablePointCloudRegression(_selected_model);
            createTwoVariablePointCloudRegression(_selected_model);
            return true;
        }
        return false;
    }

    bool PointCloudsAndModels::set_cloud_point_size(double value)
    {
        if (_cloud_point_size == value)
            return false;

        _cloud_point_size = value;
        return true;
    }

    bool PointCloudsAndModels::set_control_point_size(double value)
    {
        if (_control_point_size == value)
            return false;

        _control_point_size = value;
        return true;
    }

    bool PointCloudsAndModels::show_control_polygon(bool value)
    {
        if (_show_control_polygon == value)
            return false;
        _show_control_polygon = value;

        return true;
    }

    bool PointCloudsAndModels::show_cloud(bool value)
    {
        if (_show_cloud == value)
            return false;
        _show_cloud = value;
        return true;
    }

    GLuint PointCloudsAndModels::get_named_objects_count_of_one_var_point_clouds()
    {
        GLuint size = 0;
        for (GLuint i = 0; i < _one_var_point_clouds.GetColumnCount(); i++)
        {
            size += _one_var_point_clouds[i]._n + 2;
        }
        return size;
    }

    GLuint PointCloudsAndModels::get_named_objects_count_of_bspline_curves()
    {
        GLuint size = 0;
        for (GLuint i = 0; i < _curves.GetColumnCount(); i++)
        {
            size += _curves[i]._n + 2;
        }
        return size;
    }

    GLuint PointCloudsAndModels::get_named_objects_count_of_two_var_point_clouds()
    {
        GLuint size = 0;
        for (GLuint i = 0; i < _two_var_point_clouds.GetColumnCount(); i++)
        {
            size += (_two_var_point_clouds[i]._n_u + 1)*(_two_var_point_clouds[i]._n_v + 1) + 1;
        }
        return size;
    }

    GLuint PointCloudsAndModels::get_named_objects_count_of_bspline_surfaces()
    {
        GLuint size = 0;
        for (GLuint i = 0; i < _surfaces.GetColumnCount(); i++)
        {
            size += (_surfaces[i]._n_u + 1)*(_surfaces[i]._n_v + 1) + 1;
        }
        return size;
    }

    // return true -> clicked object is the rendered point cloud and regression, or model
    // return false -> clicked object is a control point
    bool PointCloudsAndModels::find_and_select_clicked_object(GLuint name, GLint &row, int &column)
    {
        GLuint offset = 0;
        GLuint one_var = get_named_objects_count_of_one_var_point_clouds();
        GLuint curves = get_named_objects_count_of_bspline_curves();
        GLuint two_var = get_named_objects_count_of_two_var_point_clouds();
        if (one_var != 0 && name < one_var)
        {
            _selected_type = ONE_VARIABLE;
            for (GLuint i = 0; i < _one_var_point_clouds.GetColumnCount(); i++)
            {
                _selected_model = i;
                if (name == offset + _one_var_point_clouds[i]._n + 1)
                {
                    column = 0;
                    row = 0;
                    return true;
                }
                if (name < offset + _one_var_point_clouds[i]._n + 1)
                {
                    column = name - offset;
                    row = 0;
                    return false;
                }
                offset += _one_var_point_clouds[i]._n + 2;
            }
        }
        offset = one_var;
        if (curves != 0 && name < offset + curves)
        {
            _selected_type = CURVE;
            for (GLuint i = 0; i < _curves.GetColumnCount(); i++)
            {
                _selected_model = i;
                if (name == offset + _curves[i]._n + 1)
                {
                    column = 0;
                    row = 0;
                    return true;
                }
                if (name < offset + _curves[i]._n + 1)
                {
                    column = name - offset;
                    row = 0;
                    return false;
                }
                offset += _curves[i]._n + 2;
            }
        }
        offset = one_var + curves;
        if (two_var != 0 && name < offset + two_var)
        {
            _selected_type = TWO_VARIABLE;
            for (GLuint i = 0; i < _two_var_point_clouds.GetColumnCount(); i++)
            {
                _selected_model = i;
                if (name == offset + (_two_var_point_clouds[i]._n_u + 1)*(_two_var_point_clouds[i]._n_v + 1))
                {
                    column = 0;
                    row = 0;
                    return true;
                }
                if (name < offset + (_two_var_point_clouds[i]._n_u + 1)*(_two_var_point_clouds[i]._n_v + 1))
                {

                    row = (name - offset) / (_two_var_point_clouds[i]._n_v + 1);
                    column = (name - offset) % (_two_var_point_clouds[i]._n_v + 1);
                    return false;
                }
                offset += (_two_var_point_clouds[i]._n_u + 1)*(_two_var_point_clouds[i]._n_v + 1) + 1;
            }
        }
        else
        {
            _selected_type = SURFACE;
            offset += get_named_objects_count_of_two_var_point_clouds();

            for (GLuint i = 0; i < _surfaces.GetColumnCount(); i++)
            {
                _selected_model = i;
                if (name == offset + (_surfaces[i]._n_u + 1)*(_surfaces[i]._n_v + 1))
                {
                    column = 0;
                    row = 0;
                    return true;
                }
                if (name < offset + (_surfaces[i]._n_u + 1)*(_surfaces[i]._n_v + 1))
                {

                    row = (name - offset) / (_surfaces[i]._n_v + 1);
                    column = (name - offset) % (_surfaces[i]._n_v + 1);
                    return false;
                }
                offset += (_surfaces[i]._n_u + 1)*(_surfaces[i]._n_v + 1) + 1;
            }

        }

        return false;
    }


    bool PointCloudsAndModels::set_angle_x(int value)
    {
        if (_selected_type == ONE_VARIABLE)
        {
            if (_one_var_point_clouds[_selected_model]._angle_x == value)
                return false;
            _one_var_point_clouds[_selected_model]._angle_x = value;
        }
        if (_selected_type == CURVE)
        {
            if (_curves[_selected_model]._angle_x == value)
                return false;
            _curves[_selected_model]._angle_x = value;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            if (_two_var_point_clouds[_selected_model]._angle_x == value)
                return false;
            _two_var_point_clouds[_selected_model]._angle_x = value;
        }
        if (_selected_type == SURFACE)
        {
            if (_surfaces[_selected_model]._angle_x == value)
                return false;
            _surfaces[_selected_model]._angle_x = value;
        }
        return true;
    }

    bool PointCloudsAndModels::set_angle_y(int value)
    {
        if (_selected_type == ONE_VARIABLE)
        {
            if (_one_var_point_clouds[_selected_model]._angle_y == value)
                return false;
            _one_var_point_clouds[_selected_model]._angle_y = value;
        }
        if (_selected_type == CURVE)
        {
            if (_curves[_selected_model]._angle_y == value)
                return false;
            _curves[_selected_model]._angle_y = value;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            if (_two_var_point_clouds[_selected_model]._angle_y == value)
                return false;
            _two_var_point_clouds[_selected_model]._angle_y = value;
        }
        if (_selected_type == SURFACE)
        {
            if (_surfaces[_selected_model]._angle_y == value)
                return false;
            _surfaces[_selected_model]._angle_y = value;
        }
        return true;
    }

    bool PointCloudsAndModels::set_angle_z(int value)
    {
        if (_selected_type == ONE_VARIABLE)
        {
            if (_one_var_point_clouds[_selected_model]._angle_z == value)
                return false;
            _one_var_point_clouds[_selected_model]._angle_z = value;
        }
        if (_selected_type == CURVE)
        {
            if (_curves[_selected_model]._angle_z == value)
                return false;
            _curves[_selected_model]._angle_z = value;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            if (_two_var_point_clouds[_selected_model]._angle_z == value)
                return false;
            _two_var_point_clouds[_selected_model]._angle_z = value;
        }
        if (_selected_type == SURFACE)
        {
            if (_surfaces[_selected_model]._angle_z == value)
                return false;
            _surfaces[_selected_model]._angle_z = value;
        }
        return true;
    }


    bool PointCloudsAndModels::set_scale_factor(double value)
    {
        if (_selected_type == ONE_VARIABLE)
        {
            if (_one_var_point_clouds[_selected_model]._scale == value)
                return false;
            _one_var_point_clouds[_selected_model]._scale = value;
        }
        if (_selected_type == CURVE)
        {
            if (_curves[_selected_model]._scale == value)
                return false;
            _curves[_selected_model]._scale = value;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            if (_two_var_point_clouds[_selected_model]._scale == value)
                return false;
            _two_var_point_clouds[_selected_model]._scale = value;
        }
        if (_selected_type == SURFACE)
        {
            if (_surfaces[_selected_model]._scale == value)
                return false;
            _surfaces[_selected_model]._scale = value;
        }
        return true;
    }


    bool PointCloudsAndModels::set_trans_x(double value)
    {
        if (_selected_type == ONE_VARIABLE)
        {
            if (_one_var_point_clouds[_selected_model]._trans_x == value)
                return false;
            _one_var_point_clouds[_selected_model]._trans_x = value;
        }
        if (_selected_type == CURVE)
        {
            if (_curves[_selected_model]._trans_x == value)
                return false;
            _curves[_selected_model]._trans_x = value;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            if (_two_var_point_clouds[_selected_model]._trans_x == value)
                return false;
            _two_var_point_clouds[_selected_model]._trans_x = value;
        }
        if (_selected_type == SURFACE)
        {
            if (_surfaces[_selected_model]._trans_x == value)
                return false;
            _surfaces[_selected_model]._trans_x = value;
        }
        return true;
    }

    bool PointCloudsAndModels::set_trans_y(double value)
    {
        if (_selected_type == ONE_VARIABLE)
        {
            if (_one_var_point_clouds[_selected_model]._trans_y == value)
                return false;
            _one_var_point_clouds[_selected_model]._trans_y = value;
        }
        if (_selected_type == CURVE)
        {
            if (_curves[_selected_model]._trans_y == value)
                return false;
            _curves[_selected_model]._trans_y = value;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            if (_two_var_point_clouds[_selected_model]._trans_y == value)
                return false;
            _two_var_point_clouds[_selected_model]._trans_y = value;
        }
        if (_selected_type == SURFACE)
        {
            if (_surfaces[_selected_model]._trans_y == value)
                return false;
            _surfaces[_selected_model]._trans_y = value;
        }
        return true;
    }

    bool PointCloudsAndModels::set_trans_z(double value)
    {
        if (_selected_type == ONE_VARIABLE)
        {
            if (_one_var_point_clouds[_selected_model]._trans_z == value)
                return false;
            _one_var_point_clouds[_selected_model]._trans_z = value;
        }
        if (_selected_type == CURVE)
        {
            if (_curves[_selected_model]._trans_z == value)
                return false;
            _curves[_selected_model]._trans_z = value;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            if (_two_var_point_clouds[_selected_model]._trans_z == value)
                return false;
            _two_var_point_clouds[_selected_model]._trans_z = value;
        }
        if (_selected_type == SURFACE)
        {
            if (_surfaces[_selected_model]._trans_z == value)
                return false;
            _surfaces[_selected_model]._trans_z = value;
        }
        return true;
    }

    // -------
    // Methods for update gui
    // -------
    int PointCloudsAndModels::get_control_points()
    {
        return _selected_type == ONE_VARIABLE ? _one_var_point_clouds[_selected_model]._n : 1;
    }

    int PointCloudsAndModels::get_knotvector_type()
    {
        if (_selected_type != ONE_VARIABLE)
            return 0;

        KnotVector::Type _type = _one_var_point_clouds[_selected_model]._type;
        switch(_type)
        {
        case KnotVector::PERIODIC:
            return 0;
            break;
        case KnotVector::CLAMPED:
            return 1;
            break;
        case KnotVector::UNCLAMPED:
            return 2;
            break;
        }
        return 0;
    }

    int PointCloudsAndModels::get_knotvector_order()
    {
        return _selected_type == ONE_VARIABLE ? _one_var_point_clouds[_selected_model]._k : 2;
    }

    int PointCloudsAndModels::get_div_point_coint()
    {
        if (_selected_type == ONE_VARIABLE)
            return _one_var_point_clouds[_selected_model]._div_point_count;
        if (_selected_type == CURVE)
            return _curves[_selected_model]._div_point_count;
        return 0;
    }

    int PointCloudsAndModels::get_knotvector_type_u()
    {
        if (_selected_type != TWO_VARIABLE)
            return 0;

        KnotVector::Type _type_u = _two_var_point_clouds[_selected_model]._type_u;
        switch(_type_u)
        {
        case KnotVector::PERIODIC:
            return 0;
            break;
        case KnotVector::CLAMPED:
            return 1;
            break;
        case KnotVector::UNCLAMPED:
            return 2;
            break;
        }
        return 0;
    }

    int PointCloudsAndModels::get_knotvector_type_v()
    {
        if (_selected_type != TWO_VARIABLE)
            return 0;

        KnotVector::Type _type_v = _two_var_point_clouds[_selected_model]._type_v;
        switch(_type_v)
        {
        case KnotVector::PERIODIC:
            return 0;
            break;
        case KnotVector::CLAMPED:
            return 1;
            break;
        case KnotVector::UNCLAMPED:
            return 2;
            break;
        }
        return 0;
    }

    int PointCloudsAndModels::get_knotvector_order_u()
    {
        return _selected_type == TWO_VARIABLE ? _two_var_point_clouds[_selected_model]._k_u : 2;
    }

    int PointCloudsAndModels::get_knotvector_order_v()
    {
        return _selected_type == TWO_VARIABLE ? _two_var_point_clouds[_selected_model]._k_v : 2;
    }

    int PointCloudsAndModels::get_control_points_u()
    {
        return _selected_type == TWO_VARIABLE ? _two_var_point_clouds[_selected_model]._n_u : 1;
    }

    int PointCloudsAndModels::get_control_points_v()
    {
        return _selected_type == TWO_VARIABLE ? _two_var_point_clouds[_selected_model]._n_v : 1;
    }

    int PointCloudsAndModels::get_div_point_coint_u()
    {
        if (_selected_type == TWO_VARIABLE)
            return _two_var_point_clouds[_selected_model]._div_point_count_u;
        if (_selected_type == SURFACE)
            return _surfaces[_selected_model]._div_point_count_u;
        return 0;
    }

    int PointCloudsAndModels::get_div_point_coint_v()
    {
        if (_selected_type == TWO_VARIABLE)
            return _two_var_point_clouds[_selected_model]._div_point_count_v;
        if (_selected_type == SURFACE)
            return _surfaces[_selected_model]._div_point_count_v;
        return 0;
    }

    int PointCloudsAndModels::get_weight_size()
    {
        return _selected_type == ONE_VARIABLE ? _one_var_point_clouds[_selected_model]._weight.GetColumnCount() :
                                                _two_var_point_clouds[_selected_model]._weight.GetColumnCount();
    }

    int PointCloudsAndModels::get_angle_x()
    {
        if (_selected_type == ONE_VARIABLE)
        {
            return _one_var_point_clouds[_selected_model]._angle_x;
        }
        if (_selected_type == CURVE)
        {
            return _curves[_selected_model]._angle_x;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            return _two_var_point_clouds[_selected_model]._angle_x;
        }
        if (_selected_type == SURFACE)
        {
            return _surfaces[_selected_model]._angle_x;
        }
        return 0;
    }

    int PointCloudsAndModels::get_angle_y()
    {
        if (_selected_type == ONE_VARIABLE)
        {
            return _one_var_point_clouds[_selected_model]._angle_y;
        }
        if (_selected_type == CURVE)
        {
            return _curves[_selected_model]._angle_y;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            return _two_var_point_clouds[_selected_model]._angle_y;
        }
        if (_selected_type == SURFACE)
        {
            return _surfaces[_selected_model]._angle_y;
        }
        return 0;
    }

    int PointCloudsAndModels::get_angle_z()
    {
        if (_selected_type == ONE_VARIABLE)
        {
            return _one_var_point_clouds[_selected_model]._angle_z;
        }
        if (_selected_type == CURVE)
        {
            return _curves[_selected_model]._angle_z;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            return _two_var_point_clouds[_selected_model]._angle_z;
        }
        if (_selected_type == SURFACE)
        {
            return _surfaces[_selected_model]._angle_z;
        }
        return 0;
    }


    double PointCloudsAndModels::get_scale_factor()
    {
        if (_selected_type == ONE_VARIABLE)
        {
            return _one_var_point_clouds[_selected_model]._scale;
        }
        if (_selected_type == CURVE)
        {
            return _curves[_selected_model]._scale;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            return _two_var_point_clouds[_selected_model]._scale;
        }
        if (_selected_type == SURFACE)
        {
            return _surfaces[_selected_model]._scale;
        }
        return 0;
    }


    double PointCloudsAndModels::get_trans_x()
    {
        if (_selected_type == ONE_VARIABLE)
        {
            return _one_var_point_clouds[_selected_model]._trans_x;
        }
        if (_selected_type == CURVE)
        {
            return _curves[_selected_model]._trans_x;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            return _two_var_point_clouds[_selected_model]._trans_x;
        }
        if (_selected_type == SURFACE)
        {
            return _surfaces[_selected_model]._trans_x;
        }
        return 0;
    }

    double PointCloudsAndModels::get_trans_y()
    {
        if (_selected_type == ONE_VARIABLE)
        {
            return _one_var_point_clouds[_selected_model]._trans_y;
        }
        if (_selected_type == CURVE)
        {
            return _curves[_selected_model]._trans_y;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            return _two_var_point_clouds[_selected_model]._trans_y;
        }
        if (_selected_type == SURFACE)
        {
            return _surfaces[_selected_model]._trans_y;
        }
        return 0;
    }

    double PointCloudsAndModels::get_trans_z()
    {
        if (_selected_type == ONE_VARIABLE)
        {
            return _one_var_point_clouds[_selected_model]._trans_z;
        }
        if (_selected_type == CURVE)
        {
            return _curves[_selected_model]._trans_z;
        }
        if (_selected_type == TWO_VARIABLE)
        {
            return _two_var_point_clouds[_selected_model]._trans_z;
        }
        if (_selected_type == SURFACE)
        {
            return _surfaces[_selected_model]._trans_z;
        }
        return 0;
    }


    RowMatrix<GLdouble> PointCloudsAndModels::get_weight()
    {
        return _selected_type == ONE_VARIABLE ? _one_var_point_clouds[_selected_model]._weight :
                                                _two_var_point_clouds[_selected_model]._weight;
    }

    PointCloudsAndModels::~PointCloudsAndModels()
    {
        deleteAllOneVariablePointCloudRegressions();
        deleteAllBSplineCurves();
        deleteAllTwoVariablePointCloudRegressions();
        deleteAllBSplineSurfaces();
    }

    void PointCloudsAndModels::loadOneVariablePointCloud(QTextStream &in)
    {
        GLint i = _one_var_point_clouds.GetColumnCount();
        _one_var_point_clouds.ResizeColumns(i + 1);
        _one_var_point_clouds[i]._cloud = new (nothrow) PointCloudAroundCurve3();

        in >> (*_one_var_point_clouds[i]._cloud);

        _one_var_point_clouds[i]._weight.ResizeColumns(0);

        _one_var_point_clouds[i]._cloud->FindTheInterval(
                    _one_var_point_clouds[i]._curve_u_min, _one_var_point_clouds[i]._curve_u_max);
        _selected_type = ONE_VARIABLE;
        _selected_model = i;
        createOneVariablePointCloudRegression(i);
    }

    void PointCloudsAndModels::loadTwoVariablePointCloud(QTextStream &in)
    {
        GLint i = _two_var_point_clouds.GetColumnCount();
        _two_var_point_clouds.ResizeColumns(i + 1);
        _two_var_point_clouds[i]._cloud = new (nothrow) PointCloudAroundSurface3();

        in >> (*_two_var_point_clouds[i]._cloud);

        _two_var_point_clouds[i]._weight.ResizeColumns(0);

        _two_var_point_clouds[i]._cloud->FindTheInterval(
                    _two_var_point_clouds[i]._surface_u_min, _two_var_point_clouds[i]._surface_u_max,
                    _two_var_point_clouds[i]._surface_v_min, _two_var_point_clouds[i]._surface_v_max);
        _selected_type = TWO_VARIABLE;
        _selected_model = i;
        createTwoVariablePointCloudRegression(i);
    }

    void PointCloudsAndModels::loadBSplineCurve(QTextStream &in)
    {
        GLint i = _curves.GetColumnCount();
        _curves.ResizeColumns(i + 1);

        _curves[i]._bs = new (nothrow) BSplineCurve3(KnotVector::PERIODIC, 4, 3);
        in >> (*_curves[i]._bs);

        _curves[i]._n = (*_curves[i]._bs).GetKnotVector()->GetN();
        _curves[i]._type = (*_curves[i]._bs).GetKnotVector()->GetType();
        _curves[i]._k = (*_curves[i]._bs).GetKnotVector()->GetOrder();

        _selected_type = CURVE;
        _selected_model = i;
        createImageOfBSplineCurve(i);
    }

    void PointCloudsAndModels::loadBSplineSurface(QTextStream &in)
    {
        GLint i = _surfaces.GetColumnCount();
        _surfaces.ResizeColumns(i + 1);

        _surfaces[i]._patch = new (nothrow) BSplinePatch3(KnotVector::PERIODIC, KnotVector::PERIODIC, 4, 4, 3,3);

        in >> *_surfaces[i]._patch;

        KnotVector* kv_u = (*_surfaces[i]._patch).GetKnotVectorU();
        _surfaces[i]._type_u = kv_u->GetType();
        _surfaces[i]._k_u = kv_u->GetOrder();
        _surfaces[i]._n_u = kv_u->GetN();

        KnotVector* kv_v = (*_surfaces[i]._patch).GetKnotVectorV();
        _surfaces[i]._type_v = kv_v->GetType();
        _surfaces[i]._k_v = kv_v->GetOrder();
        _surfaces[i]._n_v = kv_v->GetN();

        _selected_type = SURFACE;
        _selected_model = i;
        createImageOfBSplineSurface(i);
    }


    void PointCloudsAndModels::saveOneVariablePointCloud(QTextStream &out)
    {
        if (_selected_type != ONE_VARIABLE)
            QMessageBox::information(nullptr, "Saving one variable point cloud...", QString::fromStdString("The selected model must to be a one variable point cloud!"));
        out << (*_one_var_point_clouds[_selected_model]._cloud);
    }

    void PointCloudsAndModels::saveTwoVariablePointCloud(QTextStream &out)
    {
        if (_selected_type != TWO_VARIABLE)
            QMessageBox::information(nullptr, "Saving two variable point cloud...", QString::fromStdString("The selected model must to be a two variable point cloud!"));
        out << (*_two_var_point_clouds[_selected_model]._cloud);
    }

    void PointCloudsAndModels::saveBSplineCurve(QTextStream &out)
    {
        if (_selected_type != CURVE && _selected_type != ONE_VARIABLE)
            QMessageBox::information(nullptr, "Saving B-spline curve...", QString::fromStdString("The selected model must to be a B-spline curve (regression) model!"));
        out << ( _selected_type == CURVE ? (*_curves[_selected_model]._bs) : (*_one_var_point_clouds[_selected_model]._bs) );
    }

    void PointCloudsAndModels::saveBSplineSurface(QTextStream &out)
    {
        if (_selected_type != TWO_VARIABLE && _selected_type !=  SURFACE)
            QMessageBox::information(nullptr, "Saving B-spline surface...", QString::fromStdString("The selected model must to be a B-spline surface (regression) model!"));
        out << ( _selected_type == SURFACE ? (*_surfaces[_selected_model]._patch) : (*_two_var_point_clouds[_selected_model]._patch) );
    }

}
