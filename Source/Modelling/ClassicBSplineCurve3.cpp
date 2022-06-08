#include "Modelling/ClassicBSplineCurve3.h"
#include <Core/Exceptions.h>
#include <Core/Constants.h>

using namespace std;

namespace cagd
{

ClassicBSplineCurve3::ClassicBSplineCurve3()
{
    if (!_unit_sphere.LoadFromOFF("Models/sphere.off"))
    {
        throw Exception("Could not load the model file: sphere.off!");
    }

    if (!_unit_sphere.UpdateVertexBufferObjects(GL_DYNAMIC_DRAW))
    {
        throw Exception("Could not update the VBOs of the sphere's triangulated mesh!");
    }
}

bool ClassicBSplineCurve3::createBSplineCurve()
{
    _bs = new (nothrow) BSplineCurve3(_type, _k, _n);

    if (!_bs)
    {
        deleteBSplineCurve();
        throw Exception("Could not create the classic B-spline curve");
    }

    // circle
    GLdouble step = TWO_PI / (_n + 1);
    for (GLuint i = 0; i <= _n; i++)
    {
        GLdouble u = i * step;
        DCoordinate3 &cp = (*_bs)[i];
        cp[0] = cos(u);
        cp[1] = sin(u);
        cp[2] = 0.0;
    }

    // simple curve
//    for (GLuint i = 0; i <= _n; i++)
//    {
//        GLdouble u = i * 0.4;
//        DCoordinate3 &cp = (*_bs)[i];
//        cp[0] = u;
//        cp[1] = (i % 3 == 0) ? u : -u;
//        cp[2] = 0.0;
//    }

    if (!_bs->UpdateVertexBufferObjectsOfData())
    {
        deleteBSplineCurve();
        throw Exception("Could not update the VBO's off the classic B-spline curve's control polygon");
    }

    _img_bs = _bs->GenerateImage(2, _div_point_count);

    if (!_img_bs)
    {
        deleteBSplineCurve();
        throw Exception("Could not create image of classic B-spline curve");
    }

    if (!_img_bs->UpdateVertexBufferObjects(_scale_of_vectors))
    {
        deleteBSplineCurve();
        throw Exception("Could not update the VBO's of the classic B-spline curve's imgae!");
    }

    _arcs = _bs->GenerateImageOfArcs(2, _div_point_count);

    if (!_arcs)
    {
        deleteBSplineCurve();
        throw Exception("Could not generate the arcs of the classic B-spline curve!");
    }

    for (GLuint i = 0; i < _arcs->GetColumnCount(); i++)
    {
        if (!(*_arcs)[i]->UpdateVertexBufferObjects(_scale_of_vectors))
        {
            deleteBSplineCurve();
            throw Exception("Could not update the VBO of all arcs of the classic B-spline curve!");
        }
    }

    return true;
}

bool ClassicBSplineCurve3::renderBSplineCurve(bool dark_mode)
{
    if (!_bs)
    {
        return false;
    }

    glLineWidth(2);
    if(_show_control_polygon)
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
        glColor3d(0.6, 0.0, 0.6);
        _bs->RenderData(_type == KnotVector::PERIODIC ? GL_LINE_LOOP : GL_LINE_STRIP);
    }


    if (_show_curve)
    {
        glColor3d(1.0, 0.0, 0.0);
        _img_bs->RenderDerivatives(0, _type == KnotVector::PERIODIC ? GL_LINE_LOOP : GL_LINE_STRIP);
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
                    glColor3d(1.0, 1.0, 0.0);
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

void ClassicBSplineCurve3::deleteBSplineCurve()
{
    if (_bs)
    {
        delete _bs;
        _bs = nullptr;
    }

    if (_img_bs)
    {
        delete _img_bs;
        _img_bs = nullptr;
    }

    for (GLuint i = 0; i < _arcs->GetColumnCount(); i++)
    {
        if ((*_arcs)[i])
        {
            delete (*_arcs)[i];
            (*_arcs)[i]=nullptr;
        }
    }
}

bool ClassicBSplineCurve3::set_control_points(int value)
{
    if (_n == value)
        return false;
    _n = value;

    deleteBSplineCurve();
    createBSplineCurve();

    return true;
}

bool ClassicBSplineCurve3::set_knotvector_type(int index)
{
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
    deleteBSplineCurve();
    createBSplineCurve();
    return true;
}

bool ClassicBSplineCurve3::set_knotvector_order(int value)
{
    if (_k == value)
        return false;
    if (_type != KnotVector::PERIODIC && value > static_cast<int> (_n + 1))
        return false;

    _k = value;

    deleteBSplineCurve();
    createBSplineCurve();

    return true;
}

bool ClassicBSplineCurve3::set_div_point_coint(int value)
{
    if (_div_point_count == value)
        return false;
    _div_point_count = value;

    deleteBSplineCurve();
    createBSplineCurve();
    return true;
}

bool ClassicBSplineCurve3::set_scale_of_vectors(double value)
{
    if (value == _scale_of_vectors)
    {
        return false;
    }
    _scale_of_vectors = value;

    if (!_img_bs->UpdateVertexBufferObjects(_scale_of_vectors))
    {
        deleteBSplineCurve();
        throw Exception("Could not update the VBO of the classic B-spline curve!");
    }

    for (GLuint i = 0; i < _arcs->GetColumnCount(); i++)
    {
        if (!(*_arcs)[i]->UpdateVertexBufferObjects(_scale_of_vectors))
        {
            deleteBSplineCurve();
            throw Exception("Could not update the VBO of all arcs of the classic B-spline curve!");
        }
    }
    return true;
}

bool ClassicBSplineCurve3::set_control_point_size(double value)
{
    if (_point_size == value)
        return false;

    _point_size = value;
    return true;
}

bool ClassicBSplineCurve3::show_control_polygon(bool value)
{
    if (_show_control_polygon == value)
        return false;
    _show_control_polygon = value;

    return true;
}

bool ClassicBSplineCurve3::show_curve(bool value)
{
    if (_show_curve == value)
        return false;
    _show_curve = value;

    return true;
}

bool ClassicBSplineCurve3::show_arcs(bool value)
{
    if (_show_arcs == value)
        return false;
    _show_arcs = value;

    return true;
}

bool ClassicBSplineCurve3::show_tangents(bool value)
{
    if (_show_tangents == value)
        return false;
    _show_tangents = value;

    return true;
}

bool ClassicBSplineCurve3::show_acceleration_vectors(bool value)
{
    if (_show_acceleration_vectors == value)
        return false;
    _show_acceleration_vectors = value;

    return true;
}

void ClassicBSplineCurve3::updateCurveByOnePoint(int value)
{
    if (!_bs->UpdateVertexBufferObjectsOfData())
    {
        deleteBSplineCurve();
        throw Exception("Could not update the VBO's pff the bspline curve's control polygon");
    }

    _img_bs = _bs->GenerateImage(2, _div_point_count);

    if (!_img_bs)
    {
        deleteBSplineCurve();
        throw Exception("Could not create image of bspline curve");
    }

    if (!_img_bs->UpdateVertexBufferObjects(0.4))
    {
        deleteBSplineCurve();
        throw Exception("Could not update the VBO's of the bspline curve's imgae!");
    }

    int offset = _k - 1;

    if (_type != KnotVector::PERIODIC)
    {
        int start_index = max(value, (int)_k - 1);
        int final_index = min(value + offset, (int)_n);
        for (int i = start_index; i <= final_index; i++)
        {
            if ((*_arcs)[i - offset])
            {
                delete (*_arcs)[i - offset];
                (*_arcs)[i - offset] = nullptr;
            }

            (*_arcs)[i - offset] = _bs->GenerateImageOfAnArc(i, 2, _div_point_count);

            if (!(*_arcs)[i - offset])
            {
                deleteBSplineCurve();
                throw Exception("Could not generate the arcs of the B-spline curve!");
            }

            if (!(*_arcs)[i - offset]->UpdateVertexBufferObjects(_scale_of_vectors))
            {
                deleteBSplineCurve();
                throw Exception("Could not update the VBO of all arcs of the B-spline curve!");
            }
        }
    }
    else
    {
        int start_index = max(value, (int)_k - 1);
        int final_index = value + offset;
        for (int i = start_index; i <= final_index; i++)
        {
            if ((*_arcs)[i - offset])
            {
                delete (*_arcs)[i - offset];
                (*_arcs)[i - offset] = nullptr;
            }

            (*_arcs)[i - offset] = _bs->GenerateImageOfAnArc(i, 2, _div_point_count);

            if (!(*_arcs)[i - offset])
            {
                deleteBSplineCurve();
                throw Exception("Could not generate the arcs of the B-spline curve!");
            }

            if (!(*_arcs)[i - offset]->UpdateVertexBufferObjects(_scale_of_vectors))
            {
                deleteBSplineCurve();
                throw Exception("Could not update the VBO of all arcs of the B-spline curve!");
            }


        }
        if (value >=0 && value <= (int)_k - 2)
        {
            for (GLuint r = _n + 1; r <= _n + _k - 1 - value; r++)
            {
                GLuint ii = value + r;
                if ((*_arcs)[ii - offset])
                {
                    delete (*_arcs)[ii - offset];
                    (*_arcs)[ii - offset] = nullptr;
                }

                (*_arcs)[ii - offset] = _bs->GenerateImageOfAnArc(ii, 2, _div_point_count);

                if (!(*_arcs)[ii - offset])
                {
                    deleteBSplineCurve();
                    throw Exception("Could not generate the arcs of the B-spline curve!");
                }

                if (!(*_arcs)[ii - offset]->UpdateVertexBufferObjects(_scale_of_vectors))
                {
                    deleteBSplineCurve();
                    throw Exception("Could not update the VBO of all arcs of the B-spline curve!");
                }
            }
        }
    }
}

GLuint ClassicBSplineCurve3::get_n()
{
    return _n;
}

DCoordinate3* ClassicBSplineCurve3::get_control_point(int index)
{
    return &(*_bs)[index];
}

}
