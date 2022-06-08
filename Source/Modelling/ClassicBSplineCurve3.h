#pragma once

#include <B-spline/BSplineCurves3.h>
#include <Core/TriangulatedMeshes3.h>
#include <Core/Materials.h>

namespace cagd
{
    class ClassicBSplineCurve3
    {
        friend QTextStream& operator << (QTextStream& lhs, const ClassicBSplineCurve3& rhs);
        friend QTextStream& operator >> (QTextStream& lhs, ClassicBSplineCurve3& rhs);
    protected:
        KnotVector::Type                _type = KnotVector::PERIODIC;
        GLuint                          _n = 10, _k = 4;
        GLuint                          _div_point_count = 20;
        GLdouble                        _scale_of_vectors = 0.4;

        BSplineCurve3                   *_bs = nullptr;
        GenericCurve3                   *_img_bs = nullptr;
        RowMatrix<GenericCurve3*>       *_arcs = nullptr;

        double                          _point_size = 0.03;
        TriangulatedMesh3               _unit_sphere;

        bool                            _show_control_polygon = true;
        bool                            _show_curve = false;
        bool                            _show_arcs = true;
        bool                            _show_tangents = false;
        bool                            _show_acceleration_vectors = false;

    public:
        ClassicBSplineCurve3();

        virtual bool createBSplineCurve();
        bool renderBSplineCurve(bool dark_mode);
        void deleteBSplineCurve();

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
    };

    inline QTextStream& operator << (QTextStream& lhs, const ClassicBSplineCurve3& rhs)
    {
        lhs << (*rhs._bs);
        return lhs;
    }

    inline QTextStream& operator >> (QTextStream& lhs, ClassicBSplineCurve3& rhs)
    {
        lhs >> *rhs._bs;

        KnotVector* kv = (*rhs._bs).GetKnotVector();
        rhs._type = kv->GetType();
        rhs._k = kv->GetOrder();
        rhs._n = kv->GetN();

        if (!rhs._bs->UpdateVertexBufferObjectsOfData())
        {
            rhs.deleteBSplineCurve();
            throw Exception("Could not update the VBO's off the classic B-spline curve's control polygon");
        }

        rhs._img_bs = rhs._bs->GenerateImage(2, rhs._div_point_count);

        if (!rhs._img_bs)
        {
            rhs.deleteBSplineCurve();
            throw Exception("Could not create image of classic B-spline curve");
        }

        if (!rhs._img_bs->UpdateVertexBufferObjects(rhs._scale_of_vectors))
        {
            rhs.deleteBSplineCurve();
            throw Exception("Could not update the VBO's of the classic B-spline curve's imgae!");
        }

        rhs._arcs = rhs._bs->GenerateImageOfArcs(2, rhs._div_point_count);

        if (!rhs._arcs)
        {
            rhs.deleteBSplineCurve();
            throw Exception("Could not generate the arcs of the classic B-spline curve!");
        }

        for (GLuint i = 0; i <rhs._arcs->GetColumnCount(); i++)
        {
            if (!(*rhs._arcs)[i]->UpdateVertexBufferObjects(rhs._scale_of_vectors))
            {
                rhs.deleteBSplineCurve();
                throw Exception("Could not update the VBO of all arcs of the classic B-spline curve!");
            }
        }
        return lhs;
    }
}
