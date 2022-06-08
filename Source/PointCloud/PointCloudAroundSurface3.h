#pragma once

#include "Core/DCoordinates3.h"
#include "Core/Matrices.h"
#include "Parametric/ParametricSurfaces3.h"
#include "RandomNumberGenerator/NormalRNG.h"
#include "Core/Exceptions.h"
#include "B-spline/BSplinePatches3.h"
#include "Core/RealMatrices.h"

namespace cagd
{
    class PointCloudAroundSurface3
    {
    public:
        friend std::ostream& operator << (std::ostream& lhs, const PointCloudAroundSurface3& rhs);
        friend std::istream& operator >> (std::istream& lhs, PointCloudAroundSurface3& rhs);

        friend QTextStream& operator << (QTextStream& lhs, const PointCloudAroundSurface3& rhs);
        friend QTextStream& operator >> (QTextStream& lhs, PointCloudAroundSurface3& rhs);

        class SamplePoint
        {
        public:
            GLdouble parameter_value_u; // u
            GLdouble parameter_value_v; // v
            DCoordinate3 position;      // x
        };
    private:
        Matrix<SamplePoint> _cloud;
    public:
        PointCloudAroundSurface3();

        PointCloudAroundSurface3(const PointCloudAroundSurface3& rhs);

        PointCloudAroundSurface3& operator = (const PointCloudAroundSurface3& rhs);

        // Setting the _cloud
        bool GeneratePointCloudAroundParametricSurface(
                const ParametricSurface3 &ps,
                RowMatrix<GLdouble> sigma,
                GLuint u_sample_size, GLuint v_sample_size);

        // Setting the _cloud
        bool GeneratePointCloudAroundBSplineSurface(
                const BSplinePatch3 &bs,
                RowMatrix<GLdouble> sigma,
                GLuint u_sample_size, GLuint v_sample_size);

        // Render the points of cloud
        bool RenderPointCloud(TriangulatedMesh3 *sphere, double point_size, bool dark_mode = true, bool default_color = true);

        // Setting BSpline surface from cloud
        BSplinePatch3* GenerateRegressionSurface(const RowMatrix<GLdouble> &weight, KnotVector::Type u_type, KnotVector::Type v_type,
                                                 GLuint u_k, GLuint v_k,
                                                 GLuint u_n, GLuint v_n,
                                                 GLdouble u_min = 0.0, GLdouble u_max = 1.0,
                                                 GLdouble v_min = 0.0, GLdouble v_max = 1.0,
                                                 GLuint div_point_count = 300) const;

        void FindTheInterval(GLdouble &u_min, GLdouble &u_max, GLdouble &v_min, GLdouble &v_max);

    };

    inline std::ostream& operator << (std::ostream& lhs, const PointCloudAroundSurface3::SamplePoint& rhs)
    {
        lhs << rhs.parameter_value_u << " " << rhs.parameter_value_v << " " << rhs.position << "\n";
        return lhs;
    }

    inline std::istream& operator >> (std::istream& lhs, PointCloudAroundSurface3::SamplePoint& rhs)
    {
        lhs >> rhs.parameter_value_u >> rhs.parameter_value_v >> rhs.position;
        return lhs;
    }

    inline QTextStream& operator << (QTextStream& lhs, const PointCloudAroundSurface3::SamplePoint& rhs)
    {
        lhs << rhs.parameter_value_u << " " << rhs.parameter_value_v << " " << rhs.position << "\n";
        return lhs;
    }

    inline QTextStream& operator >> (QTextStream& lhs, PointCloudAroundSurface3::SamplePoint& rhs)
    {
        lhs >> rhs.parameter_value_u >> rhs.parameter_value_v >> rhs.position;
        return lhs;
    }

    inline std::ostream& operator << (std::ostream& lhs, const PointCloudAroundSurface3& rhs)
    {
        lhs << rhs._cloud;
        return lhs;
    }

    inline std::istream& operator >> (std::istream& lhs, PointCloudAroundSurface3& rhs)
    {
        lhs >> rhs._cloud;
        return lhs;
    }

    inline QTextStream& operator << (QTextStream& lhs, const PointCloudAroundSurface3& rhs)
    {
        lhs << rhs._cloud;
        return lhs;
    }

    inline QTextStream& operator >> (QTextStream& lhs, PointCloudAroundSurface3& rhs)
    {
        lhs >> rhs._cloud;
        return lhs;
    }

}
