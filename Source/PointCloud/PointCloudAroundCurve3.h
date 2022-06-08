#pragma once

#include "Core/DCoordinates3.h"
#include "Core/Matrices.h"
#include "Core/TriangulatedMeshes3.h"
#include "Parametric/ParametricCurves3.h"
#include "RandomNumberGenerator/NormalRNG.h"
#include "B-spline/BSplineCurves3.h"

#include <QTextStream>

using namespace std;

namespace cagd
{
    class PointCloudAroundCurve3
    {
    public:
        friend std::ostream& operator << (std::ostream& lhs, const PointCloudAroundCurve3& rhs);
        friend std::istream& operator >> (std::istream& lhs, PointCloudAroundCurve3& rhs);

        friend QTextStream& operator << (QTextStream& lhs, const PointCloudAroundCurve3& rhs);
        friend QTextStream& operator >> (QTextStream& lhs, PointCloudAroundCurve3& rhs);

        class SamplePoint
        {
        public:
            GLdouble parameter_value; // u
            DCoordinate3 position;    // x
        };

    private:
        RowMatrix<SamplePoint> _cloud;

    public:
        PointCloudAroundCurve3();

        PointCloudAroundCurve3(const PointCloudAroundCurve3& rhs);

        PointCloudAroundCurve3& operator =(const PointCloudAroundCurve3& rhs);

        // Setting the _cloud
        bool GeneratePointCloudAroundParametricCurve(
                const ParametricCurve3 &pc,
                RowMatrix<GLdouble> sigma,
                GLuint sample_size);

        // Render the points of cloud
        bool RenderPointCloud(TriangulatedMesh3 *sphere, double point_size, bool dark_mode = true, bool default_color = true);

        // Setting BSpline curve from cloud
        BSplineCurve3* GenerateRegressionCurve(KnotVector::Type type, GLuint k, GLuint n,
                                               const RowMatrix<GLdouble> &weight,
                                               GLdouble u_min = 0.0, GLdouble u_max = 1.0,
                                               GLuint div_point_count = 500,
                                               GLenum data_usage_flag = GL_STATIC_DRAW) const;

        void FindTheInterval(GLdouble &u_min, GLdouble &u_max);

    };

    inline std::ostream& operator << (std::ostream& lhs, const PointCloudAroundCurve3::SamplePoint& rhs)
    {
        lhs << rhs.parameter_value << " " << rhs.position;
        return lhs;
    }

    inline std::istream& operator >> (std::istream& lhs, PointCloudAroundCurve3::SamplePoint& rhs)
    {
        lhs >> rhs.parameter_value >> rhs.position;
        return lhs;
    }

    inline QTextStream& operator << (QTextStream& lhs, const PointCloudAroundCurve3::SamplePoint& rhs)
    {
        lhs << rhs.parameter_value << " " << rhs.position << "\n";
        return lhs;
    }

    inline QTextStream& operator >> (QTextStream& lhs, PointCloudAroundCurve3::SamplePoint& rhs)
    {
        lhs >> rhs.parameter_value >> rhs.position;
        return lhs;
    }

    inline std::ostream& operator << (std::ostream& lhs, const PointCloudAroundCurve3& rhs)
    {
        lhs << rhs._cloud;
        return lhs;
    }

    inline std::istream& operator >> (std::istream& lhs, PointCloudAroundCurve3& rhs)
    {
        lhs >> rhs._cloud;
        return lhs;
    }

    inline QTextStream& operator << (QTextStream& lhs, const PointCloudAroundCurve3& rhs)
    {
        lhs << rhs._cloud;
        return lhs;
    }

    inline QTextStream& operator >> (QTextStream& lhs, PointCloudAroundCurve3& rhs)
    {
        lhs >> rhs._cloud;
        return lhs;
    }
}
