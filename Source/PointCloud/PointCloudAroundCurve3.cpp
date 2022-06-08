#include "PointCloud/PointCloudAroundCurve3.h"
#include "Core/Exceptions.h"
#include "Core/RealMatrices.h"
#include "Core/RealSquareMatrices.h"
#include "Core/Materials.h"
#include "Core/Constants.h"

#include <limits>
#include <random>
#include <math.h>

namespace cagd
{
PointCloudAroundCurve3::PointCloudAroundCurve3()
{
    _cloud.ResizeColumns(0);
}

PointCloudAroundCurve3::PointCloudAroundCurve3(const PointCloudAroundCurve3& rhs)
    :_cloud(rhs._cloud)
{

}

PointCloudAroundCurve3& PointCloudAroundCurve3::operator =(const PointCloudAroundCurve3& rhs)
{
    if (this != &rhs)
    {
        _cloud = rhs._cloud;
    }

    return *this;
}

bool PointCloudAroundCurve3::GeneratePointCloudAroundParametricCurve(
        const ParametricCurve3 &pc,
        RowMatrix<GLdouble> sigma,
        GLuint sample_size)
{
    if (sample_size <= 0 || sample_size > INT_MAX)
    {
        throw Exception("The number of points must be greater than 0 and less than 2147483647!");
        return false;
    }
    if (sigma.GetColumnCount() != 3)
    {
        throw Exception("The sigma size must be 3!");
        return false;
    }
    _cloud.ResizeColumns(sample_size);

    GLdouble u_min, u_max;
    pc.GetDefinitionDomain(u_min, u_max);

    // random number generator
    uniform_real_distribution<double> dist(u_min, u_max);
    //Mersenne Twister
    mt19937 rng;
    rng.seed(random_device{}());

    RowMatrix<NormalRNG*> normrnd(sigma.GetColumnCount());

    for (int j = 0; j < (int)sigma.GetColumnCount(); j++)
    {
        normrnd[j] = new NormalRNG(0, sigma[j]);
    }

    ///  X = c(U) + epsilon
#pragma omp parallel for
    for (GLint i = 0; i < static_cast<GLint> (sample_size); i++)
    {
        // generating random value in range of parametric curve
        GLdouble u = dist(rng);

        _cloud[i].parameter_value = u;

        // calculating epsilon value
        // noise for each coordinate axis
        RowMatrix<GLdouble> epsilon(sigma.GetColumnCount());
        for (GLuint j = 0; j < sigma.GetColumnCount(); j++)
        {
            // the j -th normal rng give 1 value in a row matrix
            epsilon[j] = (*normrnd[j])(1)[0];
        }

        // the value of function c(u)
        DCoordinate3 c_u = pc(0, _cloud[i].parameter_value);
        _cloud[i].position = DCoordinate3(c_u.x() + epsilon[0], c_u.y() + epsilon[1], c_u.z() + epsilon[2]);
    }

    return true;
}

bool PointCloudAroundCurve3::RenderPointCloud(TriangulatedMesh3 *sphere, double point_size, bool dark_mode, bool default_color)
{
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);

    for (GLuint i = 0; i < _cloud.GetColumnCount(); i++)
    {
        glPushMatrix();
        glTranslated(_cloud[i].position[0], _cloud[i].position[1], _cloud[i].position[2]);
        glScalef(point_size, point_size, point_size);
        if (!default_color)
        {
            MatFBBrass.Apply();
        }
        else
        {
            if (dark_mode)
            {
                MatFBPearl.Apply();
            }
            else
            {
                MatFBSilver.Apply();
            }
        }
        sphere->Render();
        glPopMatrix();
    }
    glDisable(GL_LIGHTING);
    return true;
}

BSplineCurve3* PointCloudAroundCurve3::GenerateRegressionCurve(KnotVector::Type type, GLuint k, GLuint n,
                                                               const RowMatrix<GLdouble> &weight,
                                                               GLdouble u_min, GLdouble u_max,
                                                               GLuint div_point_count,
                                                               GLenum data_usage_flag) const
{
    BSplineCurve3* result = new BSplineCurve3(type, k, n, u_min, u_max, data_usage_flag);

    if (!result)
    {
        return nullptr;
    }

    // non-square collocation matrix:
    RealMatrix F(_cloud.GetColumnCount(), n + 1);
    ColumnMatrix<DCoordinate3> X(_cloud.GetColumnCount());


#pragma omp parallel for
    for (GLint i = 0; i < static_cast<GLint> (_cloud.GetColumnCount()); i++)
    {
        RowMatrix<GLdouble> values(1);
        switch(type)
        {
        case KnotVector::CLAMPED:
            result->BlendingFunctionValues(_cloud[i].parameter_value, values);
            break;
        case KnotVector::UNCLAMPED:
            result->BlendingFunctionValues(_cloud[i].parameter_value, values);
            break;
        case KnotVector::PERIODIC:
            result->BlendingFunctionValuesForPeriodicRegressionCurve(_cloud[i].parameter_value, values);
            break;
        }

        // set row of F
        F.SetRow(i, values);
        X[i] = _cloud[i].position;
    }

    // FT = F'; FT_F = FT * F;
    RealMatrix FT_F = F.Transpose() * F;

    RealMatrix A = result->GetKnotVector()->LookUpTableForCurveOptimizatioin(weight, div_point_count);

    FT_F = FT_F + A;

    RealSquareMatrix FT_F2(FT_F);
    ColumnMatrix<DCoordinate3> FT_X;
    FT_X = F.Transpose() * X;
    // P = inv(FT_F) * FT_X;
    ColumnMatrix<DCoordinate3> P;
    FT_F2.SolveLinearSystem(FT_X, P);

#pragma omp parallel for
    for (GLint i = 0; i < static_cast<GLint> (P.GetRowCount()); i++)
    {
        (*result)[i] = P[i];
    }

    return result;
}


void PointCloudAroundCurve3::FindTheInterval(GLdouble &u_min, GLdouble &u_max)
{
    u_min = _cloud[0].parameter_value;
    u_max = _cloud[0].parameter_value;

    for (GLuint i = 1; i < _cloud.GetColumnCount(); i++)
    {
        if (_cloud[i].parameter_value > u_max)
        {
            u_max = _cloud[i].parameter_value;
        }

        if (_cloud[i].parameter_value < u_min)
        {
            u_min = _cloud[i].parameter_value;
        }
    }

//    u_min = floor(u_min);
//    u_max = ceil(u_max);
}
}
