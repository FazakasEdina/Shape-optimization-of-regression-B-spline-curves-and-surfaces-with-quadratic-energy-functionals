#include "PointCloudAroundSurface3.h"

#include "Core/RealSquareMatrices.h"
#include "Core/Materials.h"
#include "Core/Constants.h"

#include <limits>
#include <random>
#include <cmath>

using namespace std;

namespace cagd
{
PointCloudAroundSurface3::PointCloudAroundSurface3()
{
    _cloud.ResizeRows(1);
    _cloud.ResizeColumns(1);
}

PointCloudAroundSurface3::PointCloudAroundSurface3(const PointCloudAroundSurface3& rhs)
    :_cloud(rhs._cloud)
{

}

PointCloudAroundSurface3& PointCloudAroundSurface3::operator = (const PointCloudAroundSurface3& rhs)
{
    if (this != &rhs)
    {
        _cloud = rhs._cloud;
    }

    return *this;
}

bool PointCloudAroundSurface3::GeneratePointCloudAroundParametricSurface(
        const ParametricSurface3 &ps,
        RowMatrix<GLdouble> sigma,
        GLuint u_sample_size, GLuint v_sample_size)
{
    if (u_sample_size <= 0 || u_sample_size > INT_MAX || v_sample_size <= 0 || v_sample_size > INT_MAX)
    {
        throw Exception("The number of points must be greater than 0 and less than 2147483647!");
        return false;
    }
    if (sigma.GetColumnCount() != 3)
    {
        throw Exception("The sigma size must be 3!");
        return false;
    }
    _cloud.ResizeRows(u_sample_size);
    _cloud.ResizeColumns(v_sample_size);

    GLdouble u_min, u_max, v_min, v_max;
    ps.GetDefinitionDomain(u_min, u_max, v_min, v_max);

    // random number generator
    uniform_real_distribution<double> dist_u(u_min, u_max);
    uniform_real_distribution<double> dist_v(v_min, v_max);
    //Mersenne Twister
    mt19937 rng;
    //Initialize with non-deterministic seeds
    rng.seed(random_device{}());

    RowMatrix<NormalRNG*> normrnd(3);

#pragma omp parallel for
    for (GLint j = 0; j < static_cast<GLint> (sigma.GetColumnCount()); j++)
    {
        normrnd[j] = new NormalRNG(0, sigma[j]);
    }

    // generating random value in range of parametric surface
    RowMatrix<GLdouble> U(u_sample_size);
#pragma omp parallel for
    for (GLint j = 0; j < static_cast<GLint> (u_sample_size); j++)
    {
        U[j] = dist_u(rng);
    }

    RowMatrix<GLdouble> V(v_sample_size);
#pragma omp parallel for
    for (GLint j = 0; j < static_cast<GLint> (v_sample_size); j++)
    {
        V[j] = dist_v(rng);
    }

    ///  X = c(U, V) + epsilon
#pragma omp parallel for
    for (GLint i_j = 0; i_j < static_cast<GLint> (u_sample_size * v_sample_size); i_j++)
    {
        GLint i = i_j / v_sample_size;
        GLint j = i_j % v_sample_size;

        _cloud(i, j).parameter_value_u = U[i];
        _cloud(i, j).parameter_value_v = V[j];

        // calculating epsilon value
        // noise for each coordinate axis
        RowMatrix<GLdouble> epsilon(3);
        for (GLuint k = 0; k < 3; k++)
        {
            // the k -th normal rng give 1 value in a row matrix
            epsilon[k] = (*normrnd[k])(1)[0];
        }

        // the value of function c(u, v)
        DCoordinate3 c_u_v = ps(U[i], V[j]);
        _cloud(i, j).position = DCoordinate3(c_u_v.x() + epsilon[0], c_u_v.y() + epsilon[1], c_u_v.z() + epsilon[2]);

    }
    return true;
}

// Render the points of cloud
bool PointCloudAroundSurface3::RenderPointCloud(TriangulatedMesh3 *sphere, double point_size, bool dark_mode, bool default_color)
{
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);

    for (GLuint i = 0; i < _cloud.GetRowCount(); i++)
    {
        for (GLuint j = 0; j < _cloud.GetColumnCount(); j++)
        {
            glPushMatrix();
            glTranslated(_cloud(i, j).position[0], _cloud(i, j).position[1], _cloud(i, j).position[2]);
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

    }
    glDisable(GL_LIGHTING);
    return true;
}

BSplinePatch3* PointCloudAroundSurface3::GenerateRegressionSurface(const RowMatrix<GLdouble> &weight, KnotVector::Type u_type, KnotVector::Type v_type,
                                                                   GLuint u_k, GLuint v_k,
                                                                   GLuint u_n, GLuint v_n,
                                                                   GLdouble u_min, GLdouble u_max,
                                                                   GLdouble v_min, GLdouble v_max,
                                                                   GLuint div_point_count) const
{
    BSplinePatch3* result = new (nothrow) BSplinePatch3(u_type, v_type, u_k, v_k, u_n, v_n, u_min, u_max, v_min, v_max);

    if (!result)
    {
        return nullptr;
    }

    GLuint u_cloud_size = _cloud.GetRowCount();
    GLuint v_cloud_size = _cloud.GetColumnCount();
    RealMatrix F(u_cloud_size, u_n + 1);
    RealMatrix G(v_cloud_size, v_n + 1);
    GLuint rho = weight.GetColumnCount();

#pragma omp parallel for
    for (GLint i = 0; i < static_cast<GLint> (u_cloud_size); i++)    // setting F matrix
    {
        RowMatrix<GLdouble> u_blending_values(u_n + 1);
        switch(u_type)
        {
        case KnotVector::CLAMPED:
            result->UBlendingFunctionValues(_cloud(i, 0).parameter_value_u, u_blending_values);
            break;
        case KnotVector::UNCLAMPED:
            result->UBlendingFunctionValues(_cloud(i, 0).parameter_value_u, u_blending_values);
            break;
        case KnotVector::PERIODIC:
            result->UBlendingFunctionValuesForPeriodicRegressionSurface(_cloud(i, 0).parameter_value_u, u_blending_values);
            break;
        }

        // set row of F
        F.SetRow(i, u_blending_values);
    }

#pragma omp parallel for
    for (GLint i = 0; i < static_cast<GLint> (v_cloud_size); i++)    // setting G matrix
    {
        RowMatrix<GLdouble> v_blending_values(1);
        GLdouble v = _cloud(0, i).parameter_value_v;
        switch(v_type)
        {
        case KnotVector::CLAMPED:
            result->VBlendingFunctionValues(v, v_blending_values);
            break;
        case KnotVector::UNCLAMPED:
            result->VBlendingFunctionValues(v, v_blending_values);
            break;
        case KnotVector::PERIODIC:
            result->VBlendingFunctionValuesForPeriodicRegressionSurface(v, v_blending_values);
            break;
        }

        // set row of G
        G.SetRow(i, v_blending_values);
    }

    Matrix<DCoordinate3> D(u_cloud_size, v_cloud_size);
#pragma omp parallel for
    for (GLint i_j = 0; i_j < static_cast<GLint> (u_cloud_size * v_cloud_size); i_j++)      // setting D matrix
    {
        GLint i = i_j / v_cloud_size;
        GLint j = i_j % v_cloud_size;
        D(i, j) = _cloud(i, j).position;
    }

    // Y = F' * D * G
    Matrix<DCoordinate3> Y = F.Transpose() * D * G;

    GLuint size = (u_n + 1) * (v_n + 1);
    ColumnMatrix<DCoordinate3> b(size);
    for (GLuint i = 0; i < u_n + 1; i++)
    {
        for (GLuint j = 0;  j < v_n + 1; j++)
        {
            b[i * (v_n + 1) + j] = Y(i, j);
        }
    }

    TriangularMatrix<RealMatrix> fi(rho + 1);
    TriangularMatrix<RealMatrix> gamma(rho + 1);

    for (GLuint r = 1; r <= rho; r++)
    {
        if (weight[r - 1] != 0.0)
        {
            RowMatrix<RealMatrix*> derivatives_fi = result->GetKnotVectorU()->LookUpTablesForSurfaceOptimizatioin(weight[r - 1], r, div_point_count);
            RowMatrix<RealMatrix*> derivatives_gamma = result->GetKnotVectorV()->LookUpTablesForSurfaceOptimizatioin(weight[r - 1], r, div_point_count);
            for (GLuint zeta = 0; zeta <= r; zeta ++)
            {
                fi(r, zeta) = (*derivatives_fi(zeta));
                gamma(r, zeta) = (*derivatives_gamma(zeta));

                delete derivatives_fi[zeta];
                derivatives_fi[zeta] = nullptr;
                delete derivatives_gamma[zeta];
                derivatives_gamma[zeta] = nullptr;
            }
        }
    }

    RealSquareMatrix A(size);
    for (GLuint s = 0; s <= u_n; s++)
    {
        for (GLuint t = 0; t <= v_n; t++)
        {
            for (GLuint k = 0; k <= u_n; k++)
            {
                for (GLuint l = 0; l <= v_n; l++)
                {
                    GLdouble aux = 0.0;
                    for (GLuint i = 0; i < u_cloud_size; i++)
                    {
                        for (GLuint j = 0; j < v_cloud_size; j++)
                        {
                            aux += F(i, s) * F(i, k) * G(j, t) * G(j, l);
                        }
                    }

                    GLdouble aux2 = 0.0;
                    for (GLuint r = 1; r <= rho; r++)
                    {
                        if (weight[r - 1] != 0.0)
                        {
                            for (GLuint zeta = 0; zeta <= r; zeta++)
                            {
                                aux2 += fi(r, r - zeta)(s, k) * gamma(r, zeta) (t, l);
                            }
                        }
                    }

                    A(s * (v_n + 1) + t, k * (v_n + 1) + l) = aux + aux2;
                }
            }
        }
    }

    ColumnMatrix<DCoordinate3> P(size);

    A.SolveLinearSystem(b, P);

    // Set polygon
#pragma omp parallel for
    for (GLint i_j = 0; i_j < static_cast<GLint>(size); i_j++)
    {
        GLint i = i_j / (v_n + 1);
        GLint j = i_j % (v_n + 1);

        (*result)(i, j) = P[i_j];
    }

    return result;
}

void PointCloudAroundSurface3::FindTheInterval(GLdouble &u_min, GLdouble &u_max, GLdouble &v_min, GLdouble &v_max)
{
    u_min = _cloud(0, 0).parameter_value_u;
    u_max = _cloud(0, 0).parameter_value_u;
    v_min = _cloud(0, 0).parameter_value_v;
    v_max = _cloud(0, 0).parameter_value_v;


    for (GLuint i = 0; i < _cloud.GetRowCount(); i++)
    {
        for (GLuint j = 0; j < _cloud.GetColumnCount(); j++)
        {
            if (_cloud(i, j).parameter_value_u > u_max)
            {
                u_max = _cloud(i, j).parameter_value_u;
            }

            if (_cloud(i, j).parameter_value_u < u_min)
            {
                u_min = _cloud(i, j).parameter_value_u;
            }

            if (_cloud(i, j).parameter_value_v > v_max)
            {
                v_max = _cloud(i, j).parameter_value_v;
            }

            if (_cloud(i, j).parameter_value_v < v_min)
            {
                v_min = _cloud(i, j).parameter_value_v;
            }
        }
    }

//    u_min = floor(u_min);
//    u_max = ceil(u_max);
//    v_min = floor(v_min);
//    v_max = ceil(v_max);
}
}
