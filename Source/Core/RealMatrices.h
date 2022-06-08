#pragma once

#include <GL/glew.h>
#include "Matrices.h"
#include "Exceptions.h"
#include "DCoordinates3.h"

namespace cagd
{
class RealMatrix: public Matrix<GLdouble>
{
public:
    RealMatrix(GLuint row_size = 1, GLuint column_size = 1);

    RealMatrix(const RealMatrix& m);

    RealMatrix& operator =(const RealMatrix& rhs);

    const RealMatrix operator +(const RealMatrix& rhs) const;

    const RealMatrix operator -(const RealMatrix& rhs) const;

    const RealMatrix operator *(const RealMatrix& rhs) const;

    const RealMatrix operator *(const GLdouble value) const;

    template<class T>
    const ColumnMatrix<T> operator *(const ColumnMatrix<T>& rhs) const;

    template<class T>
    const Matrix<T> operator *(const Matrix<T>& rhs) const;

    const RealMatrix Transpose() const;
};

template<class T>
const ColumnMatrix<T> RealMatrix::operator *(const ColumnMatrix<T>& rhs) const
{
    if (this->GetColumnCount() != rhs.GetRowCount())
    {
        throw Exception("The size of the two matrices is incorrect.");
    }

    ColumnMatrix<T> C(this->GetRowCount());


    if (this->GetRowCount() <= this->GetColumnCount() && typeid(T) == typeid(DCoordinate3))
    {
        for (GLuint i = 0; i < this->GetRowCount(); i++)
        {
            if (typeid(T) == typeid(DCoordinate3))
            {
                GLdouble x = 0.0;
                GLdouble y = 0.0;
                GLdouble z = 0.0;
#pragma omp parallel for reduction(+:x, y, z)
                for (GLint j = 0; j < static_cast<GLint>(this->GetColumnCount()); j++)
                {
                    x += rhs[j].x() * (*this)(i, j);
                    y += rhs[j].y() * (*this)(i, j);
                    z += rhs[j].z() * (*this)(i, j);

                }
                C[i][0] = x;
                C[i][1] = y;
                C[i][2] = z;
            }
        }
    }
    else
    {
#pragma omp parallel for
        for (GLint i = 0; i < static_cast<GLint>(this->GetRowCount()); i++)
        {
            for (GLuint j = 0; j < this->GetColumnCount(); j++)
            {
                C[i] += rhs[j] * (*this)(i, j);
            }
        }

    }


    return C;
}



template<class T>
const Matrix<T> RealMatrix::operator *(const Matrix<T>& rhs) const
{
    if (this->GetColumnCount() != rhs.GetRowCount())
    {
        throw Exception("The size of the two matrices is incorrect.");
    }

    Matrix<T> C(this->GetRowCount(), rhs.GetColumnCount());


#pragma omp parallel for
    for(GLint i_j = 0; i_j < static_cast<GLint>(this->GetRowCount() * rhs.GetColumnCount()); i_j++)
    {
        int i = i_j / rhs.GetColumnCount();
        int j = i_j % rhs.GetColumnCount();

        // dot product
        for (GLuint k = 0; k < this->GetColumnCount(); k++)
        {
            C(i, j) += (*this)(i, k) * rhs(k, j);
        }
    }


    return C;
}


template<class T>
const Matrix<T> operator *(const Matrix<T>& lhs, const RealMatrix& rhs)
{
    if (lhs.GetColumnCount() != rhs.GetRowCount())
    {
        throw Exception("The size of the two matrices is incorrect.");
    }

    Matrix<T> C(lhs.GetRowCount(), rhs.GetColumnCount());


#pragma omp parallel for
    for(GLint i_j = 0; i_j < static_cast<GLint>(lhs.GetRowCount() * rhs.GetColumnCount()); i_j++)
    {
        int i = i_j / rhs.GetColumnCount();
        int j = i_j % rhs.GetColumnCount();

        // dot product
        for (GLuint k = 0; k < lhs.GetColumnCount(); k++)
        {
            C(i, j) += lhs(i, k) * rhs(k, j);
        }
    }

    return C;
}
}
