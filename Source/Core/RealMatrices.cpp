#include "RealMatrices.h"
#include "Exceptions.h"

namespace cagd
{
RealMatrix::RealMatrix(GLuint row_size, GLuint column_size):
    Matrix<GLdouble>(row_size, column_size)
{
}

RealMatrix::RealMatrix(const RealMatrix& m):
    Matrix<GLdouble>(m)
{
}

RealMatrix& RealMatrix::operator =(const RealMatrix& rhs)
{
    if (this != &rhs)
    {
        Matrix<GLdouble>::operator=(rhs);
    }
    return *this;
}

const RealMatrix RealMatrix::operator +(const RealMatrix& rhs) const
{
    if (this->GetColumnCount() != rhs.GetColumnCount() ||
            this->GetRowCount() != rhs.GetRowCount())
    {
        throw Exception("The size of the two matrices does not match.");
    }

    RealMatrix C(this->GetRowCount(), this->GetColumnCount());

#pragma omp parallel for
    for(GLint i_j = 0; i_j < static_cast<GLint> (this->GetRowCount() * rhs.GetColumnCount()); i_j++)
    {
        GLint i = i_j / rhs.GetColumnCount();
        GLint j = i_j % rhs.GetColumnCount();

        C(i, j) = (*this)(i, j) + rhs(i, j);
    }
    return C;
}

const RealMatrix RealMatrix::operator -(const RealMatrix& rhs) const
{
    if (this->GetColumnCount() != rhs.GetColumnCount() ||
            this->GetRowCount() != rhs.GetRowCount())
    {
        throw Exception("The size of the two matrices does not match.");
    }

    RealMatrix C(this->GetRowCount(), this->GetColumnCount());
#pragma omp parallel for
    for(GLint i_j = 0; i_j < static_cast<GLint> (this->GetRowCount() * rhs.GetColumnCount()); i_j++)
    {
        GLint i = i_j / rhs.GetColumnCount();
        GLint j = i_j % rhs.GetColumnCount();

        C(i, j) = (*this)(i, j) - rhs(i, j);
    }
    return C;
}

const RealMatrix RealMatrix::operator *(const RealMatrix& rhs) const
{
    if (this->GetColumnCount() != rhs.GetRowCount())
    {
        throw Exception("The size of the two matrices is incorrect.");
    }

    RealMatrix C(this->GetRowCount(), rhs.GetColumnCount());

#pragma omp parallel for
    for(GLint i_j = 0; i_j < static_cast<GLint> (this->GetRowCount() * rhs.GetColumnCount()); i_j++)
    {
        GLint i = i_j / rhs.GetColumnCount();
        GLint j = i_j % rhs.GetColumnCount();

        // dot product
        for (GLuint k = 0; k < this->GetColumnCount(); k++)
        {
            C(i, j) += (*this)(i, k) * rhs(k, j);
        }
    }
    return C;
}


const RealMatrix RealMatrix::operator *(const GLdouble value) const
{
    RealMatrix C(this->GetRowCount(), this->GetColumnCount());

#pragma omp parallel for
    for(GLint i_j = 0; i_j < static_cast<GLint> (this->GetRowCount() * this->GetColumnCount()); i_j++)
    {
        GLint i = i_j / this->GetColumnCount();
        GLint j = i_j % this->GetColumnCount();

        C(i, j) = (*this)(i, j) * value;
    }
    return C;
}


const RealMatrix RealMatrix::Transpose() const
{
    RealMatrix C(this->GetColumnCount(), this->GetRowCount());

#pragma omp parallel for
    for(GLint i_j = 0; i_j < static_cast<GLint> (this->GetRowCount() * this->GetColumnCount()); i_j++)
    {
        GLint i = i_j / this->GetColumnCount();
        GLint j = i_j % this->GetColumnCount();

        C(j, i) = (*this)(i, j);
    }
    return C;
}
}
