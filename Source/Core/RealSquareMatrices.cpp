#include "RealSquareMatrices.h"
using namespace std;

namespace cagd
{
RealSquareMatrix::RealSquareMatrix(GLuint size):
    RealMatrix(size, size),
    _lu_decomposition_is_done(false)
{}

RealSquareMatrix::RealSquareMatrix(const RealSquareMatrix& m):
    RealMatrix(m),
    _lu_decomposition_is_done(m._lu_decomposition_is_done),
    _row_permutation(m._row_permutation)
{
}

// specific contructor
RealSquareMatrix::RealSquareMatrix(const RealMatrix& m):
    RealMatrix(m),
    _lu_decomposition_is_done(false)
{

}


RealSquareMatrix& RealSquareMatrix::operator =(const RealSquareMatrix& rhs)
{
    if(this != &rhs)
    {
        RealMatrix::operator=(rhs);
        this->_lu_decomposition_is_done = rhs._lu_decomposition_is_done;
        this->_row_permutation = rhs._row_permutation;
    }
    return *this;
}

GLboolean RealSquareMatrix::ResizeRows(GLuint row_count)
{
    return RealMatrix::ResizeRows(row_count) && RealMatrix::ResizeColumns(row_count);
}

GLboolean RealSquareMatrix::ResizeColumns(GLuint column_count)
{
    return RealMatrix::ResizeRows(column_count) && RealMatrix::ResizeColumns(column_count);
}

GLboolean RealSquareMatrix::PerformLUDecomposition()
{
    if (_lu_decomposition_is_done)
        return GL_TRUE;

    if (_row_count <= 1)
        return GL_FALSE;

    const GLdouble tiny = numeric_limits<GLdouble>::min();

    GLuint size = static_cast<GLuint>(_data.size());
    vector<GLdouble> implicit_scaling_of_each_row(size);

    _row_permutation.resize(size);

    GLdouble row_interchanges = 1.0;

    //-------------------------------------------------------
    // loop over rows to get the implicit scaling information
    //-------------------------------------------------------
    vector<GLdouble>::iterator its = implicit_scaling_of_each_row.begin();
    for (vector<vector<GLdouble> >::const_iterator itr = _data.begin(); itr < _data.end(); ++itr)
    {
        GLdouble big = 0.0;
        for (vector<GLdouble>::const_iterator itc = itr->begin(); itc < itr->end(); ++itc)
        {
            GLdouble temp = abs(*itc);
            if (temp > big)
                big = temp;
        }

        if (big == 0.0)
        {
            // the matrix is singular
            return GL_FALSE;
        }
        *its = 1.0 / big;
        ++its;
    }

    //-----------------------------------
    // search for the largest pivot element
    //-----------------------------------
    for (GLuint k = 0; k < size; ++k)
    {
        GLuint imax = k;
        GLdouble big = 0.0;
        for (GLuint i = k; i < size; ++i)
        {
            GLdouble temp = implicit_scaling_of_each_row[i] * abs(_data[i][k]);
            if (temp > big)
            {
                big = temp;
                imax = i;
            }
        }

        // do we need to interchange rows?
        if (k != imax)
        {
            for (GLuint j = 0; j < size; ++j)
            {
                GLdouble temp = _data[imax][j];
                _data[imax][j] = _data[k][j];
                _data[k][j] = temp;
            }
            // change the parity of row_interchanges
            row_interchanges = -row_interchanges;
            // also interchange the scale factor
            implicit_scaling_of_each_row[imax] = implicit_scaling_of_each_row[k];
        }

        _row_permutation[k] = imax;
        if (_data[k][k] == 0.0)
            _data[k][k] = tiny;

        for (GLuint i = k + 1; i < size; ++i)
        {
            // divide by pivot element
            GLdouble temp = _data[i][k] /= _data[k][k];

            // reduce remaining submatrix
            for (GLuint j = k + 1; j < size; ++j)
                _data[i][j] -= temp * _data[k][j];
        }
    }

    _lu_decomposition_is_done = GL_TRUE;

    return GL_TRUE;
}

}
