#pragma once

#include <cmath>
#include <GL/glew.h>
#include <iostream>

namespace cagd
{
    //--------------------------------------
    // 3-dimensional homogeneous coordinates
    //--------------------------------------
    class HCoordinate3
    {
    protected:
        GLfloat _data[4]; // x, y, z, w;

    public:
        // default constructor
        HCoordinate3();

        // special constructor
        HCoordinate3(GLfloat x, GLfloat y, GLfloat z = 0.0, GLfloat w = 1.0);

        // get components by value
        GLfloat operator[](GLuint rhs) const;
        GLfloat x() const;
        GLfloat y() const;
        GLfloat z() const;
        GLfloat w() const;

        // get components by reference
        GLfloat& operator[](GLuint rhs);
        GLfloat& x();
        GLfloat& y();
        GLfloat& z();
        GLfloat& w();

        // add
        const HCoordinate3 operator +(const HCoordinate3& rhs) const;

        // add to this
        HCoordinate3& operator +=(const HCoordinate3& rhs);

        // subtract
        const HCoordinate3 operator -(const HCoordinate3& rhs) const;

        // subtract from this
        HCoordinate3& operator -=(const HCoordinate3& rhs);

        // dot product
        GLfloat operator *(const HCoordinate3& rhs) const;

        // cross product
        const HCoordinate3 operator ^(const HCoordinate3& rhs) const;

        // cross product with this
        HCoordinate3& operator ^=(const HCoordinate3& rhs);

        // multiplicate with scalar from right
        const HCoordinate3 operator *(GLfloat rhs) const;

        // multiplicate this with a scalar
        HCoordinate3& operator *=(GLfloat rhs);

        // divide with scalar
        const HCoordinate3 operator /(GLfloat rhs) const;

        // divide this with a scalar
        HCoordinate3& operator /=(GLfloat rhs);

        // length of vector represented by this homogeneous coordinate
        GLfloat length() const;

        // normalize
        HCoordinate3& normalize();
    };

    //--------------------------------------------------
    // implementation of HCoordinates3
    //--------------------------------------------------

    // default constructor
    inline HCoordinate3::HCoordinate3()
    {
        _data[0] = _data[1] = _data[2] = 0.0;
        _data[3] = 1.0;
    }

    // special constructor
    inline HCoordinate3::HCoordinate3(GLfloat x, GLfloat y, GLfloat z, GLfloat w)
    {
        _data[0] = x;
        _data[1] = y;
        _data[2] = z;
        _data[3] = w;
    }

    // data access by value
    inline GLfloat HCoordinate3::operator[](GLuint rhs) const
    {
        return _data[rhs];
    }

    inline GLfloat HCoordinate3::x() const
    {
        return _data[0];
    }

    inline GLfloat HCoordinate3::y() const
    {
        return _data[1];
    }

    inline GLfloat HCoordinate3::z() const
    {
        return _data[2];
    }

    inline GLfloat HCoordinate3::w() const
    {
        return _data[3];
    }

    // data access by reference
    inline GLfloat& HCoordinate3::operator[](GLuint rhs)
    {
        return _data[rhs];
    }

    inline GLfloat& HCoordinate3::x()
    {
        return _data[0];
    }

    inline GLfloat& HCoordinate3::y()
    {
        return _data[1];
    }

    inline GLfloat& HCoordinate3::z()
    {
        return _data[2];
    }

    inline GLfloat& HCoordinate3::w()
    {
        return _data[3];
    }

    // add
    inline const HCoordinate3 HCoordinate3::operator +(const HCoordinate3& rhs) const
    {
        return HCoordinate3(
                rhs.w() * x() + w() * rhs.x(),
                rhs.w() * y() + w() * rhs.y(),
                rhs.w() * z() + w() * rhs.z(),
                w() * rhs.w());
    }

     inline HCoordinate3& HCoordinate3::operator +=(const HCoordinate3& rhs)
     {
         _data[0] = rhs.w() * x() + w() * rhs.x();
         _data[1] = rhs.w() * y() + w() * rhs.y();
         _data[2] = rhs.w() * z() + w() * rhs.z();
         _data[3] = w() * rhs.w();

         return *this;
     }

     // substract
     inline const HCoordinate3 HCoordinate3::operator -(const HCoordinate3& rhs) const
     {
         return HCoordinate3(
                 rhs.w() * x() - w() * rhs.x(),
                 rhs.w() * y() - w() * rhs.y(),
                 rhs.w() * z() - w() * rhs.z(),
                 w() * rhs.w());
     }

     inline HCoordinate3& HCoordinate3::operator -=(const HCoordinate3& rhs)
     {
         _data[0] = rhs.w() * x() - w() * rhs.x();
         _data[1] = rhs.w() * y() - w() * rhs.y();
         _data[2] = rhs.w() * z() - w() * rhs.z();
         _data[3] = w() * rhs.w();

         return *this;
     }

     // dot product
     inline GLfloat HCoordinate3::operator *(const HCoordinate3& rhs) const
     {
         return (_data[0] * rhs._data[0] +
                _data[1] * rhs._data[1] +
                _data[2] * rhs._data[2]) / (_data[3] * rhs._data[3]);
     }

     // cross product
     inline const HCoordinate3 HCoordinate3::operator ^(const HCoordinate3& rhs) const
     {
         return  HCoordinate3(
                     _data[1] * rhs._data[2] - rhs._data[1] * _data[2],
                     rhs._data[0] * _data[2] - _data[0] * rhs._data[2],
                     _data[0] * rhs._data[1] - rhs._data[0] * _data[1],
                     _data[3] * rhs._data[3]
                     );
     }

     inline HCoordinate3& HCoordinate3::operator ^=(const HCoordinate3& rhs)
     {
         _data[0] = _data[1] * rhs._data[2] - rhs._data[1] * _data[2];
         _data[1] = rhs._data[0] * _data[2] - _data[0] * rhs._data[2];
         _data[2] = _data[0] * rhs._data[1] - rhs._data[0] * _data[1];
         _data[3] = _data[3] * rhs._data[3];

         return *this;
     }

     // multiplicate with scalar from right
     inline const HCoordinate3 HCoordinate3::operator *(GLfloat rhs) const
     {
         return HCoordinate3(_data[0] * rhs, _data[1] * rhs, _data[2] * rhs, _data[3]);
     }

     inline HCoordinate3& HCoordinate3::operator *=(GLfloat rhs)
     {
         _data[0] *= rhs;
         _data[1] *= rhs;
         _data[2] *= rhs;

         return *this;
     }

     // division
     inline const HCoordinate3 HCoordinate3::operator /(GLfloat rhs) const
     {
         return HCoordinate3(_data[0], _data[1], _data[2], _data[3] * rhs);
     }

     inline HCoordinate3& HCoordinate3::operator /=(GLfloat rhs)
     {
         _data[3] *= rhs;

         return *this;
     }

     inline GLfloat HCoordinate3::length() const
     {
         return (GLfloat) 1.0 / abs(_data[3]) * sqrt(_data[0] * _data[0] + _data[1] * _data[1] + _data[2] * _data[2]);
     }

     inline HCoordinate3& HCoordinate3::normalize()
     {
         double l = length();

         if (l && l != 1.0)
             *this /= (GLfloat) l;

         return *this;
     }

    // additional function implementations:

    // scale from left with a scalar
    inline const HCoordinate3 operator *(GLfloat lhs, const HCoordinate3& rhs)
    {
        return rhs * lhs;
    }

    // output to stream
    inline std::ostream& operator <<(std::ostream& lhs, const HCoordinate3& rhs)
    {
        lhs << rhs.x() << " " << rhs.y() << " " << rhs.z() << " " << rhs.w() << std::endl;
    }

    // input from stream
    inline std::istream& operator >>(std::istream& lhs, HCoordinate3& rhs)
    {
        lhs >> rhs[0] >> rhs[1] >> rhs[2] >> rhs[3];
    }
}
