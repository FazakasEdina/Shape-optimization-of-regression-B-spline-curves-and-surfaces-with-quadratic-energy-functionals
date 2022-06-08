#pragma once

#include <cmath>
#include <GL/glew.h>
#include <iostream>
#include <QTextStream>

namespace cagd
{
    //-------------------
    // class DCoordinate3
    //-------------------
    class DCoordinate3
    {
    private:
        double _data[3];

    public:

        // default constructor
        DCoordinate3();

        // special constructor
        DCoordinate3(double x, double y, double z = 0.0);

        // get components by value
        double operator [](unsigned int index) const;
        double x() const;
        double y() const;
        double z() const;


        // get components by reference
        double& operator [](unsigned int index);
        double& x();
        double& y();
        double& z();

        // change sign
        const DCoordinate3 operator +() const;
        const DCoordinate3 operator -() const;

        // add
        const DCoordinate3 operator +(const DCoordinate3& rhs) const;

        // add to *this
        DCoordinate3& operator +=(const DCoordinate3& rhs);

        // subtract
        const DCoordinate3 operator -(const DCoordinate3& rhs) const;

        // subtract from *this
        DCoordinate3& operator -=(const DCoordinate3& rhs);

        // cross product
        const DCoordinate3 operator ^(const DCoordinate3& rhs) const;

        // cross product, result is stored by *this
        DCoordinate3& operator ^=(const DCoordinate3& rhs);

        // dot product
        double operator *(const DCoordinate3& rhs) const;

        // scale
        const DCoordinate3 operator *(const double& rhs) const;
        const DCoordinate3 operator /(const double& rhs) const;

        // scale *this
        DCoordinate3& operator *=(const double& rhs);
        DCoordinate3& operator /=(const double& rhs);

        // length
        double length() const;

        // normalize
        DCoordinate3& normalize();

        // logical operators
        bool operator !=(const double& rhs) const;
    };

    //-------------------------------------
    // implementation of class DCoordinate3
    //-------------------------------------

    // default constructor
    inline DCoordinate3::DCoordinate3()
    {
        _data[0] = _data[1] = _data[2] = 0.0;
    }

    // special constructor
    inline DCoordinate3::DCoordinate3(double x, double y, double z)
    {
        _data[0] = x;
        _data[1] = y;
        _data[2] = z;
    }

    // get components by value
    inline double DCoordinate3::operator [](unsigned int index) const
    {
        return _data[index];
    }

    inline double DCoordinate3::x() const
    {
        return _data[0];
    }

    inline double DCoordinate3::y() const
    {
        return _data[1];
    }

    inline double DCoordinate3::z() const
    {
        return _data[2];
    }

    // get components by reference
    inline double& DCoordinate3::operator [](unsigned int index)
    {
        return _data[index];
    }

    inline double& DCoordinate3::x()
    {
        return _data[0];
    }

    inline double& DCoordinate3::y()
    {
        return _data[1];
    }

    inline double& DCoordinate3::z()
    {
        return _data[2];
    }

    // change sign
    inline const DCoordinate3 DCoordinate3::operator +() const
    {
        return DCoordinate3(_data[0], _data[1], _data[2]);
    }

    inline const DCoordinate3 DCoordinate3::operator -() const
    {
        return DCoordinate3(-_data[0], -_data[1], -_data[2]);
    }

    // add
    inline const DCoordinate3 DCoordinate3::operator +(const DCoordinate3& rhs) const
    {
        return DCoordinate3(_data[0] + rhs._data[0], _data[1] + rhs._data[1], _data[2] + rhs._data[2]);
    }

    // add to *this
    inline DCoordinate3& DCoordinate3::operator +=(const DCoordinate3& rhs)
    {
        _data[0] += rhs._data[0];
        _data[1] += rhs._data[1];
        _data[2] += rhs._data[2];
        return *this;
    }

    // subtract
    inline const DCoordinate3 DCoordinate3::operator -(const DCoordinate3& rhs) const
    {
        return DCoordinate3(_data[0] - rhs._data[0], _data[1] - rhs._data[1], _data[2] - rhs._data[2]);
    }

    // subtract from *this
    inline DCoordinate3& DCoordinate3::operator -=(const DCoordinate3& rhs)
    {
        _data[0] -= rhs._data[0];
        _data[1] -= rhs._data[1];
        _data[2] -= rhs._data[2];
        return *this;
    }

    // cross product
    inline const DCoordinate3 DCoordinate3::operator ^(const DCoordinate3& rhs) const
    {
        return DCoordinate3(
                _data[1] * rhs._data[2] - _data[2] * rhs._data[1],
                _data[2] * rhs._data[0] - _data[0] * rhs._data[2],
                _data[0] * rhs._data[1] - _data[1] * rhs._data[0]);
    }

    // cross product, result is stored by *this
    inline DCoordinate3& DCoordinate3::operator ^=(const DCoordinate3& rhs)
    {
        // use only 2 temp. variables
        double x = _data[0], y = _data[1];
        _data[0] = y * rhs._data[2] - _data[2] * rhs._data[1],
        _data[1] = _data[2] * rhs._data[0] - x * rhs._data[2],
        _data[2] = x * rhs._data[1] - y * rhs._data[0];
        return *this;
    }

    // dot product
    inline double DCoordinate3::operator *(const DCoordinate3& rhs) const
    {
        return _data[0] * rhs._data[0] + _data[1] * rhs._data[1] + _data[2] * rhs._data[2];
    }

    // scale
    inline const DCoordinate3 DCoordinate3::operator *(const double& rhs) const
    {
        return DCoordinate3(_data[0] * rhs, _data[1] * rhs, _data[2] * rhs);
    }

    inline const DCoordinate3 operator *(const double& lhs, const DCoordinate3& rhs)
    {
       return DCoordinate3(lhs * rhs[0], lhs * rhs[1], lhs * rhs[2]);
    }

    inline const DCoordinate3 DCoordinate3::operator /(const double& rhs) const
    {
        return DCoordinate3(_data[0] / rhs, _data[1] / rhs, _data[2] / rhs);
    }

    // scale *this
    inline DCoordinate3& DCoordinate3::operator *=(const double& rhs)
    {
        _data[0] *= rhs;
        _data[1] *= rhs;
        _data[2] *= rhs;

        return *this;
    }

    inline DCoordinate3& DCoordinate3::operator /=(const double& rhs)
    {
        _data[0] = _data[0] / rhs;
        _data[1] = _data[1] / rhs;
        _data[2] = _data[2] / rhs;

        return *this;
    }

    // length
    inline double DCoordinate3::length() const
    {
        return sqrt((*this) * (*this));
    }

    // normalize
    inline DCoordinate3& DCoordinate3::normalize()
    {
        double l = length();

        if (l && l != 1.0)
            *this /= l;

        return *this;
    }

    // logical operators
    inline bool DCoordinate3::operator !=(const double& rhs) const
    {
        return (_data[0] != rhs || _data[1] != rhs || _data[2] != rhs);
    }

    //----------------------------------------------------------------
    // definitions of overloaded input/output from/to stream operators
    //----------------------------------------------------------------

    // output to stream
    inline std::ostream& operator <<(std::ostream& lhs, const DCoordinate3& rhs)
    {
        return lhs << rhs[0] << " " << rhs[1] << " " << rhs[2];
    }

    // input from stream
    inline std::istream& operator >>(std::istream& lhs, DCoordinate3& rhs)
    {
        return lhs >> rhs[0] >> rhs[1] >> rhs[2];
    }

    inline QTextStream& operator <<(QTextStream& lhs, const DCoordinate3& rhs)
    {
        return lhs << rhs[0] << " " << rhs[1] << " " << rhs[2];
    }

    // input from stream
    inline QTextStream& operator >>(QTextStream& lhs, DCoordinate3& rhs)
    {
        return lhs >> rhs[0] >> rhs[1] >> rhs[2];
    }
}
