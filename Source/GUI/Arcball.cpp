#include "Arcball.h"
#include <math.h>

#include "Core/Constants.h"

using namespace std;

namespace cagd
{
Arcball::Arcball()
{
    _ballRadius = 600;
    _isRotating = false;
    _width = 0;
    _height = 0;
}

void Arcball::setWidthHeight(int w, int h)
{
    _width = w;
    _height = h;
    _ballRadius = min((int)(w/2), (int)(h/2));
}

void Arcball::setRadius(float newRadius)
{
    _ballRadius = newRadius;
}

void Arcball::startRotation(int x, int y)
{
    int _x = x - _width / 2;
    int _y = _height / 2 - y;

    _startRotationVector = convertXY(_x,_y);
    _startRotationVector.normalize();

    _currentRotationVector =  _startRotationVector;
    _isRotating = true;
}


void Arcball::updateRotation(int x, int y)
{
    int _x = x - _width / 2;
    int _y = _height / 2 - y;

    _currentRotationVector = convertXY(_x,_y);
    _currentRotationVector.normalize();
}


double Arcball::stopRotation(DCoordinate3* rotationAxis)
{
    if (_isRotating)
    {
        DCoordinate3 temp = _currentRotationVector - _startRotationVector;
        if (temp.length() > EPS)
        {
            (*rotationAxis) = _currentRotationVector ^ _startRotationVector;
            rotationAxis->normalize();

            double val = _currentRotationVector * _startRotationVector;
            val = val > (1 - EPS) ? 1.0 : val;

            double rotationAngle = acos(val) * 180.0f/(float)PI;

            _isRotating = false;
            return rotationAngle;
        }
    }
    return 0.0;
}

DCoordinate3 Arcball::convertXY(int x, int y)
{

    int d = x*x+y*y;
    float radiusSquared = _ballRadius*_ballRadius;
    if (d > radiusSquared)
    {
        return DCoordinate3((float)x,(float)y, 0 );
    }
    else
    {
        return DCoordinate3((float)x,(float)y, sqrt(radiusSquared - d));
    }
}
}
