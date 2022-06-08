#pragma once

#include "Core/DCoordinates3.h"

namespace cagd
{
    class Arcball
    {
    private:
        // Arcball variables for rotation
        double      _ballRadius;
        bool        _isRotating;

        int         _width;
        int         _height;

        DCoordinate3 _startRotationVector;
        DCoordinate3 _currentRotationVector;

        DCoordinate3 convertXY(int x, int y);
    public:
        Arcball();

        void setWidthHeight(int w, int h);
        void setRadius(float newRadius);

        void startRotation(int x, int y);
        void updateRotation(int x, int y);
        double stopRotation(DCoordinate3* rotationAxis);
    };
}
