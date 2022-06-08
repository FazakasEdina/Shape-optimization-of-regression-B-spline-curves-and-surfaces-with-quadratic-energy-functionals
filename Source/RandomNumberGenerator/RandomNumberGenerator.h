#pragma once

#include <GL/glew.h>

#include "Core/Matrices.h"

namespace cagd
{
    class RNG
    {
    private:
    public:
        RNG()
        {
        }

        virtual RowMatrix<double> operator ()(const int n = 1) const = 0;

    };
}
