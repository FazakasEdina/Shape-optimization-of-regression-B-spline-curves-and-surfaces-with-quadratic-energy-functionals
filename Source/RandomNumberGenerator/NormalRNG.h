#pragma once

#include "RandomNumberGenerator.h"

namespace cagd
{
    class NormalRNG : RNG
    {
    private:
        double _mu;
        double _sigma;
    public:
        NormalRNG(double mu = 0.0, double sigma = 1.1);

        NormalRNG& operator =(const NormalRNG& rhs);

        RowMatrix<double> operator ()(const int n = 1) const;
    };
}
