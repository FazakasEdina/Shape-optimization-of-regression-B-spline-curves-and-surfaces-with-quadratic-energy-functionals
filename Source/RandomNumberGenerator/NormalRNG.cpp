#include "NormalRNG.h"
#include "Core/Constants.h"
#include <math.h>
#include <random>

using namespace std;

namespace cagd
{
    NormalRNG::NormalRNG(double mu, double sigma):
        RNG(),
        _mu(mu),
        _sigma(sigma)
    {

    }

    NormalRNG& NormalRNG::operator =(const NormalRNG& rhs)
    {
        if (this != &rhs)
        {
            _mu = rhs._mu;
            _sigma = rhs._sigma;
        }

        return *this;
    }

    RowMatrix<double> NormalRNG::operator ()(const int n) const
    {
        double alfa = 1 / (exp(0.5)); // \frac{1}{\sqrt{e}}
        double beta = 0.5;
        double lambda = sqrt(2);

        RowMatrix<double> result(n);
        bool L = false;

        uniform_real_distribution<double> dist(0.0, 1.0);
        //Mersenne Twister
        mt19937 rng;
        //Initialize with non-deterministic seeds
        rng.seed(random_device{}());

        for (int i = 0; i < n; i++)
        {
            L = false;
            double y;
            while (!L)
            {
               double u = dist(rng);
               double v = dist(rng);

               y = tan(PI * v);
               double s = beta * pow(y, 2);
               double w = alfa * u / (beta + s);

               L = abs(y) > lambda ? false : (w <= 1 - s);
               L = L == false ? (w <= exp(-s)) : L;

            }

            result[i] = _sigma * y + _mu;
        }
        return result;
    }
}
