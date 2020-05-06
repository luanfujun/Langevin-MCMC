#pragma once

#include "commondef.h"
#include <algorithm>
#include <vector>

struct Chain; 

struct Gaussian {
    Matrix covL;
    Matrix invCov;
    Vector mean;
    Float logDet;

    // For MALA with diagonal preconditioning  
    bool isDiagonal = false; 
    Vector covL_d;
    Vector invCov_d;
};


inline int GetDimension(const Gaussian &gaussian) {
    return gaussian.mean.size();
}

void IsotropicGaussian(const int dim, const Float sigma, Gaussian &gaussian);
Float GaussianLogPdf(const Vector &offset, const Gaussian &gaussian);
void GenerateSample(Gaussian &gaussian, Vector &x, RNG &rng);