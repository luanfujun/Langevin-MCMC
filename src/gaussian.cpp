#include "gaussian.h"
#include "fastmath.h"

void IsotropicGaussian(const int dim, const Float sigma, Gaussian &gaussian) {
    auto vsigma = Vector::Constant(dim, sigma);
    auto invSigmaSq = vsigma.cwiseProduct(vsigma).cwiseInverse();
    gaussian.isDiagonal = false; 
    if (gaussian.mean.size() != dim) {
        gaussian.mean = Vector::Zero(dim);
        gaussian.covL_d = Vector::Constant(dim, sigma);
        gaussian.invCov_d = Vector::Constant(dim, Float(1.0) / (sigma * sigma));
    } else {
        for (int i = 0; i < dim; i++) {
            gaussian.mean[i] = Float(0.0);
            gaussian.covL_d[i] = sigma;
            gaussian.invCov_d[i] = Float(1.0) / (sigma * sigma);
        }   
    } 
    gaussian.covL = vsigma.asDiagonal();
    gaussian.invCov = invSigmaSq.asDiagonal();
    gaussian.logDet = dim * fastlog(invSigmaSq[0]);
}

Float GaussianLogPdf(const Vector &offset, const Gaussian &gaussian) {
    assert(gaussian.mean.size() == offset.size());
    auto d = offset - gaussian.mean;
    Float logPdf = gaussian.mean.size() * (-Float(0.9189385332046727)); // = (-Float(0.5) * log(Float(2.0 * M_PI)));
    logPdf += Float(0.5) * gaussian.logDet;
    if (!gaussian.isDiagonal) {
        logPdf -= Float(0.5) * (d.transpose() * (gaussian.invCov * d))[0];
    } else {
        assert(gaussian.invCov_d.size() == offset.size());
        logPdf -= Float(0.5) * d.dot(gaussian.invCov_d.cwiseProduct(d));
    }
    return logPdf;
}

void GenerateSample(Gaussian &gaussian, Vector &x, RNG &rng) {
    if (gaussian.mean.size() != x.size()) {
        std::cerr << "gaussian.mean.size():" << gaussian.mean.size() << std::endl;
        std::cerr << "x.size():" << x.size() << std::endl;
    }
    assert(gaussian.mean.size() == x.size());
    std::normal_distribution<Float> normDist(Float(0.0), Float(1.0));
    for (int i = 0; i < x.size(); i++) {
        x[i] = normDist(rng);
    }

    if (!gaussian.isDiagonal) {
        x = gaussian.covL * x + gaussian.mean;
    } else {
        assert(gaussian.covL_d.size() == x.size());
        x = gaussian.covL_d.cwiseProduct(x) + gaussian.mean;
    }
}

