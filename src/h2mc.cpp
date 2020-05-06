#include "h2mc.h"

template <int dim>
void ComputeGaussian(const H2MCParam &param,
                     const Eigen::Matrix<Float, dim, 1> &grad,
                     const Eigen::Matrix<Float, dim, dim> &hess,
                     const Eigen::Matrix<Float, dim, 1> &invSigmaSq,
                     Gaussian &gaussian) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Float, dim, dim>> eigenSolver;
    eigenSolver.compute(hess);
    const auto &hEigenvector = eigenSolver.eigenvectors();
    const auto &hEigenvalues = eigenSolver.eigenvalues();
    Eigen::Matrix<Float, dim, 1> eigenBuff;
    Eigen::Matrix<Float, dim, 1> offsetBuff;
    Eigen::Matrix<Float, dim, 1> postInvCovEigenvalues;
    int dimension = dim == -1 ? grad.size() : dim;
    eigenBuff.resize(dimension);
    offsetBuff.resize(dimension);
    postInvCovEigenvalues.resize(dimension);

    for (int i = 0; i < dimension; i++) {
        if (fabs(hEigenvalues(i)) > Float(1e-10)) {
            eigenBuff(i) = 1.0 / fabs(hEigenvalues(i));
        } else {
            eigenBuff(i) = 0.0;
        }
    }

    offsetBuff.noalias() = eigenBuff.asDiagonal() * (hEigenvector.transpose() * grad);
    for (int i = 0; i < dimension; i++) {
        Float s2 = Float(1.0);
        Float o = Float(0.0);
        if (fabs(hEigenvalues(i)) > Float(1e-10)) {
            o = offsetBuff(i);
            if (hEigenvalues(i) > 0.0) {
                s2 = param.posScaleFactor;   // Float(0.5) * (exp(c_L) - exp(-c_L)) ^ 2;
                o *= param.posOffsetFactor;  //(Float(0.5) * (exp(c_L) + exp(-c_L)) - Float(1.0));
            } else {                         //<= 0.0
                s2 = param.negScaleFactor;   // sin(c_L) ^ 2;
                o *= param.negOffsetFactor;  //-(cos(c_L) - Float(1.0));
            }
        } else {
            s2 = param.L * param.L;
            o = Float(0.5) * offsetBuff(i) * param.L * param.L;
        }
        eigenBuff(i) *= (s2);
        if (eigenBuff(i) > Float(1e-10)) {
            eigenBuff(i) = Float(1.0) / eigenBuff(i);
        } else {
            eigenBuff(i) = Float(0.0);
        }
        offsetBuff(i) = o;
    }

    postInvCovEigenvalues = eigenBuff.array() + invSigmaSq.array();
    
    gaussian.invCov.noalias() =
        hEigenvector * postInvCovEigenvalues.asDiagonal() * hEigenvector.transpose();
    gaussian.mean.noalias() =
        hEigenvector * (eigenBuff.cwiseQuotient(postInvCovEigenvalues).asDiagonal() * offsetBuff);
    gaussian.covL.noalias() =
        hEigenvector * postInvCovEigenvalues.cwiseInverse().cwiseSqrt().asDiagonal();

    gaussian.logDet = Float(0.0);
    for (int i = 0; i < dim; i++) {
        gaussian.logDet += log(postInvCovEigenvalues[i]);
    }
}

void ComputeGaussian(const H2MCParam &param,
                     const Float sc,
                     const AlignedStdVector &vGrad,
                     const AlignedStdVector &vHess,
                     Gaussian &gaussian) {
    int dim = (int)vGrad.size();
    Eigen::Map<const Vector, Eigen::Aligned> grad(&vGrad[0], dim);
    Eigen::Map<const Matrix, Eigen::Aligned> hess(&vHess[0], dim, dim);

    gaussian.isDiagonal = false; 
    
    Vector sigma = Vector::Constant(dim, param.sigma);
    Float sigmaMax = sigma.maxCoeff();
    auto sigmaSq = sigma.cwiseProduct(sigma);
    Vector invSigmaSq = sigmaSq.cwiseInverse();
    if (sc <= Float(1e-15) || hess.norm() < Float(0.5) / (sigmaMax * sigmaMax)) {
        gaussian.mean = Vector::Zero(dim);
        gaussian.covL = sigma.asDiagonal();
        gaussian.invCov = invSigmaSq.asDiagonal();
        gaussian.logDet = Float(0.0);
        for (int i = 0; i < dim; i++) {
            gaussian.logDet += log(invSigmaSq[i]);
        }
    } else {
        switch (dim) {
            case 2: {
                ComputeGaussian<2>(param, grad, hess, invSigmaSq, gaussian);
                break;
            }
            case 3: {
                ComputeGaussian<3>(param, grad, hess, invSigmaSq, gaussian);
                break;
            }
            case 4: {
                ComputeGaussian<4>(param, grad, hess, invSigmaSq, gaussian);
                break;
            }
            case 5: {
                ComputeGaussian<5>(param, grad, hess, invSigmaSq, gaussian);
                break;
            }
            case 6: {
                ComputeGaussian<6>(param, grad, hess, invSigmaSq, gaussian);
                break;
            }
            case 7: {
                ComputeGaussian<7>(param, grad, hess, invSigmaSq, gaussian);
                break;
            }
            case 8: {
                ComputeGaussian<8>(param, grad, hess, invSigmaSq, gaussian);
                break;
            }
            case 9: {
                ComputeGaussian<9>(param, grad, hess, invSigmaSq, gaussian);
                break;
            }
            case 10: {
                ComputeGaussian<10>(param, grad, hess, invSigmaSq, gaussian);
                break;
            }
            case 11: {
                ComputeGaussian<11>(param, grad, hess, invSigmaSq, gaussian);
                break;
            }
            case 12: {
                ComputeGaussian<12>(param, grad, hess, invSigmaSq, gaussian);
                break;
            }
            default: { ComputeGaussian<-1>(param, grad, hess, invSigmaSq, gaussian); }
        }
   }
}
