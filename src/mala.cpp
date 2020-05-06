#include "mala.h"
#include "fastmath.h"
#include "utils.h"
#include "gaussian.h"
#include "alignedallocator.h"

void ComputeGaussian(const int dim, 
                     const std::vector<Float> &v1, 
					 const std::vector<Float> &v2, 
					 const Float ss,
                     const Float shk,
					 const std::vector<Float> &M,
					 const int t, 
                     const Float sc,
                     Gaussian &gaussian)
{
	gaussian.isDiagonal = true;
	gaussian.logDet = Float(0.0);

    const Float shrk = inverse(shk * shk);
	if (sc <= Float(1e-10)) {
        const Float cov = Float(shk);
        if (gaussian.mean.size() != dim) {
            gaussian.mean = Vector::Zero(dim);
            gaussian.invCov_d = Vector::Constant(dim, shrk);
            gaussian.covL_d = Vector::Constant(dim, cov);
        } else {
            for (int i = 0; i < dim; i++) {
                gaussian.mean[i] = Float(0.0);
                gaussian.invCov_d[i] = shrk;
                gaussian.covL_d[i] = cov;
            }
        }
        gaussian.logDet = dim * fastlog(inverse(shk * shk));
    }
    else {
        if (gaussian.mean.size() != dim) {
            gaussian.mean = Vector::Zero(dim);
            gaussian.covL_d = Vector::Zero(dim);
            gaussian.invCov_d = Vector::Zero(dim);
        }
    	for (int i = 0; i < dim; i++) {
            Float cov_t = ss * ss * (M[i] + Float(1.0));
            Float invcov = inverse(cov_t) + shrk; 
            Float cov = inverse(invcov);
            gaussian.invCov_d[i] = invcov; 
            gaussian.covL_d[i] = sqrt(cov);
            gaussian.mean[i] = Clamp(v1[i], MTM_MIN, MTM_MAX) * cov / 2;
            gaussian.logDet += fastlog(invcov);
        }    
	}
}

