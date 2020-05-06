#pragma once

#include "commondef.h"
#include "gaussian.h"
#include "alignedallocator.h"

#include <vector>

#define PCD_MIN           Float(0.01) 
#define PCD_MAX           Float(100) 
#define MTM_MIN           Float(-5.0)
#define MTM_MAX           Float(5.0)
#define LS_RATIO          Float(0.1)

void ComputeGaussian(const int dim,
					 const std::vector<Float> &v1, 
					 const std::vector<Float> &v2, 
					 const Float ss,
					 const Float shk,
					 const std::vector<Float> &M,
					 const int t, 
                     const Float sc,
                     Gaussian &gaussian);