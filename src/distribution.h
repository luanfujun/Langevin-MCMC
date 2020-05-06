#pragma once

#include "commondef.h"
#include "utils.h"
#include <vector>

// From https://github.com/mmp/pbrt-v3/blob/master/src/core/sampling.h
struct PiecewiseConstant1D {
    PiecewiseConstant1D(const Float *f, int n) {
        count = n;
        func = new Float[n];
        memcpy(func, f, n * sizeof(Float));
        cdf = new Float[n + 1];
        cdf[0] = 0.;
        for (int i = 1; i < count + 1; ++i)
            cdf[i] = cdf[i - 1] + func[i - 1] / n;

        funcInt = cdf[count];
        if (funcInt == 0.f) {
            for (int i = 1; i < n + 1; ++i)
                cdf[i] = Float(i) / Float(n);
        } else {
            for (int i = 1; i < n + 1; ++i)
                cdf[i] /= funcInt;
        }
    }
    ~PiecewiseConstant1D() {
        delete[] func;
        delete[] cdf;
    }
    Float SampleContinuous(const Float u, Float *pdf, int *off = nullptr) const {
        Float *ptr = std::upper_bound(cdf, cdf + count + 1, u);
        int offset = Clamp(int(ptr - cdf - 1), 0, count - 1);
        if (off)
            *off = offset;

        Float du = (u - cdf[offset]) / (cdf[offset + 1] - cdf[offset]);

        if (pdf)
            *pdf = func[offset] / funcInt;

        return (offset + du) / count;
    }
    int SampleDiscrete(const Float u, Float *pdf) const {
        Float *ptr = std::upper_bound(cdf, cdf + count + 1, u);
        int offset = Clamp(int(ptr - cdf - 1), 0, count - 1);
        if (pdf != nullptr)
            *pdf = func[offset] / (funcInt * count);
        return offset;
    }

    Float Pmf(int offset) const {
        return func[offset] / (funcInt * count);
    }

    Float GetNormalization() const {
        return funcInt * count;
    }

    Float *func, *cdf;
    Float funcInt;
    int count;
};
