#pragma once

#include "commondef.h"

template <typename FloatType>
struct TRay {
    TVector3<FloatType> org;
    TVector3<FloatType> dir;
};

using Ray = TRay<Float>;
using ADRay = TRay<ADFloat>;

struct RaySegment {
    Ray ray;
    Float minT;
    Float maxT;
};
