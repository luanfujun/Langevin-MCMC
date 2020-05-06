#pragma once

#include "chad.h"
#include "flexception.h"
#include "pcg_random.hpp"
#include "alignedallocator.h"

#include <Eigen/Dense>
#include <random>

using namespace chad;

template <typename T, int N, int _Options = Eigen::AutoAlign>
using TVector = Eigen::Matrix<T, N, 1, _Options>;
template <typename T, int _Options = Eigen::AutoAlign>
using TVector1 = TVector<T, 1, _Options>;
template <typename T, int _Options = Eigen::AutoAlign>
using TVector2 = TVector<T, 2, _Options>;
template <typename T, int _Options = Eigen::AutoAlign>
using TVector3 = TVector<T, 3, _Options>;
template <typename T, int _Options = Eigen::AutoAlign>
using TVector4 = TVector<T, 4, _Options>;
template <typename T, int _Options = Eigen::AutoAlign>
using TMatrix3x3 = Eigen::Matrix<T, 3, 3, _Options>;
template <typename T, int _Options = Eigen::AutoAlign>
using TMatrix4x4 = Eigen::Matrix<T, 4, 4, _Options>;
#if defined(SINGLE_PRECISION)
using Float = float;
#elif defined(DOUBLE_PRECISION)
using Float = double;
#endif
using Vector1 = TVector1<Float>;
using Vector2 = TVector2<Float>;
using Vector3 = TVector3<Float>;
using Vector4 = TVector4<Float>;
using Matrix3x3 = TMatrix3x3<Float>;
using Matrix4x4 = TMatrix4x4<Float>;
using ADFloat = ExpressionCPtr;
using ADBool = BooleanCPtr;
using ADVector2 = TVector2<ADFloat>;
using ADVector3 = TVector3<ADFloat>;
using ADVector4 = TVector4<ADFloat>;
using ADMatrix3x3 = TMatrix3x3<ADFloat>;
using ADMatrix4x4 = TMatrix4x4<ADFloat>;
using Vector2i = TVector2<int>;

using Vector = Eigen::Matrix<Float, Eigen::Dynamic, 1>;
using Matrix = Eigen::Matrix<Float, Eigen::Dynamic, Eigen::Dynamic>;

using AlignedStdVector = std::vector<Float, aligned_allocator<Float, 64>>;

#if defined(SINGLE_PRECISION)
const Float c_IsectEpsilon = Float(5e-4); // Float(1e-3);
const Float c_ShadowEpsilon = Float(5e-4); // Float(1e-3);
#elif defined(DOUBLE_PRECISION)
const Float c_IsectEpsilon = Float(1e-7);
const Float c_ShadowEpsilon = Float(1e-5);
#endif
// Avoid grazing angle artifacts
const Float c_CosEpsilon = Float(1e-4);

// using RNG = std::mt19937;
using RNG = pcg32_k64_fast;

const Float FTRUE = Float(1.0);
const Float FFALSE = Float(0.0);

inline Float BoolToFloat(const bool c) {
    return c ? FTRUE : FFALSE;
}

using PrimID = int;
using ShapeID = int;
using TriIndexID = uint32_t;

const Float c_PI = Float(3.14159265358979323846);
const Float c_INVPI = Float(1.0) / c_PI;
const Float c_TWOPI = Float(2.0) * c_PI;
const Float c_INVTWOPI = Float(1.0) / c_TWOPI;
const Float c_FOURPI = Float(4.0) * c_PI;
const Float c_INVFOURPI = Float(1.0) / c_FOURPI;
const Float c_PIOVERTWO = Float(0.5) * c_PI;
const Float c_PIOVERFOUR = Float(0.25) * c_PI;

#if defined(__APPLE__) && defined(__MACH__)
// clang on MacOS does not support thread_local keyword...
#define thread_local __thread
#endif
