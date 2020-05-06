#pragma once

#include "commondef.h"

#include <algorithm>
#include <vector>
#include <cmath>

inline ExpressionCPtrVec ToADFloatVec(const ADVector4 &v) {
    ExpressionCPtrVec ret(4);
    for (int i = 0; i < 4; i++) {
        ret[i] = v[i];
    }
    return ret;
}

inline ExpressionCPtrVec ToADFloatVec(const ADMatrix4x4 &m) {
    ExpressionCPtrVec ret(16);
    int k = 0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ret[k++] = m(j, i);
        }
    }
    return ret;
}

inline ADVector4 ToADVector4(const ExpressionCPtrVec &v) {
    ADVector4 ret;
    for (int i = 0; i < 4; i++) {
        ret[i] = v[i];
    }
    return ret;
}

inline ADVector4 ToADVector4(const std::vector<CondExprCPtr> &v) {
    ADVector4 ret;
    for (int i = 0; i < 4; i++) {
        ret[i] = v[i];
    }
    return ret;
}

inline ADMatrix4x4 ToADMatrix4x4(const ExpressionCPtrVec &m) {
    ADMatrix4x4 ret;
    int k = 0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ret(j, i) = m[k++];
        }
    }
    return ret;
}

inline ADMatrix4x4 ToADMatrix4x4(const std::vector<CondExprCPtr> &m) {
    ADMatrix4x4 ret;
    int k = 0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ret(j, i) = m[k++];
        }
    }
    return ret;
}

inline std::vector<CondExprCPtr> CreateCondExprVec(const int len) {
    std::vector<CondExprCPtr> ret(len);
    for (int i = 0; i < len; i++) {
        ret[i] = CondExpr::Create();
    }
    return ret;
}

template <typename FloatType>
FloatType Const(const Float c) {
    return c;
}

template <>
inline ADFloat Const(const Float c) {
    return Constant::Create(c);
}

inline Float inverse(const Float x) {
    return Float(1.0) / x;
}

inline Float square(const Float x) {
    return x * x;
}

inline ExpressionCPtr if_else(const BooleanCPtr &c,
                              const ExpressionCPtr &x,
                              const ExpressionCPtr &y) {
    CondExprCPtr ret = CondExpr::Create();
    BeginIf(c, {ret});
    { SetCondOutput({x}); }
    BeginElse();
    { SetCondOutput({y}); }
    EndIf();
    return ret;
}

template <typename T>
inline T if_else(const bool &c, const T &x, const T &y) {
    return c ? x : y;
}

inline bool Gt(const Float x, const Float y) {
    return x > y;
}
inline bool Gte(const Float x, const Float y) {
    return x >= y;
}
inline bool Eq(const Float x, const Float y) {
    return x == y;
}
inline bool Lte(const Float x, const Float y) {
    return x <= y;
}
inline bool Lt(const Float x, const Float y) {
    return x < y;
}

template <typename FloatType>
inline FloatType LengthSquared(const TVector3<FloatType> &v) {
    return square(v[0]) + square(v[1]) + square(v[2]);
}

template <typename FloatType>
inline FloatType LengthSquared(const TVector4<FloatType> &v) {
    return square(v[0]) + square(v[1]) + square(v[2]) + square(v[3]);
}

template <typename FloatType>
inline FloatType DistanceSquared(const TVector3<FloatType> &v0, const TVector3<FloatType> &v1) {
    return square(v0[0] - v1[0]) + square(v0[1] - v1[1]) + square(v0[2] - v1[2]);
}

inline Float length3d(const Float x, const Float y, const Float z) {
    return std::sqrt(x * x + y * y + z * z);
}

inline Float length4d(const Float x, const Float y, const Float z, const Float w) {
    return std::sqrt(x * x + y * y + z * z + w * w);
}

template <typename FloatType>
inline FloatType Distance(const TVector3<FloatType> &v0, const TVector3<FloatType> &v1) {
    return length3d(v0[0] - v1[0], v0[1] - v1[1], v0[2] - v1[2]);
}

template <typename FloatType>
inline FloatType Dot(const TVector3<FloatType> &v1, const TVector3<FloatType> &v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

template <>
inline ADFloat Dot<ADFloat>(const TVector3<ADFloat> &v1, const TVector3<ADFloat> &v2) {
    return dot3d({{v1[0], v1[1], v1[2]}}, {{v2[0], v2[1], v2[2]}});
}

template <typename FloatType>
inline FloatType Dot(const TVector4<FloatType> &v1, const TVector4<FloatType> &v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2] + v1[3] * v2[3];
}

template <typename FloatType>
inline FloatType Length(const TVector3<FloatType> &v) {
    return length3d(v[0], v[1], v[2]);
}

template <typename FloatType>
inline FloatType Length(const TVector4<FloatType> &v) {
    return length4d(v[0], v[1], v[2], v[3]);
}

template <typename FloatType>
inline TVector3<FloatType> Normalize(const TVector3<FloatType> &v) {
    FloatType invLen = inverse(Length(v));
    return v * invLen;
}

template <typename FloatType>
inline TVector4<FloatType> Normalize(const TVector4<FloatType> &v) {
    FloatType invLen = inverse(Length(v));
    return v * invLen;
}

template <typename FloatType>
TVector3<FloatType> Cross(const TVector3<FloatType> &v1, const TVector3<FloatType> &v2) {
    return TVector3<FloatType>(v1[1] * v2[2] - v1[2] * v2[1],
                               v1[2] * v2[0] - v1[0] * v2[2],
                               v1[0] * v2[1] - v1[1] * v2[0]);
}

template <typename FloatType>
TVector3<FloatType> Reflect(const TVector3<FloatType> &wi, const TVector3<FloatType> &n) {
    return (Float(2.0) * Dot(wi, n)) * n - wi;
}

template <typename FloatType>
TVector3<FloatType> Refract(const TVector3<FloatType> &wi,
                            const TVector3<FloatType> &n,
                            const FloatType cosThetaT,
                            const FloatType eta,
                            const FloatType invEta) {
    FloatType eta_ = if_else(Lt(cosThetaT, Float(0.0)), invEta, eta);
    return n * (Dot(wi, n) * eta_ + cosThetaT) - wi * eta_;
}

template <typename FloatType>
inline FloatType Luminance(const TVector3<FloatType> &v) {
    return v[0] * Float(0.212671) + v[1] * Float(0.715160) + v[2] * Float(0.072169);
}

template <typename FloatType, int N>
inline FloatType Avg(const TVector<FloatType, N> &v) {
    return v.sum() * (Float(1.0) / N);
}

inline void CoordinateSystem(const Vector3 &n, Vector3 &b1, Vector3 &b2) {
    if (n[2] < Float(-1.0 + 1e-6)) {
        b1 = Vector3(Float(0.0), Float(-1.0), Float(0.0));
        b2 = Vector3(Float(-1.0), Float(0.0), Float(0.0));
        return;
    }
    const Float a = Float(1.0) / (Float(1.0) + n[2]);
    const Float b = -n[0] * n[1] * a;
    b1 = Vector3(Float(1.0) - square(n[0]) * a, b, -n[0]);
    b2 = Vector3(b, Float(1.0) - square(n[1]) * a, -n[1]);
}

inline void CoordinateSystem(const ADVector3 &n, ADVector3 &b1, ADVector3 &b2) {
    std::vector<CondExprCPtr> ret = CreateCondExprVec(6);
    BeginIf(Lt(n[2], Float(-1.0 + 1e-6)), ret);
    {
        ADVector3 b1, b2;
        b1 = ADVector3(Const<ADFloat>(0.0), Const<ADFloat>(-1.0), Const<ADFloat>(0.0));
        b2 = ADVector3(Const<ADFloat>(-1.0), Const<ADFloat>(0.0), Const<ADFloat>(0.0));
        SetCondOutput({b1[0], b1[1], b1[2], b2[0], b2[1], b2[2]});
    }
    BeginElse();
    {
        ADVector3 b1, b2;
        const ADFloat a = Float(1.0) / (Float(1.0) + n[2]);
        const ADFloat b = -n[0] * n[1] * a;
        b1 = ADVector3(Float(1.0) - square(n[0]) * a, b, -n[0]);
        b2 = ADVector3(b, Float(1.0) - square(n[1]) * a, -n[1]);
        SetCondOutput({b1[0], b1[1], b1[2], b2[0], b2[1], b2[2]});
    }
    EndIf();
    b1[0] = ret[0];
    b1[1] = ret[1];
    b1[2] = ret[2];
    b2[0] = ret[3];
    b2[1] = ret[4];
    b2[2] = ret[5];
}

inline Float Tent(const Float sample) {
    if (sample < Float(0.5)) {
        return (Float(1.0) - sqrt(Float(2.0) * sample));
    } else {
        return sqrt(Float(2.0) * (sample - Float(0.5))) - Float(1.0);
    }
}

inline ADFloat Tent(const ADFloat sample) {
    CondExprCPtr ret = CondExpr::Create();
    BeginIf(Lt(sample, Float(0.5)), {ret});
    { SetCondOutput({Float(1.0) - sqrt(Float(2.0) * sample)}); }
    BeginElse();
    { SetCondOutput({sqrt(Float(2.0) * (sample - Float(0.5))) - Float(1.0)}); }
    EndIf();
    return ret;
}

template <typename T>
T Clamp(const T v, const T lb, const T ub) {
    return std::min(std::max(v, lb), ub);
}

inline Float Radians(const Float deg) {
    return (c_PI / Float(180.0)) * deg;
}

inline Float *Serialize(const Float v, Float *buffer) {
    *buffer++ = v;
    return buffer;
}

template <typename FloatType>
inline const FloatType *Deserialize(const FloatType *buffer, FloatType &v) {
    v = *buffer++;
    return buffer;
}

template <int _Options>
inline Float *Serialize(const TVector2<Float, _Options> v, Float *buffer) {
    *buffer++ = v[0];
    *buffer++ = v[1];
    return buffer;
}

template <typename FloatType, int _Options>
inline const FloatType *Deserialize(const FloatType *buffer, TVector2<FloatType, _Options> &v) {
    v[0] = *buffer++;
    v[1] = *buffer++;
    return buffer;
}

template <int _Options>
inline Float *Serialize(const TVector3<Float, _Options> &v, Float *buffer) {
    *buffer++ = v[0];
    *buffer++ = v[1];
    *buffer++ = v[2];
    return buffer;
}

template <typename FloatType, int _Options>
inline const FloatType *Deserialize(const FloatType *buffer, TVector3<FloatType, _Options> &v) {
    v[0] = *buffer++;
    v[1] = *buffer++;
    v[2] = *buffer++;
    return buffer;
}

template <int _Options>
inline Float *Serialize(const TVector4<Float, _Options> &v, Float *buffer) {
    *buffer++ = v[0];
    *buffer++ = v[1];
    *buffer++ = v[2];
    *buffer++ = v[3];
    return buffer;
}

template <typename FloatType, int _Options>
inline const FloatType *Deserialize(const FloatType *buffer, TVector4<FloatType, _Options> &v) {
    v[0] = *buffer++;
    v[1] = *buffer++;
    v[2] = *buffer++;
    v[3] = *buffer++;
    return buffer;
}

inline Float *Serialize(const Matrix4x4 &m, Float *buffer) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            *buffer++ = m(j, i);
        }
    }
    return buffer;
}

template <typename FloatType>
inline const FloatType *Deserialize(const FloatType *buffer, TMatrix4x4<FloatType> &m) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            m(j, i) = *buffer++;
        }
    }
    return buffer;
}

inline void ResizeIfNeeded(std::vector<Float> &vec, size_t size) {
    if (vec.size() < size) {
        vec.resize(size);
    }
}

inline int32_t Modulo(const int32_t a, const int32_t b) {
    int32_t r = a % b;
    return (r < 0) ? r + b : r;
}

inline int64_t Modulo(const int64_t a, const int64_t b) {
    int64_t r = a % b;
    return (r < 0) ? r + b : r;
}

inline Float Modulo(const Float a, const Float b) {
    Float r = fmod(a, b);
    return (r < 0.0) ? r + b : r;
}

inline uint32_t FloatToBits(float f) {
    uint32_t ui;
    memcpy(&ui, &f, sizeof(float));
    return ui;
}

inline uint64_t FloatToBits(double f) {
    uint64_t ui;
    memcpy(&ui, &f, sizeof(double));
    return ui;
}

inline double BitsToFloat(uint64_t ui) {
    double f;
    memcpy(&f, &ui, sizeof(uint64_t));
    return f;
}

inline float BitsToFloat(uint32_t ui) {
    float f;
    memcpy(&f, &ui, sizeof(uint32_t));
    return f;
}

// From boost
template <class T>
inline void hash_combine(std::size_t &seed, const T &v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std {
template <typename S, typename T>
struct hash<pair<S, T>> {
    inline size_t operator()(const pair<S, T> &v) const {
        size_t seed = 0;
        ::hash_combine(seed, v.first);
        ::hash_combine(seed, v.second);
        return seed;
    }
};
}

template <typename allocator>
inline bool IsFinite(const std::vector<Float, allocator> &v) {
    for (const auto f : v) {
        if (!std::isfinite(f)) {
            return false;
        }
    }
    return true;
}

template <typename FloatType>
inline FloatType ADEpsilon() {
    return Float(0.0);
}

template <>
inline ADFloat ADEpsilon<ADFloat>() {
    return Const<ADFloat>(1e-6);
}
