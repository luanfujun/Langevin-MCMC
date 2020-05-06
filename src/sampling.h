#pragma once

#include "commondef.h"
#include "utils.h"

template <typename FloatType>
inline TVector3<FloatType> SampleSphere(const TVector2<FloatType> coord, FloatType &jacobian) {
    const FloatType scaledTheta = c_TWOPI * coord[0];
    const FloatType scaledPhi = c_PI * coord[1];
    const FloatType sinPhi = sin(scaledPhi);
    const FloatType cosPhi = cos(scaledPhi);
    TVector3<FloatType> dir(sinPhi * cos(scaledTheta), sinPhi * sin(scaledTheta), cosPhi);
    // jacobian = fabs(sinPhi);
    jacobian = fabs(sinPhi) * c_TWOPI * c_PI;
    return dir;
}

template <typename FloatType>
inline TVector3<FloatType> SampleSphere(const TVector2<FloatType> coord) {
    FloatType jacobian;
    return SampleSphere(coord, jacobian);
}

inline Float patan2(const Float y, const Float x) {
    if (y == Float(0.0) && x == Float(0.0)) {
        return Float(0.0);
    }
    Float result = atan2(y, x);
    if (result < 0.0) {
        result += c_TWOPI;
    }

    return result;
}

inline Vector2 ToSphericalCoord(const Vector3 &dir, Float &jacobian) {
    Float theta = patan2(dir[1], dir[0]) * c_INVTWOPI;
    Float phi = acos(dir[2]);
    // jacobian = fabs(sin(phi));
    jacobian = fabs(sin(phi)) * c_TWOPI * c_PI;
    phi *= c_INVPI;
    return Vector2(theta, phi);
}

inline Vector2 ToSphericalCoord(const Vector3 &dir) {
    Float jacobian;
    return ToSphericalCoord(dir, jacobian);
}

inline Vector2 SampleConcentricDisc(const Vector2 rndParam) {
    Float r1 = Float(2.0) * rndParam[0] - Float(1.0);
    Float r2 = Float(2.0) * rndParam[1] - Float(1.0);

    // http://psgraphics.blogspot.ch/2011/01/improved-code-for-concentric-map.html
    Float phi, r;
    if (r1 == 0 || r2 == 0) {
        r = phi = 0;
    } else if (square(r1) > square(r2)) {
        r = r1;
        phi = c_PIOVERFOUR * (r2 / r1);
    } else {
        r = r2;
        phi = c_PIOVERTWO - (r1 / r2) * c_PIOVERFOUR;
    }

    Float sinPhi = sin(phi);
    Float cosPhi = cos(phi);

    return Vector2(r * cosPhi, r * sinPhi);
}

inline ADVector2 SampleConcentricDisc(const ADVector2 rndParam) {
    ADFloat r1 = Float(2.0) * rndParam[0] - Float(1.0);
    ADFloat r2 = Float(2.0) * rndParam[1] - Float(1.0);

    std::vector<CondExprCPtr> ret = CreateCondExprVec(2);
    BeginIf(Eq(r1, Float(0.0)), ret);
    { SetCondOutput({Const<ADFloat>(0.0), Const<ADFloat>(0.0)}); }
    BeginElseIf(Eq(r2, Float(0.0)));
    { SetCondOutput({Const<ADFloat>(0.0), Const<ADFloat>(0.0)}); }
    BeginElseIf(Gt(square(r1), square(r2)));
    {
        ADFloat r = r1;
        ADFloat phi = c_PIOVERFOUR * (r2 / r1);
        SetCondOutput({r, phi});
    }
    BeginElse();
    {
        ADFloat r = r2;
        ADFloat phi = c_PIOVERTWO - (r1 / r2) * c_PIOVERFOUR;
        SetCondOutput({r, phi});
    }
    EndIf();
    ADFloat r = ret[0];
    ADFloat phi = ret[1];

    ADFloat sinPhi = sin(phi);
    ADFloat cosPhi = cos(phi);

    return ADVector2(r * cosPhi, r * sinPhi);
}

template <typename FloatType>
inline TVector3<FloatType> SampleCosHemisphere(const TVector2<FloatType> rndParam) {
    FloatType phi = c_TWOPI * rndParam[0];
    FloatType tmp = sqrt(fmax(Float(1.0) - rndParam[1], ADEpsilon<FloatType>()));
    TVector3<FloatType> ret(
        cos(phi) * tmp, sin(phi) * tmp, sqrt(fmax(rndParam[1], ADEpsilon<FloatType>())));
    return ret;
}
