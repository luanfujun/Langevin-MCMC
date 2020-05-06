#pragma once

#include "commondef.h"
#include "quaternion.h"
#include "utils.h"

template <typename T>
TMatrix4x4<T> Translate(const TVector3<T> &delta) {
    // TMatrix4x4<T> m;
    // m << Const<T>(1.0), Const<T>(0.0), Const<T>(0.0), delta[0], Const<T>(0.0), Const<T>(1.0),
    //     Const<T>(0.0), delta[1], Const<T>(0.0), Const<T>(0.0), Const<T>(1.0), delta[2],
    //     Const<T>(0.0), Const<T>(0.0), Const<T>(0.0), Const<T>(1.0);
    TMatrix4x4<T> m; 
    m(0, 0) = Const<T>(1.0);
    m(0, 1) = Const<T>(0.0);
    m(0, 2) = Const<T>(0.0);
    m(0, 3) = delta[0]; 

    m(1, 0) = Const<T>(0.0);
    m(1, 1) = Const<T>(1.0);
    m(1, 2) = Const<T>(0.0);
    m(1, 3) = delta[1];
    
    m(2, 0) = Const<T>(0.0);
    m(2, 1) = Const<T>(0.0);
    m(2, 2) = Const<T>(1.0);
    m(2, 3) = delta[2];

    m(3, 0) = Const<T>(0.0);
    m(3, 1) = Const<T>(0.0);
    m(3, 2) = Const<T>(0.0);
    m(3, 3) = Const<T>(1.0);

    return m;
}
Matrix4x4 Scale(const Vector3 &scale);
Matrix4x4 Rotate(const Float angle, const Vector3 &axis);
Matrix4x4 LookAt(const Vector3 &pos, const Vector3 &look, const Vector3 &up);
Matrix4x4 Perspective(const Float fov, const Float clipNear, const Float clipFar);

template <typename FloatType>
inline TVector3<FloatType> XformPoint(const TMatrix4x4<FloatType> &xform,
                                      const TVector3<FloatType> &pt) {
    TVector4<FloatType> tpt(
        xform(0, 0) * pt[0] + xform(0, 1) * pt[1] + xform(0, 2) * pt[2] + xform(0, 3),
        xform(1, 0) * pt[0] + xform(1, 1) * pt[1] + xform(1, 2) * pt[2] + xform(1, 3),
        xform(2, 0) * pt[0] + xform(2, 1) * pt[1] + xform(2, 2) * pt[2] + xform(2, 3),
        xform(3, 0) * pt[0] + xform(3, 1) * pt[1] + xform(3, 2) * pt[2] + xform(3, 3));
    FloatType invW = inverse(tpt[3]);
    return TVector3<FloatType>(tpt[0] * invW, tpt[1] * invW, tpt[2] * invW);
}

template <typename FloatType>
inline TVector3<FloatType> XformVector(const TMatrix4x4<FloatType> &xform,
                                       const TVector3<FloatType> &vec) {
    return TVector3<FloatType>(xform(0, 0) * vec[0] + xform(0, 1) * vec[1] + xform(0, 2) * vec[2],
                               xform(1, 0) * vec[0] + xform(1, 1) * vec[1] + xform(1, 2) * vec[2],
                               xform(2, 0) * vec[0] + xform(2, 1) * vec[1] + xform(2, 2) * vec[2]);
}

template <typename FloatType>
inline TVector3<FloatType> XformNormal(const TMatrix4x4<FloatType> &invXform,
                                       const TVector3<FloatType> &vec) {
    return TVector3<FloatType>(
        invXform(0, 0) * vec[0] + invXform(1, 0) * vec[1] + invXform(2, 0) * vec[2],
        invXform(0, 1) * vec[0] + invXform(1, 1) * vec[1] + invXform(2, 1) * vec[2],
        invXform(0, 2) * vec[0] + invXform(1, 2) * vec[1] + invXform(2, 2) * vec[2]);
}
