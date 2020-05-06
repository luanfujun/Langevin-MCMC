#pragma once

#include "commondef.h"
#include "utils.h"

using Quaternion = Vector4;
using ADQuaternion = ADVector4;
template <typename T>
using TQuaternion = TVector4<T>;

Quaternion MakeQuaternion(const Matrix4x4 &m);

template <typename FloatType>
TMatrix4x4<FloatType> ToMatrix4x4(const TQuaternion<FloatType> &q) {
    const FloatType xx = q[0] * q[0], yy = q[1] * q[1], zz = q[2] * q[2];
    const FloatType xy = q[0] * q[1], xz = q[0] * q[2], yz = q[1] * q[2];
    const FloatType wx = q[0] * q[3], wy = q[1] * q[3], wz = q[2] * q[3];

    TMatrix4x4<FloatType> m;
    m(0, 0) = Float(1.0) - Float(2.0) * (yy + zz);
    m(0, 1) = Float(2.0) * (xy + wz);
    m(0, 2) = Float(2.0) * (xz - wy);
    m(0, 3) = Const<FloatType>(0.0);
    m(1, 0) = Float(2.0) * (xy - wz);
    m(1, 1) = Float(1.0) - Float(2.0) * (xx + zz);
    m(1, 2) = Float(2.0) * (yz + wx);
    m(1, 3) = Const<FloatType>(0.0);
    m(2, 0) = Float(2.0) * (xz + wy);
    m(2, 1) = Float(2.0) * (yz - wx);
    m(2, 2) = Float(1.0) - Float(2.0) * (xx + yy);
    m(2, 3) = Const<FloatType>(0.0);
    m(3, 0) = Const<FloatType>(0.0);
    m(3, 1) = Const<FloatType>(0.0);
    m(3, 2) = Const<FloatType>(0.0);
    m(3, 3) = Const<FloatType>(1.0);

    return m.transpose();
}

Quaternion Slerp(const Float &t, const Quaternion &q1, const Quaternion &q2);
ADQuaternion Slerp(const ADFloat &t, const ADQuaternion &q1, const ADQuaternion &q2);
