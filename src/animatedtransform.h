#pragma once

#include "commondef.h"
#include "quaternion.h"
#include "transform.h"

int GetAnimatedTransformSerializedSize();

void Decompose(const Matrix4x4 &m, Vector3 *t, Quaternion *r);

template <typename FloatType>
struct TAnimatedTransform {
    FloatType isMoving;
    TVector3<FloatType, Eigen::DontAlign> translate[2];
    TVector4<FloatType, Eigen::DontAlign> rotate[2];
};

template <>
struct TAnimatedTransform<ADFloat> {
    ADFloat isMoving;
    ADVector3 translate[2];
    ADVector4 rotate[2];
};

typedef TAnimatedTransform<Float> AnimatedTransform;
typedef TAnimatedTransform<ADFloat> ADAnimatedTransform;

inline Float *Serialize(const AnimatedTransform &transform, Float *buffer) {
    buffer = Serialize(transform.isMoving, buffer);
    buffer = Serialize(transform.translate[0], buffer);
    buffer = Serialize(transform.translate[1], buffer);
    buffer = Serialize(transform.rotate[0], buffer);
    buffer = Serialize(transform.rotate[1], buffer);
    return buffer;
}

template <typename FloatType>
const FloatType *Deserialize(const FloatType *buffer, TAnimatedTransform<FloatType> &transform) {
    buffer = Deserialize(buffer, transform.isMoving);
    buffer = Deserialize(buffer, transform.translate[0]);
    buffer = Deserialize(buffer, transform.translate[1]);
    buffer = Deserialize(buffer, transform.rotate[0]);
    buffer = Deserialize(buffer, transform.rotate[1]);
    return buffer;
}

inline AnimatedTransform MakeAnimatedTransform(const Matrix4x4 &m0, const Matrix4x4 &m1) {
    AnimatedTransform ret;
    ret.isMoving = BoolToFloat(m0 != m1);
    Vector3 translate;
    Vector4 rotate;
    Decompose(m0, &translate, &rotate);
    ret.translate[0] = translate;
    ret.rotate[0] = rotate;
    if (ret.isMoving == FTRUE) {
        Decompose(m1, &translate, &rotate);
        ret.translate[1] = translate;
        ret.rotate[1] = rotate;
    } else {
        ret.translate[1] = ret.translate[0];
        ret.rotate[1] = ret.rotate[0];
    }
    return ret;
}

Float *Serialize(const AnimatedTransform &transform, Float *buffer);

Matrix4x4 Interpolate(const AnimatedTransform &transform, const Float time);

ADMatrix4x4 Interpolate(const ADAnimatedTransform &transform, const ADFloat time);
ADMatrix4x4 ToMatrix4x4(const ADAnimatedTransform &transform);

template <typename FloatType>
TAnimatedTransform<FloatType> Invert(const TAnimatedTransform<FloatType> &transform) {
    TAnimatedTransform<FloatType> ret;
    ret.isMoving = transform.isMoving;
    ret.rotate[0] = TVector4<FloatType>(-transform.rotate[0][0],
                                        -transform.rotate[0][1],
                                        -transform.rotate[0][2],
                                        transform.rotate[0][3]);
    ret.rotate[1] = TVector4<FloatType>(-transform.rotate[1][0],
                                        -transform.rotate[1][1],
                                        -transform.rotate[1][2],
                                        transform.rotate[1][3]);
    TMatrix4x4<FloatType> rot0 = ToMatrix4x4<FloatType>(ret.rotate[0]);
    TMatrix4x4<FloatType> rot1 = ToMatrix4x4<FloatType>(ret.rotate[1]);
    ret.translate[0] = -XformVector<FloatType>(rot0, transform.translate[0]);
    ret.translate[1] = -XformVector<FloatType>(rot1, transform.translate[1]);
    return ret;
}
