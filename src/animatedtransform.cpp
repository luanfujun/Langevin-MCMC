#include "animatedtransform.h"
#include "transform.h"

int GetAnimatedTransformSerializedSize() {
    return 1 +      // isMoving
           3 * 2 +  // translate[2]
           4 * 2;   // rotate[2]
}

void Decompose(const Matrix4x4 &m, Vector3 *t, Quaternion *r) {
    Matrix3x3 A = m.topLeftCorner<3, 3>();
    Eigen::JacobiSVD<Matrix3x3> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Matrix3x3 U = svd.matrixU();
    Matrix3x3 V = svd.matrixV();
    Matrix3x3 S = svd.singularValues().asDiagonal();
    if (svd.singularValues().prod() < 0) {
        S = -S;
        U = -U;
    }
    Matrix3x3 Q = U * V.transpose();
    Matrix3x3 P = V * S * V.transpose();
    if (fabs(P(0, 0) - Float(1.0)) > Float(1e-5) || fabs(P(1, 1) - Float(1.0)) > Float(1e-5) ||
        fabs(P(2, 2) - Float(1.0)) > Float(1e-5)) {
        Error("Scaling in animation");
    }

    Matrix4x4 Q4 = Matrix4x4::Identity();
    Q4.topLeftCorner<3, 3>() = Q;
    *r = MakeQuaternion(Q4);
    *t = Vector3(m(0, 3), m(1, 3), m(2, 3));
}

Matrix4x4 Interpolate(const AnimatedTransform &transform, const Float time) {
    if (transform.isMoving == FFALSE) {
        Vector3 translate = transform.translate[0];
        Vector4 rotate = transform.rotate[0];
        return Translate(translate) * ToMatrix4x4(rotate);
    } else {
        Vector3 trans =
            (Float(1.0) - time) * transform.translate[0] + time * transform.translate[1];
        Quaternion rot = Slerp(time, transform.rotate[0], transform.rotate[1]);
        return Translate(trans) * ToMatrix4x4(rot);
    }
}

ADMatrix4x4 Interpolate(const ADAnimatedTransform &transform, const ADFloat time) {
    std::vector<CondExprCPtr> ret = CreateCondExprVec(16);
    BeginIf(Eq(transform.isMoving, FFALSE), ret);
    {
        ADMatrix4x4 ret = Translate(transform.translate[0]) * ToMatrix4x4(transform.rotate[0]);
        SetCondOutput(ToADFloatVec(ret));
    }
    BeginElse();
    {
        ADVector3 trans =
            (Float(1.0) - time) * transform.translate[0] + time * transform.translate[1];
        ADQuaternion rot = Slerp(time, transform.rotate[0], transform.rotate[1]);
        ADMatrix4x4 ret = Translate(trans) * ToMatrix4x4(rot);
        SetCondOutput(ToADFloatVec(ret));
    }
    EndIf();
    return ToADMatrix4x4(ret);
}

ADMatrix4x4 ToMatrix4x4(const ADAnimatedTransform &transform) {
    return Translate(transform.translate[0]) * ToMatrix4x4(transform.rotate[0]);
}
