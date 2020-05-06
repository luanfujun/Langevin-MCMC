#include "quaternion.h"
#include "utils.h"

Quaternion MakeQuaternion(const Matrix4x4 &m) {
    Quaternion q;
    const Float trace = m(0, 0) + m(1, 1) + m(2, 2);
    if (trace > Float(1e-7)) {
        Float s = sqrt(trace + 1.0);
        q[3] = s / Float(2.0);
        s = Float(0.5) / s;
        q[0] = (m(2, 1) - m(1, 2)) * s;
        q[1] = (m(0, 2) - m(2, 0)) * s;
        q[2] = (m(1, 0) - m(0, 1)) * s;
    } else {
        const int nxt[3] = {1, 2, 0};
        Float _q[3];
        int i = 0;
        if (m(1, 1) > m(0, 0))
            i = 1;
        if (m(2, 2) > m(i, i))
            i = 2;
        int j = nxt[i];
        int k = nxt[j];
        Float s = sqrt((m(i, i) - (m(j, j) + m(k, k))) + Float(1.0));
        _q[i] = s * Float(0.5);
        if (s != Float(0.0))
            s = Float(0.5) / s;
        q[3] = (m(k, j) - m(j, k)) * s;
        _q[j] = (m(j, i) + m(i, j)) * s;
        _q[k] = (m(k, i) + m(i, k)) * s;
        q[0] = _q[0];
        q[1] = _q[1];
        q[2] = _q[2];
    }
    return q;
}

Quaternion Slerp(const Float &t, const Quaternion &q1, const Quaternion &_q2) {
    Quaternion q2 = _q2;
    Float cosTheta = Dot(q1, q2);
    if (cosTheta < Float(0.0)) {
        q2 = -q2;
        cosTheta = -cosTheta;
    }

    if (cosTheta > Float(.9995)) {
        return Normalize(Vector4((Float(1.0) - t) * q1 + t * q2));
    } else {
        const Float theta = acos(Clamp(cosTheta, Float(-1.0), Float(1.0)));
        const Float thetap = theta * t;
        Quaternion qperp = Normalize(Vector4(q2 - q1 * cosTheta));
        return q1 * cos(thetap) + qperp * sin(thetap);
    }
}

ADQuaternion Slerp(const ADFloat &t, const ADQuaternion &q1, const ADQuaternion &_q2) {
    ADFloat _cosTheta = Dot(q1, _q2);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(5);
    BeginIf(Lt(_cosTheta, Float(0.0)), ret);
    {
        ADVector4 q2 = -_q2;
        ADFloat cosTheta = -_cosTheta;
        std::vector<ADFloat> ret = ToADFloatVec(q2);
        ret.push_back(cosTheta);
        SetCondOutput(ret);
    }
    BeginElse();
    {
        std::vector<ADFloat> ret = ToADFloatVec(_q2);
        ret.push_back(_cosTheta);
        SetCondOutput(ret);
    }
    EndIf();
    ADVector4 q2 = ToADVector4(ret);
    ADFloat cosTheta = ret[4];

    ret = CreateCondExprVec(4);
    BeginIf(Gt(cosTheta, Float(.9995)), ret);
    {
        ADVector4 ret = Normalize(ADVector4((Float(1.0) - t) * q1 + t * q2));
        SetCondOutput(ToADFloatVec(ret));
    }
    BeginElse();
    {
        ADFloat theta = acos(cosTheta);
        ADFloat thetap = theta * t;
        ADVector4 qperp = Normalize(ADVector4(q2 - q1 * cosTheta));
        ADVector4 ret = q1 * cos(thetap) + qperp * sin(thetap);
        SetCondOutput(ToADFloatVec(ret));
    }
    EndIf();
    return ToADVector4(ret);
}
