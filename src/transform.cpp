#include "transform.h"
#include "utils.h"

Matrix4x4 Scale(const Vector3 &scale) {
    Matrix4x4 m;
    m << scale[0], 0, 0, 0, 0, scale[1], 0, 0, 0, 0, scale[2], 0, 0, 0, 0, 1;
    return m;
}

Matrix4x4 Rotate(const Float angle, const Vector3 &axis) {
    const Vector3 a = Normalize(axis);
    const Float s = sin(Radians(angle));
    const Float c = cos(Radians(angle));
    Float m[4][4];

    m[0][0] = a[0] * a[0] + (Float(1.0) - a[0] * a[0]) * c;
    m[0][1] = a[0] * a[1] * (Float(1.0) - c) - a[2] * s;
    m[0][2] = a[0] * a[2] * (Float(1.0) - c) + a[1] * s;
    m[0][3] = 0;

    m[1][0] = a[0] * a[1] * (Float(1.0) - c) + a[2] * s;
    m[1][1] = a[1] * a[1] + (Float(1.0) - a[1] * a[1]) * c;
    m[1][2] = a[1] * a[2] * (Float(1.0) - c) - a[0] * s;
    m[1][3] = 0;

    m[2][0] = a[0] * a[2] * (Float(1.0) - c) - a[1] * s;
    m[2][1] = a[1] * a[2] * (Float(1.0) - c) + a[0] * s;
    m[2][2] = a[2] * a[2] + (Float(1.0) - a[2] * a[2]) * c;
    m[2][3] = 0;

    m[3][0] = 0;
    m[3][1] = 0;
    m[3][2] = 0;
    m[3][3] = 1;

    Matrix4x4 mat;
    mat << m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], m[1][2], m[1][3], m[2][0], m[2][1],
        m[2][2], m[2][3], m[3][0], m[3][1], m[3][2], m[3][3];
    return mat;
}

Matrix4x4 LookAt(const Vector3 &pos, const Vector3 &look, const Vector3 &up) {
    Float m[4][4];
    // Initialize fourth column of viewing matrix
    m[0][3] = pos[0];
    m[1][3] = pos[1];
    m[2][3] = pos[2];
    m[3][3] = 1;

    // Initialize first three columns of viewing matrix
    Vector3 dir = Normalize(Vector3(look - pos));
    if (Length(Cross(Normalize(up), dir)) == 0) {
        throw std::runtime_error(
            "[Lookat] Up vector and viewing direction passed"
            " are point in the same direction");
        return Matrix4x4::Zero();
    }
    Vector3 left = Normalize(Cross(Normalize(up), dir));
    Vector3 newUp = Cross(dir, left);
    m[0][0] = left[0];
    m[1][0] = left[1];
    m[2][0] = left[2];
    m[3][0] = 0.;
    m[0][1] = newUp[0];
    m[1][1] = newUp[1];
    m[2][1] = newUp[2];
    m[3][1] = 0.;
    m[0][2] = dir[0];
    m[1][2] = dir[1];
    m[2][2] = dir[2];
    m[3][2] = 0.;

    Matrix4x4 mat;
    mat << m[0][0], m[0][1], m[0][2], m[0][3], m[1][0], m[1][1], m[1][2], m[1][3], m[2][0], m[2][1],
        m[2][2], m[2][3], m[3][0], m[3][1], m[3][2], m[3][3];
    return mat;
}

Matrix4x4 Perspective(const Float fov, const Float clipNear, const Float clipFar) {
    Float recip = Float(1.0) / (clipFar - clipNear);

    /* Perform a scale so that the field of view is mapped
     * to the interval [-1, 1] */
    Float cot = Float(1.0) / tan(Radians(fov / 2.0));

    Matrix4x4 m;
    m << cot, 0, 0, 0, 0, cot, 0, 0, 0, 0, clipFar *recip, -clipNear *clipFar *recip, 0, 0, 1, 0;

    return m;
}
