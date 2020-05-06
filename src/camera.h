#pragma once

#include "commondef.h"
#include "animatedtransform.h"
#include "image.h"
#include "ray.h"

int GetCameraSerializedSize();

struct Camera {
    Camera(const AnimatedTransform &camToWorld,
           const Float fov,
           const std::shared_ptr<Image3> film,
           const Float nearClip,
           const Float farClip);

    Matrix4x4 sampleToCam, camToSample;
    AnimatedTransform camToWorld, worldToCamera;

    std::shared_ptr<Image3> film;
    Float nearClip, farClip;
    Float dist;
};

Float *Serialize(const Camera *camera, Float *buffer);

void SamplePrimary(const Camera *camera,
                   const Vector2 screenPos,
                   const Float time,
                   RaySegment &raySeg);

void SamplePrimary(const ADMatrix4x4 &sampleToCam,
                   const ADAnimatedTransform &camToWorld,
                   const ADVector2 screenPos,
                   const ADFloat time,
                   const bool isStatic,
                   ADRay &ray);

bool ProjectPoint(const Camera *camera, const Vector3 &p, const Float t, Vector2 &screenPos);

inline std::shared_ptr<Image3> GetFilm(const Camera *camera) {
    return camera->film;
}

inline int GetPixelHeight(const Camera *camera) {
    return camera->film->pixelHeight;
}

inline int GetPixelWidth(const Camera *camera) {
    return camera->film->pixelWidth;
}
