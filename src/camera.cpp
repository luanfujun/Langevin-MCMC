#include "camera.h"
#include "transform.h"

int GetCameraSerializedSize() {
    return 16 +                                    // sampleToCam
           GetAnimatedTransformSerializedSize() +  // camToWorld
           1 +                                     // screenPixelCount
           1;                                      // dist
}

Camera::Camera(const AnimatedTransform &camToWorld,
               const Float fov,
               const std::shared_ptr<Image3> film,
               const Float nearClip,
               const Float farClip)
    : camToWorld(camToWorld),
      worldToCamera(Invert(camToWorld)),
      film(film),
      nearClip(nearClip),
      farClip(farClip) {
    Float aspect = (Float)film->pixelWidth / (Float)film->pixelHeight;
    camToSample = Scale(Vector3(-Float(0.5), -Float(0.5) * aspect, Float(1.0))) *
                  Translate(Vector3(-Float(1.0), -Float(1.0) / aspect, Float(0.0))) *
                  Perspective(fov, nearClip, farClip);

    sampleToCam = camToSample.inverse();
    dist = film->pixelWidth / (Float(2.0) * tan((fov / Float(2.0)) * (c_PI / Float(180.0))));
}

Float *Serialize(const Camera *camera, Float *buffer) {
    buffer = Serialize(camera->sampleToCam, buffer);
    buffer = Serialize(camera->camToWorld, buffer);
    buffer = Serialize(Float(GetPixelHeight(camera) * GetPixelWidth(camera)), buffer);
    buffer = Serialize(camera->dist, buffer);
    return buffer;
}

void SamplePrimary(const Camera *camera,
                   const Vector2 screenPos,
                   const Float time,
                   RaySegment &raySeg) {
    Ray &ray = raySeg.ray;
    ray.org = XformPoint(camera->sampleToCam, Vector3(screenPos[0], screenPos[1], Float(0.0)));
    ray.dir = Normalize(ray.org);
    Float invZ = inverse(ray.dir[2]);
    Matrix4x4 toWorld = Interpolate(camera->camToWorld, time);
    ray.org = XformPoint(toWorld, Vector3(Vector3::Zero()));
    ray.dir = XformVector(toWorld, ray.dir);
    raySeg.minT = camera->nearClip * invZ;
    raySeg.maxT = camera->farClip * invZ;
}

void SamplePrimary(const ADMatrix4x4 &sampleToCam,
                   const ADAnimatedTransform &camToWorld,
                   const ADVector2 screenPos,
                   const ADFloat time,
                   const bool isStatic,
                   ADRay &ray) {
    ray.org = XformPoint(sampleToCam, ADVector3(screenPos[0], screenPos[1], Const<ADFloat>(0.0)));
    ray.dir = Normalize(ray.org);
    ADMatrix4x4 toWorld = isStatic ? ToMatrix4x4(camToWorld) : Interpolate(camToWorld, time);
    ray.org = XformPoint(toWorld,
                         ADVector3(Const<ADFloat>(0.0), Const<ADFloat>(0.0), Const<ADFloat>(0.0)));
    ray.dir = XformVector(toWorld, ray.dir);
}

bool ProjectPoint(const Camera *camera, const Vector3 &p, const Float time, Vector2 &screenPos) {
    Matrix4x4 wtc = Interpolate(camera->worldToCamera, time);
    Vector3 camP = XformPoint(wtc, p);
    if (camP[2] < camera->nearClip || camP[2] > camera->farClip) {
        return false;
    }

    Vector3 rasterP = XformPoint(camera->camToSample, camP);

    if (rasterP[0] < Float(0.0) || rasterP[0] > Float(1.0) || rasterP[1] < Float(0.0) ||
        rasterP[1] > Float(1.0)) {
        return false;
    }

    screenPos[0] = rasterP[0];
    screenPos[1] = rasterP[1];
    return true;
}
