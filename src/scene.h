#pragma once

#include "commondef.h"
#include "bounds.h"
#include "ray.h"
#include "dptoptions.h"

#include <memory>
#include <string>

// #include <embree2/rtcore.h>
// #include <embree2/rtcore_ray.h>
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>

struct Camera;
struct Shape;
struct Light;
struct PiecewiseConstant1D;
struct ShapeInst;
struct EnvLight;

struct Scene {
    Scene(std::shared_ptr<DptOptions> &options,
          const std::shared_ptr<const Camera> &camera,
          const std::vector<std::shared_ptr<const Shape>> &objects,
          const std::vector<std::shared_ptr<const Light>> &lights,
          const std::shared_ptr<const EnvLight> &envLight,
          const std::string &outputName);
    ~Scene();

    std::shared_ptr<DptOptions> options;
    std::shared_ptr<const Camera> camera;
    std::vector<std::shared_ptr<const Shape>> objects;
    std::vector<std::shared_ptr<const Light>> lights;
    std::unique_ptr<PiecewiseConstant1D> lightDist;
    std::shared_ptr<const EnvLight> envLight;
    BSphere bSphere;

    std::string outputName;

    RTCDevice rtcDevice;
    RTCScene rtcScene;

    Float lightWeightSum;
};

bool Intersect(const Scene *scene,
               const Float time,
               const RaySegment &raySeg,
               ShapeInst &shapeInst);

bool Occluded(const Scene *scene, const Float time, const Ray &ray, const Float dist);
bool Occluded(const Scene *scene, const Float time, const RaySegment &raySeg);

const Light *PickLight(const Scene *scene, const Float u, Float &prob);

const Float PickLightProb(const Scene *scene, const Light *light);

int GetSceneSerializedSize();

Float *Serialize(const Scene *scene, Float *buffer);
