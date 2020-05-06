#include "scene.h"
#include "distribution.h"
#include "shape.h"
#include "light.h"
#include "camera.h"
#include "bounds.h"

Scene::Scene(std::shared_ptr<DptOptions> &options,
             const std::shared_ptr<const Camera> &camera,
             const std::vector<std::shared_ptr<const Shape>> &objects,
             const std::vector<std::shared_ptr<const Light>> &lights,
             const std::shared_ptr<const EnvLight> &envLight,
             const std::string &outputName)
    : options(options),
      camera(camera),
      objects(objects),
      lights(lights),
      envLight(envLight),
      outputName(outputName) {
    // Build light cdf
    std::vector<Float> weights(lights.size());
    lightWeightSum = Float(0.0);
    for (size_t i = 0; i < weights.size(); i++) {
        weights[i] = lights[i]->samplingWeight;
        lightWeightSum += weights[i];
    }
    lightDist =
        std::unique_ptr<PiecewiseConstant1D>(new PiecewiseConstant1D(&weights[0], weights.size()));
    rtcDevice = rtcNewDevice(NULL);
    rtcScene = rtcNewScene(rtcDevice);
    // rtcSetSceneFlags(rtcScene,RTC_BUILD_QUALITY_MEDIUM | RTC_SCENE_FLAG_NONE | RTC_BUILD_QUALITY_HIGH | RTC_SCENE_FLAG_ROBUST); // EMBREE_FIXME: set proper scene flags
    // rtcSetSceneBuildQuality(rtcScene,RTC_BUILD_QUALITY_MEDIUM | RTC_SCENE_FLAG_NONE | RTC_BUILD_QUALITY_HIGH | RTC_SCENE_FLAG_ROBUST); // EMBREE_FIXME: set proper build quality
    
    BBox bbox;
    for (auto &obj : objects) {
        obj->RtcRegister(rtcScene, rtcDevice);
        bbox = Merge(bbox, obj->GetBBox());
    }
    bSphere = BSphere(bbox);
    bSphere.radius *= Float(1000.0); // important: ensure it's far enough for MIS weighting
    std::cout << "bSphere: center = " << bSphere.center[0] << "," << 
                                         bSphere.center[1] << "," << 
                                         bSphere.center[2] << 
                         " radius = " << bSphere.radius << std::endl; 
    rtcCommitScene(rtcScene);
}

Scene::~Scene() {
    rtcReleaseScene(rtcScene);
    rtcReleaseDevice (rtcDevice);
}

static inline RTCRayHit ToRTCHitRay(const Float time, const RaySegment &raySeg) {
    const Ray &ray = raySeg.ray;
    RTCRayHit rtcRay; // EMBREE_FIXME: use RTCRay for occlusion rays
    rtcRay.ray.flags = 0;
     
    assert(std::isfinite(ray.org[0]));
    assert(std::isfinite(ray.dir[0]));
    rtcRay.ray.org_x = float(ray.org[0]);
    rtcRay.ray.dir_x = float(ray.dir[0]);
    assert(std::isfinite(ray.org[1]));
    assert(std::isfinite(ray.dir[1]));
    rtcRay.ray.org_y = float(ray.org[1]);
    rtcRay.ray.dir_y = float(ray.dir[1]);
    assert(std::isfinite(ray.org[2]));
    assert(std::isfinite(ray.dir[2]));
    rtcRay.ray.org_z = float(ray.org[2]);
    rtcRay.ray.dir_z = float(ray.dir[2]);

    rtcRay.ray.tnear = float(raySeg.minT);
    rtcRay.ray.tfar = float(raySeg.maxT);
    rtcRay.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rtcRay.hit.primID = RTC_INVALID_GEOMETRY_ID;
    rtcRay.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
    rtcRay.ray.mask = 0xFFFFFFFF;
    rtcRay.ray.time = float(time);
    return rtcRay;
}

static inline RTCRay ToRTCRay(const Float time, const RaySegment &raySeg) {
    const Ray &ray = raySeg.ray;
    RTCRay rtcRay; // EMBREE_FIXME: use RTCRay for occlusion rays
    rtcRay.flags = 0;
     
    assert(std::isfinite(ray.org[0]));
    assert(std::isfinite(ray.dir[0]));
    rtcRay.org_x = float(ray.org[0]);
    rtcRay.dir_x = float(ray.dir[0]);
    assert(std::isfinite(ray.org[1]));
    assert(std::isfinite(ray.dir[1]));
    rtcRay.org_y = float(ray.org[1]);
    rtcRay.dir_y = float(ray.dir[1]);
    assert(std::isfinite(ray.org[2]));
    assert(std::isfinite(ray.dir[2]));
    rtcRay.org_z = float(ray.org[2]);
    rtcRay.dir_z = float(ray.dir[2]);

    rtcRay.tnear = float(raySeg.minT);
    rtcRay.tfar = float(raySeg.maxT);
    rtcRay.mask = 0xFFFFFFFF;
    rtcRay.time = float(time);
    return rtcRay;
}

bool Intersect(const Scene *scene,
               const Float time,
               const RaySegment &raySeg,
               ShapeInst &shapeInst) {
    RTCRayHit rtcRay = ToRTCHitRay(time, raySeg);
    {
      RTCIntersectContext context;
      rtcInitIntersectContext(&context);
      rtcIntersect1(scene->rtcScene,&context,&rtcRay);
      rtcRay.hit.Ng_x = -rtcRay.hit.Ng_x; // EMBREE_FIXME: only correct for triangles,quads, and subdivision surfaces
      rtcRay.hit.Ng_y = -rtcRay.hit.Ng_y;
      rtcRay.hit.Ng_z = -rtcRay.hit.Ng_z;
    }
    if (rtcRay.hit.geomID == RTC_INVALID_GEOMETRY_ID) {
        return false;
    }

    shapeInst.obj = scene->objects[rtcRay.hit.geomID].get();
    shapeInst.primID = rtcRay.hit.primID;
    return true;
}

bool Occluded(const Scene *scene, const Float time, const Ray &ray, const Float dist) {
    Float minT, maxT;
    if (dist == std::numeric_limits<Float>::infinity()) {
        minT = c_IsectEpsilon;
        maxT = std::numeric_limits<Float>::infinity();
    } else {
        minT = c_IsectEpsilon;
        maxT = (Float(1.0) - c_ShadowEpsilon) * dist;
    }
    return Occluded(scene, time, RaySegment{ray, minT, maxT});
}

bool Occluded(const Scene *scene, const Float time, const RaySegment &raySeg) {
    RTCRay rtcRay = ToRTCRay(time, raySeg);
    {
      RTCIntersectContext context;
      rtcInitIntersectContext(&context);
      rtcOccluded1(scene->rtcScene,&context,&rtcRay);
      // EMBREE_FIXME: rtcRay is occluded when rtcRay.tfar < 0.0f
    }
    return rtcRay.tfar < Float(0.0f);
}

const Light *PickLight(const Scene *scene, const Float u, Float &prob) {
    int lightId = scene->lightDist->SampleDiscrete(u, &prob);
    return scene->lights[lightId].get();
}

const Float PickLightProb(const Scene *scene, const Light *light) {
    return light->samplingWeight / scene->lightWeightSum;
}

int GetSceneSerializedSize() {
    return 1 + GetCameraSerializedSize() + GetBSphereSerializedSize();
}

Float *Serialize(const Scene *scene, Float *buffer) {
    buffer = Serialize(BoolToFloat(scene->options->useLightCoordinateSampling), buffer);
    buffer = Serialize(scene->camera.get(), buffer);
    buffer = Serialize(scene->bSphere, buffer);
    return buffer;
}
