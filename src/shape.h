#pragma once

#include "commondef.h"
#include "bounds.h"
#include "ray.h"

// #include <embree2/rtcore.h>
// #include <embree2/rtcore_ray.h>
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>

struct AreaLight;
struct BSDF;

enum class ShapeType { TriangleMesh };

int GetMaxShapeSerializedSize();

template <typename FloatType>
struct TIntersection {
    TVector3<FloatType> position;
    TVector3<FloatType> shadingNormal;
    TVector3<FloatType> geomNormal;
};

using Intersection = TIntersection<Float>;
using ADIntersection = TIntersection<ADFloat>;

struct Shape {
    Shape(const std::shared_ptr<const BSDF> bsdf) : bsdf(bsdf), areaLight(nullptr) {
    }
    virtual ShapeType GetType() const = 0;
    virtual ShapeID RtcRegister(const RTCScene &scene, const RTCDevice &device) const = 0;
    virtual void Serialize(const PrimID primID, Float *buffer) const = 0;
    virtual bool Intersect(const PrimID &primID,
                           const Float time,
                           const RaySegment &raySeg,
                           Intersection &isect,
                           Vector2 &st) const = 0;
    // Find sampleParam so that Shape::Sample(sampleParam) returns the same position
    virtual Vector2 GetSampleParam(const PrimID &primID,
                                   const Vector3 &position,
                                   const Float time) const = 0;
    virtual void SetAreaLight(const AreaLight *areaLight);
    virtual PrimID Sample(const Float u) const = 0;
    virtual void Sample(const Vector2 rndParam,
                        const Float time,
                        const PrimID primID,
                        Vector3 &position,
                        Vector3 &normal,
                        Float *pdf) const = 0;
    virtual Float SamplePdf() const = 0;
    virtual BBox GetBBox() const = 0;
    virtual bool IsMoving() const = 0;

    const std::shared_ptr<const BSDF> bsdf;
    const AreaLight *areaLight;
};

struct ShapeInst {
    const Shape *obj;
    PrimID primID;
    Vector2 st;
};

const ADFloat *Intersect(const ADFloat *buffer,
                         const ADRay &ray,
                         const ADFloat time,
                         const bool isStatic,
                         ADIntersection &isect,
                         ADVector2 &st);

void SampleShape(const ADFloat *buffer,
                 const ADVector2 rndParam,
                 const ADFloat time,
                 const bool isStatic,
                 ADVector3 &position,
                 ADVector3 &normal,
                 ADFloat &pdf);

const ADFloat *SampleShapePdf(const ADFloat *buffer, ADFloat &shapePdf);
