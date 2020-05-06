#pragma once

#include "shape.h"
#include "distribution.h"

int GetTriangleMeshSerializedSize();

struct TriIndex {
    TriIndex() {
    }
    TriIndex(const TriIndexID id0, const TriIndexID id1, const TriIndexID id2) {
        index[0] = id0;
        index[1] = id1;
        index[2] = id2;
    }
    TriIndexID index[3];
};

struct TriMeshData {
    std::vector<Vector3> position0;
    std::vector<Vector3> position1;
    std::vector<Vector3> normal0;
    std::vector<Vector3> normal1;
    std::vector<Vector2> st;
    std::vector<Vector3> colors;
    std::vector<TriIndex> indices;
    bool isMoving;
};

struct TriangleMesh : public Shape {
    TriangleMesh(const std::shared_ptr<const BSDF> bsdf, const std::shared_ptr<TriMeshData> data);
    ShapeType GetType() const override {
        return ShapeType::TriangleMesh;
    }
    ShapeID RtcRegister(const RTCScene &scene, const RTCDevice &device) const override;
    void Serialize(const PrimID primID, Float *buffer) const override;
    bool Intersect(const PrimID &primID,
                   const Float time,
                   const RaySegment &raySeg,
                   Intersection &isect,
                   Vector2 &st) const override;
    Vector2 GetSampleParam(const PrimID &primID,
                           const Vector3 &position,
                           const Float time) const override;
    void SetAreaLight(const AreaLight *areaLight) override;
    PrimID Sample(const Float u) const override;
    void Sample(const Vector2 rndParam,
                const Float time,
                const PrimID primID,
                Vector3 &position,
                Vector3 &normal,
                Float *pdf) const override;
    Float SamplePdf() const override {
        return inverse(totalArea);
    }
    BBox GetBBox() const override {
        return bbox;
    }
    bool IsMoving() const override {
        return data->isMoving;
    }

    const std::shared_ptr<const TriMeshData> data;
    const BBox bbox;
    // Only used when the mesh is associated with an area light
    Float totalArea;
    std::unique_ptr<PiecewiseConstant1D> areaDist;
};

void IntersectTriangleMesh(const ADFloat *buffer,
                           const ADRay &ray,
                           const ADFloat time,
                           const bool isStatic,
                           ADIntersection &isect,
                           ADVector2 &st);

void SampleTriangleMesh(const ADFloat *buffer,
                        const ADVector2 rndParam,
                        const ADFloat time,
                        const bool isStatic,
                        ADVector3 &position,
                        ADVector3 &normal,
                        ADFloat &pdf);

ADFloat SampleTriangleMeshPdf(const ADFloat *buffer);
