#include "trianglemesh.h"

int GetTriangleMeshSerializedSize() {
    return 1 +                    // type
           1 +                    // isMoving
           2 * (3 * 3 + 3 * 3) +  // (3 normal + 1 pos + 2 edges) * 2 time
           1 +                    // hasST
           3 * 2 +                // 3 st
           1;                     // invTotalArea
}

BBox ComputeBBox(const std::shared_ptr<TriMeshData> data) {
    BBox bbox;
    for (const auto &p : data->position0) {
        bbox = Grow(bbox, p);
    }
    if (data->isMoving) {
        for (const auto &p : data->position1) {
            bbox = Grow(bbox, p);
        }
    }
    return bbox;
}

TriangleMesh::TriangleMesh(const std::shared_ptr<const BSDF> bsdf,
                           const std::shared_ptr<TriMeshData> data)
    : Shape(bsdf), data(data), bbox(ComputeBBox(data)) {
}

static inline bool TriangleIntersect(const RaySegment &raySeg,
                                     const Vector3 &p0,
                                     const Vector3 &e1,
                                     const Vector3 &e2,
                                     Vector3 &geomNormal,
                                     Vector2 &uv,
                                     Float &t) {
    const Ray &ray = raySeg.ray;
    geomNormal = Normalize(Cross(e1, e2));
    Vector3 s1 = Cross(ray.dir, e2);
    Float divisor = Dot(s1, e1);
    if (divisor == Float(0.0)) {
        return false;
    }
    Float invDivisor = inverse(divisor);
    Vector3 s = ray.org - p0;
    uv[0] = Dot(s, s1) * invDivisor;
    Vector3 s2 = Cross(s, e1);
    uv[1] = Dot(ray.dir, s2) * invDivisor;

    if (uv[1] >= Float(0.0) && uv[0] + uv[1] <= Float(1.0)) {
        t = Dot(e2, s2) * invDivisor;
        return true;
    }

    return false;
}

bool TriangleIntersect(const RaySegment &raySeg,
                       const Vector3 &p0,
                       const Vector3 &e1,
                       const Vector3 &e2,
                       const Vector3 &n0,
                       const Vector3 &n1,
                       const Vector3 &n2,
                       Intersection &isect,
                       Vector2 &uv) {
    Float t;
    if (!TriangleIntersect(raySeg, p0, e1, e2, isect.geomNormal, uv, t)) {
        return false;
    }
    const Ray &ray = raySeg.ray;
    Float w = Float(1.0) - uv[0] - uv[1];
    isect.position = ray.org + t * ray.dir;
    isect.shadingNormal = Normalize(Vector3(w * n0 + uv[0] * n1 + uv[1] * n2));
    if (Dot(isect.geomNormal, isect.shadingNormal) < Float(0.0)) {
        isect.geomNormal = -isect.geomNormal;
    }
    return true;
}

template <typename FloatType>
void TriangleIntersect(const TRay<FloatType> &ray,
                       const TVector3<FloatType> &p0,
                       const TVector3<FloatType> &e1,
                       const TVector3<FloatType> &e2,
                       const TVector3<FloatType> &n0,
                       const TVector3<FloatType> &n1,
                       const TVector3<FloatType> &n2,
                       TIntersection<FloatType> &isect,
                       TVector2<FloatType> &uv) {
    isect.geomNormal = Normalize(Cross(e1, e2));
    TVector3<FloatType> s1 = Cross(ray.dir, e2);
    FloatType divisor = Dot(s1, e1);
    FloatType invDivisor = inverse(divisor);

    TVector3<FloatType> s = ray.org - p0;
    uv[0] = Dot(s, s1) * invDivisor;

    TVector3<FloatType> s2 = Cross(s, e1);
    uv[1] = Dot(ray.dir, s2) * invDivisor;
    FloatType t = Dot(e2, s2) * invDivisor;
    FloatType w = Float(1.0) - uv[0] - uv[1];
    isect.position = ray.org + t * ray.dir;
    isect.shadingNormal = Normalize(TVector3<FloatType>(w * n0 + uv[0] * n1 + uv[1] * n2));
}

ShapeID TriangleMesh::RtcRegister(const RTCScene &rtcScene, const RTCDevice &rtcDevice) const {
    struct Vertex {
        float x, y, z, a;
    };
    struct Index {
        uint32_t v0, v1, v2;
    };
    
    RTCGeometry geom_0 = rtcNewGeometry(rtcDevice, RTC_GEOMETRY_TYPE_TRIANGLE); // EMBREE_FIXME: check if geometry gets properly committed
     rtcSetGeometryBuildQuality(geom_0,RTC_BUILD_QUALITY_MEDIUM);
     rtcSetGeometryTimeStepCount(geom_0,data->isMoving ? 2 : 1);
    ShapeID geomID = rtcAttachGeometry(rtcScene,geom_0);
     rtcReleaseGeometry(geom_0);

    if (!data->isMoving) {
        Vertex *vertices = (Vertex *)rtcSetNewGeometryBuffer(geom_0,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,4*sizeof(float),data->position0.size());
        for (const auto &p : data->position0)
            *vertices++ = Vertex{float(p[0]), float(p[1]), float(p[2]), 0.0f};
        
    } else {
        Vertex *vertices0 = (Vertex *)rtcSetNewGeometryBuffer(geom_0,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,4*sizeof(float),data->position0.size());
        for (const auto &p : data->position0)
            *vertices0++ = Vertex{float(p[0]), float(p[1]), float(p[2]), 0.0f};
        
        Vertex *vertices1 = (Vertex *)rtcSetNewGeometryBuffer(geom_0,RTC_BUFFER_TYPE_VERTEX,1,RTC_FORMAT_FLOAT3,4*sizeof(float),data->position0.size());
        for (const auto &p : data->position1)
            *vertices1++ = Vertex{float(p[0]), float(p[1]), float(p[2]), 0.0f};
        
    }

    Index *indices = (Index *)rtcSetNewGeometryBuffer(geom_0,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,3*sizeof(int),data->indices.size());
    for (const auto &i : data->indices)
        *indices++ = Index{i.index[0], i.index[1], i.index[2]};
    
    rtcCommitGeometry(geom_0);
    return geomID;
}

void TriangleMesh::Serialize(const PrimID primID, Float *buffer) const {
    buffer = ::Serialize((Float)ShapeType::TriangleMesh, buffer);

    buffer = ::Serialize((Float)data->isMoving, buffer);
    assert(primID < int(data->indices.size()));
    const TriIndex &index = data->indices[primID];
    const Vector3 &p0_0 = data->position0[index.index[0]];
    const Vector3 &p1_0 = data->position0[index.index[1]];
    const Vector3 &p2_0 = data->position0[index.index[2]];
    const Vector3 &n0_0 = data->normal0[index.index[0]];
    const Vector3 &n1_0 = data->normal0[index.index[1]];
    const Vector3 &n2_0 = data->normal0[index.index[2]];
    const Vector3 &p0_1 = data->position1[index.index[0]];
    const Vector3 &p1_1 = data->position1[index.index[1]];
    const Vector3 &p2_1 = data->position1[index.index[2]];
    const Vector3 &n0_1 = data->normal1[index.index[0]];
    const Vector3 &n1_1 = data->normal1[index.index[1]];
    const Vector3 &n2_1 = data->normal1[index.index[2]];
    buffer = ::Serialize(p0_0, buffer);
    buffer = ::Serialize(Vector3(p1_0 - p0_0), buffer);
    buffer = ::Serialize(Vector3(p2_0 - p0_0), buffer);
    buffer = ::Serialize(n0_0, buffer);
    buffer = ::Serialize(n1_0, buffer);
    buffer = ::Serialize(n2_0, buffer);
    buffer = ::Serialize(p0_1, buffer);
    buffer = ::Serialize(Vector3(p1_1 - p0_1), buffer);
    buffer = ::Serialize(Vector3(p2_1 - p0_1), buffer);
    buffer = ::Serialize(n0_1, buffer);
    buffer = ::Serialize(n1_1, buffer);
    buffer = ::Serialize(n2_1, buffer);
    buffer = ::Serialize(BoolToFloat(data->st.size() == 0), buffer);
    if (data->st.size() > 0) {
        const Vector2 &uv0 = data->st[index.index[0]];
        const Vector2 &uv1 = data->st[index.index[1]];
        const Vector2 &uv2 = data->st[index.index[2]];
        buffer = ::Serialize(uv0, buffer);
        buffer = ::Serialize(uv1, buffer);
        buffer = ::Serialize(uv2, buffer);
    } else {
        buffer += 6;  // uv values not defined
    }
    ::Serialize(inverse(totalArea), buffer);
}

bool TriangleMesh::Intersect(const PrimID &primID,
                             const Float time,
                             const RaySegment &raySeg,
                             Intersection &isect,
                             Vector2 &st) const {
    const TriIndex &index = data->indices[primID];
    const Vector3 &p0_0 = data->position0[index.index[0]];
    const Vector3 &p1_0 = data->position0[index.index[1]];
    const Vector3 &p2_0 = data->position0[index.index[2]];
    const Vector3 &n0_0 = data->normal0[index.index[0]];
    const Vector3 &n1_0 = data->normal0[index.index[1]];
    const Vector3 &n2_0 = data->normal0[index.index[2]];
    const Vector3 &p0_1 = data->position1[index.index[0]];
    const Vector3 &p1_1 = data->position1[index.index[1]];
    const Vector3 &p2_1 = data->position1[index.index[2]];
    const Vector3 &n0_1 = data->normal1[index.index[0]];
    const Vector3 &n1_1 = data->normal1[index.index[1]];
    const Vector3 &n2_1 = data->normal1[index.index[2]];
    Vector2 uv;
    if (!data->isMoving) {
        if (!TriangleIntersect(
                raySeg, p0_0, p1_0 - p0_0, p2_0 - p0_0, n0_0, n1_0, n2_0, isect, uv)) {
            return false;
        }
    } else {
        const Float oneMinusTime = (Float(1.0) - time);
        const Vector3 p0 = p0_0 * oneMinusTime + p0_1 * time;
        const Vector3 e1 = (p1_0 - p0_0) * oneMinusTime + (p1_1 - p0_1) * time;
        const Vector3 e2 = (p2_0 - p0_0) * oneMinusTime + (p2_1 - p0_1) * time;
        const Vector3 n0 = n0_0 * oneMinusTime + n0_1 * time;
        const Vector3 n1 = n1_0 * oneMinusTime + n1_1 * time;
        const Vector3 n2 = n2_0 * oneMinusTime + n2_1 * time;
        if (!TriangleIntersect(raySeg, p0, e1, e2, n0, n1, n2, isect, uv)) {
            return false;
        }
    }

    if (data->st.size() != 0) {
        const Vector2 &st0 = data->st[index.index[0]];
        const Vector2 &st1 = data->st[index.index[1]];
        const Vector2 &st2 = data->st[index.index[2]];
        st[0] = (Float(1.0) - uv[0] - uv[1]) * st0[0] + uv[0] * st1[0] + uv[1] * st2[0];
        st[1] = (Float(1.0) - uv[0] - uv[1]) * st0[1] + uv[0] * st1[1] + uv[1] * st2[1];
    } else {
        st = uv;
    }
    return true;
}

// http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
Vector2 Barycentric(const Vector3 &position,
                    const Vector3 &p0,
                    const Vector3 &e1,
                    const Vector3 &e2) {
    const Vector3 e0 = position - p0;
    const Float d11 = Dot(e1, e1);
    const Float d12 = Dot(e1, e2);
    const Float d22 = Dot(e2, e2);
    const Float d01 = Dot(e0, e1);
    const Float d02 = Dot(e0, e2);
    const Float invDenom = inverse(d11 * d22 - d12 * d12);
    const Float b1 = (d22 * d01 - d12 * d02) * invDenom;
    const Float b2 = (d11 * d02 - d12 * d01) * invDenom;
    return Vector2(b1, b2);
}

Vector2 TriangleMesh::GetSampleParam(const PrimID &primID,
                                     const Vector3 &position,
                                     const Float time) const {
    const TriIndex &index = data->indices[primID];
    const Vector3 &p0_0 = data->position0[index.index[0]];
    const Vector3 &p1_0 = data->position0[index.index[1]];
    const Vector3 &p2_0 = data->position0[index.index[2]];
    const Vector3 &p0_1 = data->position1[index.index[0]];
    const Vector3 &p1_1 = data->position1[index.index[1]];
    const Vector3 &p2_1 = data->position1[index.index[2]];

    Vector2 b;
    if (!data->isMoving) {
        const Vector3 p0 = p0_0;
        const Vector3 e1 = p1_0 - p0_0;
        const Vector3 e2 = p2_0 - p0_0;

        b = Barycentric(position, p0, e1, e2);
    } else {
        const Float oneMinusTime = Float(1.0) - time;
        const Vector3 p0 = p0_0 * oneMinusTime + p0_1 * time;
        const Vector3 e1 = (p1_0 - p0_0) * oneMinusTime + (p1_1 - p0_1) * time;
        const Vector3 e2 = (p2_0 - p0_0) * oneMinusTime + (p2_1 - p0_1) * time;

        b = Barycentric(position, p0, e1, e2);
    }

    const Float a = Float(1.0) - b[0];
    Vector2 sampleParam;
    // a = sqrt(1 - sampleParam[0]) -> 1 - a^2 = sampleParam[0]
    sampleParam[0] = (Float(1.0) + ADEpsilon<Float>()) - square(a);
    // a * sampleParam[1] = b2
    sampleParam[1] = b[1] / a;
    return sampleParam;
}

void TriangleMesh::SetAreaLight(const AreaLight *areaLight) {
    Shape::SetAreaLight(areaLight);
    std::vector<Float> area(data->indices.size());
    totalArea = Float(0.0);
    // Currently we do not allow light sources that has varying area over time
    for (size_t i = 0; i < area.size(); i++) {
        const Vector3 &p0 = data->position0[data->indices[i].index[0]];
        const Vector3 &p1 = data->position0[data->indices[i].index[1]];
        const Vector3 &p2 = data->position0[data->indices[i].index[2]];
        const Vector3 e1 = p1 - p0;
        const Vector3 e2 = p2 - p0;
        area[i] = Float(0.5) * Length(Cross(e1, e2));
        totalArea += area[i];
    }
    areaDist = std::unique_ptr<PiecewiseConstant1D>(new PiecewiseConstant1D(&area[0], area.size()));
}

PrimID TriangleMesh::Sample(const Float u) const {
    return areaDist->SampleDiscrete(u, nullptr);
}

template <typename FloatType>
void SampleDirect(const TVector3<FloatType> &p0,
                  const TVector3<FloatType> &e1,
                  const TVector3<FloatType> &e2,
                  const TVector3<FloatType> &n0,
                  const TVector3<FloatType> &n1,
                  const TVector3<FloatType> &n2,
                  const TVector2<FloatType> rndParam,
                  TVector3<FloatType> &posOnShape,
                  TVector3<FloatType> &normalOnShape) {
    const FloatType a = sqrt((Float(1.0) + ADEpsilon<FloatType>()) - rndParam[0]);
    const FloatType b1 = Float(1.0) - a;
    const FloatType b2 = a * rndParam[1];
    posOnShape = p0 + (e1 * b1) + (e2 * b2);
    normalOnShape = Normalize(TVector3<FloatType>(n0 * (Float(1.0) - b1 - b2) + n1 * b1 + n2 * b2));
}

void TriangleMesh::Sample(const Vector2 rndParam,
                          const Float time,
                          const PrimID primID,
                          Vector3 &position,
                          Vector3 &normal,
                          Float *pdf) const {
    const TriIndex &index = data->indices[primID];
    const Vector3 &p0_0 = data->position0[index.index[0]];
    const Vector3 &p1_0 = data->position0[index.index[1]];
    const Vector3 &p2_0 = data->position0[index.index[2]];
    const Vector3 &n0_0 = data->normal0[index.index[0]];
    const Vector3 &n1_0 = data->normal0[index.index[1]];
    const Vector3 &n2_0 = data->normal0[index.index[2]];
    const Vector3 &p0_1 = data->position1[index.index[0]];
    const Vector3 &p1_1 = data->position1[index.index[1]];
    const Vector3 &p2_1 = data->position1[index.index[2]];
    const Vector3 &n0_1 = data->normal1[index.index[0]];
    const Vector3 &n1_1 = data->normal1[index.index[1]];
    const Vector3 &n2_1 = data->normal1[index.index[2]];
    if (!data->isMoving) {
        const Vector3 e1_0 = p1_0 - p0_0;
        const Vector3 e2_0 = p2_0 - p0_0;
        SampleDirect(p0_0, e1_0, e2_0, n0_0, n1_0, n2_0, rndParam, position, normal);
    } else {
        const Float oneMinusTime = (Float(1.0) - time);
        const Vector3 p0 = p0_0 * oneMinusTime + p0_1 * time;
        const Vector3 e1 = (p1_0 - p0_0) * oneMinusTime + (p1_1 - p0_1) * time;
        const Vector3 e2 = (p2_0 - p0_0) * oneMinusTime + (p2_1 - p0_1) * time;
        const Vector3 n0 = n0_0 * oneMinusTime + n0_1 * time;
        const Vector3 n1 = n1_0 * oneMinusTime + n1_1 * time;
        const Vector3 n2 = n2_0 * oneMinusTime + n2_1 * time;
        SampleDirect(p0, e1, e2, n0, n1, n2, rndParam, position, normal);
    }
    if (pdf != nullptr) {
        *pdf = inverse(totalArea);
    }
}

void IntersectTriangleMesh(const ADFloat *buffer,
                           const ADRay &ray,
                           const ADFloat time,
                           const bool isStatic,
                           ADIntersection &isect,
                           ADVector2 &st) {
    ADFloat isMoving;
    buffer = Deserialize(buffer, isMoving);

    ADVector3 p0_0, e1_0, e2_0;
    ADVector3 n0_0, n1_0, n2_0;
    ADVector3 p0_1, e1_1, e2_1;
    ADVector3 n0_1, n1_1, n2_1;
    buffer = Deserialize(buffer, p0_0);
    buffer = Deserialize(buffer, e1_0);
    buffer = Deserialize(buffer, e2_0);
    buffer = Deserialize(buffer, n0_0);
    buffer = Deserialize(buffer, n1_0);
    buffer = Deserialize(buffer, n2_0);
    buffer = Deserialize(buffer, p0_1);
    buffer = Deserialize(buffer, e1_1);
    buffer = Deserialize(buffer, e2_1);
    buffer = Deserialize(buffer, n0_1);
    buffer = Deserialize(buffer, n1_1);
    buffer = Deserialize(buffer, n2_1);

    ADFloat hasST;
    buffer = Deserialize(buffer, hasST);
    ADVector2 st0, st1, st2;
    buffer = Deserialize(buffer, st0);
    buffer = Deserialize(buffer, st1);
    buffer = Deserialize(buffer, st2);

    ADFloat invTotalArea;
    buffer = Deserialize(buffer, invTotalArea);

    ADVector2 uv;
    if (isStatic) {
        TriangleIntersect(ray, p0_0, e1_0, e2_0, n0_0, n1_0, n2_0, isect, uv);
    } else {
        std::vector<CondExprCPtr> ret = CreateCondExprVec(11);
        BeginIf(Eq(isMoving, FFALSE), ret);
        {
            ADIntersection isect;
            TriangleIntersect(ray, p0_0, e1_0, e2_0, n0_0, n1_0, n2_0, isect, uv);
            SetCondOutput({isect.position[0],
                           isect.position[1],
                           isect.position[2],
                           isect.geomNormal[0],
                           isect.geomNormal[1],
                           isect.geomNormal[2],
                           isect.shadingNormal[0],
                           isect.shadingNormal[1],
                           isect.shadingNormal[2],
                           uv[0],
                           uv[1]});
        }
        BeginElse();
        {
            ADFloat oneMinusTime = (Float(1.0) - time);
            ADVector3 p0 = p0_0 * oneMinusTime + p0_1 * time;
            ADVector3 e1 = e1_0 * oneMinusTime + e1_1 * time;
            ADVector3 e2 = e2_0 * oneMinusTime + e2_1 * time;
            ADVector3 n0 = n0_0 * oneMinusTime + n0_1 * time;
            ADVector3 n1 = n1_0 * oneMinusTime + n1_1 * time;
            ADVector3 n2 = n2_0 * oneMinusTime + n2_1 * time;
            ADIntersection isect;
            ADVector2 uv;
            TriangleIntersect(ray, p0, e1, e2, n0, n1, n2, isect, uv);
            SetCondOutput({isect.position[0],
                           isect.position[1],
                           isect.position[2],
                           isect.geomNormal[0],
                           isect.geomNormal[1],
                           isect.geomNormal[2],
                           isect.shadingNormal[0],
                           isect.shadingNormal[1],
                           isect.shadingNormal[2],
                           uv[0],
                           uv[1]});
        }
        EndIf();
        isect.position[0] = ret[0];
        isect.position[1] = ret[1];
        isect.position[2] = ret[2];
        isect.geomNormal[0] = ret[3];
        isect.geomNormal[1] = ret[4];
        isect.geomNormal[2] = ret[5];
        isect.shadingNormal[0] = ret[6];
        isect.shadingNormal[1] = ret[7];
        isect.shadingNormal[2] = ret[8];
        uv[0] = ret[9];
        uv[1] = ret[10];
    }
    std::vector<CondExprCPtr> vst = CreateCondExprVec(2);
    BeginIf(Eq(hasST, FFALSE), vst);
    { SetCondOutput({uv[0], uv[1]}); }
    BeginElse();
    {
        ADFloat s = (Float(1.0) - uv[0] - uv[1]) * st0[0] + uv[0] * st1[0] + uv[1] * st2[0];
        ADFloat t = (Float(1.0) - uv[0] - uv[1]) * st0[1] + uv[0] * st1[1] + uv[1] * st2[1];
        SetCondOutput({s, t});
    }
    EndIf();
    st[0] = vst[0];
    st[1] = vst[1];
}

void SampleTriangleMesh(const ADFloat *buffer,
                        const ADVector2 rndParam,
                        const ADFloat time,
                        const bool isStatic,
                        ADVector3 &position,
                        ADVector3 &normal,
                        ADFloat &pdf) {
    ADFloat isMoving;
    buffer = Deserialize(buffer, isMoving);

    ADVector3 p0_0, e1_0, e2_0;
    ADVector3 n0_0, n1_0, n2_0;
    ADVector3 p0_1, e1_1, e2_1;
    ADVector3 n0_1, n1_1, n2_1;
    buffer = Deserialize(buffer, p0_0);
    buffer = Deserialize(buffer, e1_0);
    buffer = Deserialize(buffer, e2_0);
    buffer = Deserialize(buffer, n0_0);
    buffer = Deserialize(buffer, n1_0);
    buffer = Deserialize(buffer, n2_0);
    buffer = Deserialize(buffer, p0_1);
    buffer = Deserialize(buffer, e1_1);
    buffer = Deserialize(buffer, e2_1);
    buffer = Deserialize(buffer, n0_1);
    buffer = Deserialize(buffer, n1_1);
    buffer = Deserialize(buffer, n2_1);

    ADFloat hasST;
    buffer = Deserialize(buffer, hasST);
    ADVector2 st0, st1, st2;
    buffer = Deserialize(buffer, st0);
    buffer = Deserialize(buffer, st1);
    buffer = Deserialize(buffer, st2);
    ADFloat invTotalArea;
    buffer = Deserialize(buffer, invTotalArea);

    if (isStatic) {
        SampleDirect(p0_0, e1_0, e2_0, n0_0, n1_0, n2_0, rndParam, position, normal);
    } else {
        std::vector<CondExprCPtr> ret = CreateCondExprVec(6);
        BeginIf(Eq(isMoving, FFALSE), ret);
        {
            ADVector3 position;
            ADVector3 normal;
            SampleDirect(p0_0, e1_0, e2_0, n0_0, n1_0, n2_0, rndParam, position, normal);
            SetCondOutput({position[0], position[1], position[2], normal[0], normal[1], normal[2]});
        }
        BeginElse();
        {
            ADVector3 position;
            ADVector3 normal;
            ADFloat oneMinusTime = (Float(1.0) - time);
            ADVector3 p0 = p0_0 * oneMinusTime + p0_1 * time;
            ADVector3 e1 = e1_0 * oneMinusTime + e1_1 * time;
            ADVector3 e2 = e2_0 * oneMinusTime + e2_1 * time;
            ADVector3 n0 = n0_0 * oneMinusTime + n0_1 * time;
            ADVector3 n1 = n1_0 * oneMinusTime + n1_1 * time;
            ADVector3 n2 = n2_0 * oneMinusTime + n2_1 * time;
            SampleDirect(p0, e1, e2, n0, n1, n2, rndParam, position, normal);
            SetCondOutput({position[0], position[1], position[2], normal[0], normal[1], normal[2]});
        }
        EndIf();
        position[0] = ret[0];
        position[1] = ret[1];
        position[2] = ret[2];
        normal[0] = ret[3];
        normal[1] = ret[4];
        normal[2] = ret[5];
    }
    pdf = invTotalArea;
}

ADFloat SampleTriangleMeshPdf(const ADFloat *buffer) {
    ADFloat isMoving;
    buffer = Deserialize(buffer, isMoving);

    ADVector3 p0_0, e1_0, e2_0;
    ADVector3 n0_0, n1_0, n2_0;
    ADVector3 p0_1, e1_1, e2_1;
    ADVector3 n0_1, n1_1, n2_1;
    buffer = Deserialize(buffer, p0_0);
    buffer = Deserialize(buffer, e1_0);
    buffer = Deserialize(buffer, e2_0);
    buffer = Deserialize(buffer, n0_0);
    buffer = Deserialize(buffer, n1_0);
    buffer = Deserialize(buffer, n2_0);
    buffer = Deserialize(buffer, p0_1);
    buffer = Deserialize(buffer, e1_1);
    buffer = Deserialize(buffer, e2_1);
    buffer = Deserialize(buffer, n0_1);
    buffer = Deserialize(buffer, n1_1);
    buffer = Deserialize(buffer, n2_1);

    ADFloat hasST;
    buffer = Deserialize(buffer, hasST);
    ADVector2 uv0, uv1, uv2;
    buffer = Deserialize(buffer, uv0);
    buffer = Deserialize(buffer, uv1);
    buffer = Deserialize(buffer, uv2);
    ADFloat invTotalArea;
    buffer = Deserialize(buffer, invTotalArea);

    return invTotalArea;
}
