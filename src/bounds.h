#pragma once

#include "commondef.h"
#include "utils.h"

struct BBox {
    BBox() = default;
    BBox(const Vector3 &pMin, const Vector3 &pMax) : pMin(pMin), pMax(pMax) {
    }

    Vector3 pMin = Vector3(std::numeric_limits<Float>::infinity(),
                           std::numeric_limits<Float>::infinity(),
                           std::numeric_limits<Float>::infinity());
    Vector3 pMax = Vector3(-std::numeric_limits<Float>::infinity(),
                           -std::numeric_limits<Float>::infinity(),
                           -std::numeric_limits<Float>::infinity());
};

inline BBox Grow(const BBox &bbox, const Vector3 &p) {
    return BBox{Vector3(bbox.pMin.cwiseMin(p)), Vector3(bbox.pMax.cwiseMax(p))};
}

inline BBox Merge(const BBox &bbox0, const BBox &bbox1) {
    return BBox{Vector3(bbox0.pMin.cwiseMin(bbox1.pMin)), Vector3(bbox0.pMax.cwiseMax(bbox1.pMax))};
}

struct BSphere {
    BSphere() {
    }
    BSphere(const BBox &bbox) {
        center = Float(0.5) * (bbox.pMin + bbox.pMax);
        radius = Float(0.5) * Distance(bbox.pMin, bbox.pMax);
    }

    Vector3 center;
    Float radius;
};

struct ADBSphere {
    ADVector3 center;
    ADFloat radius;
};

inline int GetBSphereSerializedSize() {
    return 4;  // center & radius
}

inline Float *Serialize(const BSphere &bSphere, Float *buffer) {
    buffer = Serialize(bSphere.center, buffer);
    buffer = Serialize(bSphere.radius, buffer);
    return buffer;
}

template <typename FloatType>
const FloatType *Deserialize(const FloatType *buffer, ADBSphere &bSphere) {
    buffer = Deserialize(buffer, bSphere.center);
    buffer = Deserialize(buffer, bSphere.radius);
    return buffer;
}
