#include "arealight.h"
#include "shape.h"
#include "utils.h"
#include "sampling.h"

int GetAreaLightSerializedSize() {
    return 1 +                            // type
           GetMaxShapeSerializedSize() +  // shape
           3;                             // emission
}

AreaLight::AreaLight(const Float &samplingWeight, Shape *_shape, const Vector3 &emission)
    : Light(samplingWeight), shape(_shape), emission(emission) {
    _shape->SetAreaLight(this);
}

void AreaLight::Serialize(const LightPrimID &lPrimID, Float *buffer) const {
    buffer = ::Serialize((Float)LightType::AreaLight, buffer);
    shape->Serialize(lPrimID, buffer);
    buffer += GetMaxShapeSerializedSize();
    ::Serialize(emission, buffer);
}

LightPrimID AreaLight::SampleDiscrete(const Float uDiscrete) const {
    return shape->Sample(uDiscrete);
}

bool AreaLight::SampleDirect(const BSphere & /*sceneSphere*/,
                             const Vector3 &pos,
                             const Vector3 &normal,
                             const Vector2 rndParam,
                             const Float time,
                             LightPrimID &lPrimID,
                             Vector3 &dirToLight,
                             Float &dist,
                             Vector3 &contrib,
                             Float &cosAtLight,
                             Float &directPdf,
                             Float &emissionPdf) const {
    Vector3 posOnLight;
    Vector3 normalOnLight;
    Float shapePdf;
    shape->Sample(rndParam, time, lPrimID, posOnLight, normalOnLight, &shapePdf);
    dirToLight = posOnLight - pos;
    Float distSq = LengthSquared(dirToLight);
    dist = sqrt(distSq);
    assert(dist > Float(0.0));
    dirToLight = dirToLight / dist;
    cosAtLight = -Dot(dirToLight, normalOnLight);
    if (cosAtLight > c_CosEpsilon) {  // ignore grazing angle
        contrib = (cosAtLight / (distSq * shapePdf)) * emission;
        directPdf = shapePdf * distSq / cosAtLight;
        emissionPdf = shapePdf * cosAtLight * c_INVPI;
        return true;
    } else {
        return false;
    }
}

void AreaLight::Emission(const BSphere & /*sceneSphere*/,
                         const Vector3 &dirToLight,
                         const Vector3 &normalOnLight,
                         const Float time,
                         LightPrimID &lPrimID,
                         Vector3 &emission,
                         Float &directPdf,
                         Float &emissionPdf) const {
    Float cosAtLight = -Dot(normalOnLight, dirToLight);
    if (cosAtLight > Float(0.0)) {
        emission = this->emission;
        directPdf = shape->SamplePdf();
        emissionPdf = cosAtLight * directPdf * c_INVPI;
    } else {
        emission = Vector3::Zero();
        directPdf = Float(0.0);
        emissionPdf = Float(0.0);
    }
}

void AreaLight::Emit(const BSphere & /*sceneSphere*/,
                     const Vector2 rndParamPos,
                     const Vector2 rndParamDir,
                     const Float time,
                     LightPrimID &lPrimID,
                     Ray &ray,
                     Vector3 &emission,
                     Float &cosAtLight,
                     Float &emissionPdf,
                     Float &directPdf) const {
    Vector3 normal;
    Float shapePdf;
    shape->Sample(rndParamPos, time, lPrimID, ray.org, normal, &shapePdf);
    assert(shapePdf > Float(0.0));

    Vector3 d = SampleCosHemisphere(rndParamDir);
    Vector3 b0;
    Vector3 b1;
    CoordinateSystem(normal, b0, b1);
    ray.dir = d[0] * b0 + d[1] * b1 + d[2] * normal;
    emission = this->emission * (Float(M_PI) / shapePdf);
    cosAtLight = d[2];
    emissionPdf = d[2] * c_INVPI * shapePdf;
    directPdf = shapePdf;
}

template <typename FloatType>
void _SampleDirectAreaLight(const FloatType *buffer,
                            const TVector3<FloatType> &pos,
                            const TVector3<FloatType> &normal,
                            const TVector2<FloatType> rndParam,
                            const FloatType time,
                            const bool isStatic,
                            TVector3<FloatType> &dirToLight,
                            TVector3<FloatType> &lightContrib,
                            FloatType &cosAtLight,
                            FloatType &directPdf,
                            FloatType &emissionPdf) {
    TVector3<FloatType> posOnLight;
    TVector3<FloatType> normalOnLight;
    FloatType shapePdf;
    SampleShape(buffer, rndParam, time, isStatic, posOnLight, normalOnLight, shapePdf);
    buffer += GetMaxShapeSerializedSize();
    TVector3<FloatType> emission;
    Deserialize(buffer, emission);

    dirToLight = posOnLight - pos;
    FloatType distSq = LengthSquared(dirToLight);
    FloatType dist = sqrt(distSq);
    dirToLight = dirToLight / dist;
    cosAtLight = -Dot(dirToLight, normalOnLight);
    directPdf = shapePdf * distSq / cosAtLight;
    lightContrib = emission / directPdf;
    emissionPdf = shapePdf * cosAtLight * c_INVPI;
}

void SampleDirectAreaLight(const ADFloat *buffer,
                           const ADBSphere & /*sceneSphere*/,
                           const ADVector3 &pos,
                           const ADVector3 &normal,
                           const ADVector2 rndParam,
                           const ADFloat time,
                           const bool isStatic,
                           ADVector3 &dirToLight,
                           ADVector3 &lightContrib,
                           ADFloat &cosAtLight,
                           ADFloat &directPdf,
                           ADFloat &emissionPdf) {
    _SampleDirectAreaLight(buffer,
                           pos,
                           normal,
                           rndParam,
                           time,
                           isStatic,
                           dirToLight,
                           lightContrib,
                           cosAtLight,
                           directPdf,
                           emissionPdf);
}

void EmissionAreaLight(const ADFloat *buffer,
                       const ADBSphere &sceneSphere,
                       const ADVector3 &dirToLight,
                       const ADVector3 &normalOnLight,
                       const ADFloat time,
                       ADVector3 &emission,
                       ADFloat &directPdf,
                       ADFloat &emissionPdf) {
    ADFloat shapePdf;
    buffer = SampleShapePdf(buffer, shapePdf);
    ADVector3 emission_;
    Deserialize(buffer, emission_);
    ADFloat cosAtLight = -Dot(normalOnLight, dirToLight);
    // Assume cosAtLight > 0
    emission = emission_;
    directPdf = shapePdf;
    emissionPdf = cosAtLight * directPdf * c_INVPI;
}

void EmitAreaLight(const ADFloat *buffer,
                   const ADBSphere &sceneSphere,
                   const ADVector2 rndParamPos,
                   const ADVector2 rndParamDir,
                   const ADFloat time,
                   const bool isStatic,
                   ADRay &ray,
                   ADVector3 &emission,
                   ADFloat &cosAtLight,
                   ADFloat &emissionPdf,
                   ADFloat &directPdf) {
    ADVector3 normalOnLight;
    ADFloat shapePdf;
    SampleShape(buffer, rndParamPos, time, isStatic, ray.org, normalOnLight, shapePdf);
    buffer += GetMaxShapeSerializedSize();
    ADVector3 emission_;
    Deserialize(buffer, emission_);

    ADVector3 d = SampleCosHemisphere(rndParamDir);
    ADVector3 b0;
    ADVector3 b1;
    CoordinateSystem(normalOnLight, b0, b1);

    ray.dir = d[0] * b0 + d[1] * b1 + d[2] * normalOnLight;
    emission = emission_ * (Float(M_PI) / shapePdf);
    cosAtLight = d[2];
    emissionPdf = d[2] * c_INVPI * shapePdf;
    directPdf = shapePdf;
}
