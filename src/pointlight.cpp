#include "pointlight.h"
#include "utils.h"
#include "sampling.h"

int GetPointLightSerializedSize() {
    return 1 +  // type
           3 +  // position
           3;   // emission
}

PointLight::PointLight(const Float &samplingWeight, const Vector3 &pos, const Vector3 &emission)
    : Light(samplingWeight), lightPos(pos), emission(emission) {
}

void PointLight::Serialize(const LightPrimID &lPrimID, Float *buffer) const {
    buffer = ::Serialize((Float)LightType::PointLight, buffer);
    buffer = ::Serialize(lightPos, buffer);
    ::Serialize(emission, buffer);
}

template <typename FloatType>
void _SampleDirectPointLight(const TVector3<FloatType> &lightPos,
                             const TVector3<FloatType> &emission,
                             const TVector3<FloatType> &pos,
                             FloatType &dist,
                             TVector3<FloatType> &dirToLight,
                             TVector3<FloatType> &lightContrib,
                             FloatType &pdf) {
    dirToLight = lightPos - pos;
    const FloatType distSq = LengthSquared(dirToLight);
    pdf = distSq;
    dist = sqrt(distSq);
    dirToLight = dirToLight / dist;
    lightContrib = emission * inverse(distSq);
}

bool PointLight::SampleDirect(const BSphere & /*sceneSphere*/,
                              const Vector3 &pos,
                              const Vector3 & /*normal*/,
                              const Vector2 /*rndParam*/,
                              const Float /*time*/,
                              LightPrimID &lPrimID,
                              Vector3 &dirToLight,
                              Float &dist,
                              Vector3 &contrib,
                              Float &cosAtLight,
                              Float &directPdf,
                              Float &emissionPdf) const {
    _SampleDirectPointLight(lightPos, emission, pos, dist, dirToLight, contrib, directPdf);
    assert(dist > Float(0.0));
    emissionPdf = c_INVFOURPI;
    cosAtLight = Float(1.0);
    lPrimID = 0;
    return true;
}

void PointLight::Emit(const BSphere & /*sceneSphere*/,
                      const Vector2 /*rndParamPos*/,
                      const Vector2 rndParamDir,
                      const Float /*time*/,
                      LightPrimID & /*lPrimID*/,
                      Ray &ray,
                      Vector3 &emission,
                      Float &cosAtLight,
                      Float &emissionPdf,
                      Float &directPdf) const {
    ray.org = this->lightPos;
    ray.dir = SampleSphere(rndParamDir);
    emission = this->emission;
    emissionPdf = c_INVFOURPI;
    cosAtLight = directPdf = Float(1.0);
}

void SampleDirectPointLight(const ADFloat *buffer,
                            const ADBSphere & /*sceneSphere*/,
                            const ADVector3 &pos,
                            const ADVector3 & /*normal*/,
                            const ADVector2 /*rndParam*/,
                            const ADFloat /*time*/,
                            const bool /*isStatic*/,
                            ADVector3 &dirToLight,
                            ADVector3 &lightContrib,
                            ADFloat &cosAtLight,
                            ADFloat &directPdf,
                            ADFloat &emissionPdf) {
    ADVector3 lightPos, emission;
    buffer = Deserialize(buffer, lightPos);
    Deserialize(buffer, emission);
    ADFloat dist;
    _SampleDirectPointLight(lightPos, emission, pos, dist, dirToLight, lightContrib, directPdf);
    emissionPdf = Const<ADFloat>(c_INVFOURPI);
    cosAtLight = Const<ADFloat>(1.0);
}

void EmitPointLight(const ADFloat *buffer,
                    const ADBSphere &sceneSphere,
                    const ADVector2 rndParamPos,
                    const ADVector2 rndParamDir,
                    const ADFloat time,
                    const bool /*isStatic*/,
                    ADRay &ray,
                    ADVector3 &emission,
                    ADFloat &cosAtLight,
                    ADFloat &emissionPdf,
                    ADFloat &directPdf) {
    ADVector3 lightPos, emission_;
    buffer = Deserialize(buffer, lightPos);
    Deserialize(buffer, emission_);

    ray.org = lightPos;
    ray.dir = SampleSphere(rndParamDir);
    emission = emission_;
    emissionPdf = Const<ADFloat>(c_INVFOURPI);
    cosAtLight = Const<ADFloat>(1.0);
    directPdf = Const<ADFloat>(1.0);
}
