#pragma once

#include "commondef.h"
#include "bsdf.h"
#include "texture.h"

int GetLambertianSerializedSize();

struct Lambertian : public BSDF {
    Lambertian(const bool twoSided, const std::shared_ptr<const TextureRGB> &Kd)
        : twoSided(twoSided), Kd(Kd) {
    }

    BSDFType GetType() const override {
        return BSDFType::Lambertian;
    }
    void Serialize(const Vector2 st, Float *buffer) const override;
    void Evaluate(const Vector3 &wi,
                  const Vector3 &normal,
                  const Vector3 &wo,
                  const Vector2 st,
                  Vector3 &contrib,
                  Float &cosWo,
                  Float &pdf,
                  Float &revPdf) const override;
    bool Sample(const Vector3 &wi,
                const Vector3 &normal,
                const Vector2 st,
                const Vector2 rndParam,
                const Float uDiscrete,
                Vector3 &wo,
                Vector3 &contrib,
                Float &cosWo,
                Float &pdf,
                Float &revPdf) const override;

    Float Roughness(const Vector2 st, const Float uDiscrete) const override {
        return Float(1.0);
    }

    const bool twoSided;
    const std::shared_ptr<const TextureRGB> Kd;
};

void EvaluateLambertian(const bool adjoint,
                        const ADFloat *buffer,
                        const ADVector3 &wi,
                        const ADVector3 &normal,
                        const ADVector3 &wo,
                        const ADVector2 st,
                        ADVector3 &contrib,
                        ADFloat &cosWo,
                        ADFloat &pdf,
                        ADFloat &revPdf);

void SampleLambertian(const bool adjoint,
                      const ADFloat *buffer,
                      const ADVector3 &wi,
                      const ADVector3 &normal,
                      const ADVector2 st,
                      const ADVector2 rndParam,
                      const ADFloat uDiscrete,
                      const bool fixDiscrete,
                      ADVector3 &wo,
                      ADVector3 &contrib,
                      ADFloat &cosWo,
                      ADFloat &pdf,
                      ADFloat &revPdf);
