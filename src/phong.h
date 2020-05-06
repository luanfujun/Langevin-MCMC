#pragma once

#include "commondef.h"
#include "bsdf.h"
#include "texture.h"

int GetPhongSerializedSize();

struct Phong : public BSDF {
    Phong(const bool twoSided,
          const std::shared_ptr<const TextureRGB> &Kd,
          const std::shared_ptr<const TextureRGB> &Ks,
          const std::shared_ptr<const Texture1D> &exponent)
        : twoSided(twoSided), Kd(Kd), Ks(Ks), exponent(exponent), KsWeight(GetKsWeight()) {
    }

    BSDFType GetType() const override {
        return BSDFType::Phong;
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

    Float GetKsWeight() const;

    Float Roughness(const Vector2 st, const Float uDiscrete) const override;

    const bool twoSided;
    const std::shared_ptr<const TextureRGB> Kd;
    const std::shared_ptr<const TextureRGB> Ks;
    const std::shared_ptr<const Texture1D> exponent;
    const Float KsWeight;
};

void EvaluatePhong(const bool adjoint,
                   const ADFloat *buffer,
                   const ADVector3 &wi,
                   const ADVector3 &normal,
                   const ADVector3 &wo,
                   const ADVector2 st,
                   ADVector3 &contrib,
                   ADFloat &cosWo,
                   ADFloat &pdf,
                   ADFloat &revPdf);


void SamplePhong(const bool adjoint,
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
