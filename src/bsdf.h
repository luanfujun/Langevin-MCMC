#pragma once

#include "commondef.h"
#include <string>

enum class BSDFType { Lambertian, Phong, RoughDielectric };

int GetMaxBSDFSerializedSize();

struct BSDF {
    virtual BSDFType GetType() const = 0;
    virtual void Serialize(const Vector2 st, Float *buffer) const = 0;
    virtual void Evaluate(const Vector3 &wi,
                          const Vector3 &normal,
                          const Vector3 &wo,
                          const Vector2 st,
                          Vector3 &contrib,
                          Float &cosWo,
                          Float &pdf,
                          Float &revPdf) const = 0;
    virtual void EvaluateAdjoint(const Vector3 &wi,
                                 const Vector3 &normal,
                                 const Vector3 &wo,
                                 const Vector2 st,
                                 Vector3 &contrib,
                                 Float &cosWo,
                                 Float &pdf,
                                 Float &revPdf) const {
        Evaluate(wi, normal, wo, st, contrib, cosWo, pdf, revPdf);
    }
    virtual bool Sample(const Vector3 &wi,
                        const Vector3 &normal,
                        const Vector2 st,
                        const Vector2 rndParam,
                        const Float uDiscrete,
                        Vector3 &wo,
                        Vector3 &contrib,
                        Float &cosWo,
                        Float &pdf,
                        Float &revPdf) const = 0;
    virtual bool SampleAdjoint(const Vector3 &wi,
                               const Vector3 &normal,
                               const Vector2 st,
                               const Vector2 rndParam,
                               const Float uDiscrete,
                               Vector3 &wo,
                               Vector3 &contrib,
                               Float &cosWo,
                               Float &pdf,
                               Float &revPdf) const {
        return Sample(wi, normal, st, rndParam, uDiscrete, wo, contrib, cosWo, pdf, revPdf);
    }

    virtual Float Roughness(const Vector2 st, const Float uDiscrete) const = 0;

    virtual std::string GetBSDFTypeString() const {
        std::string ret;
        if (this->GetType() == BSDFType::Lambertian) 
            ret = "diffuse";
        else if (this->GetType() == BSDFType::RoughDielectric)
            ret = "glass";
        else if (this->GetType() == BSDFType::Phong)
            ret = "glossy";
        else 
            ret = "unknown";
        return ret;
    }
};

const ADFloat *EvaluateBSDF(const bool adjoint,
                            const ADFloat *buffer,
                            const ADVector3 &wi,
                            const ADVector3 &normal,
                            const ADVector3 &wo,
                            const ADVector2 st,
                            ADVector3 &contrib,
                            ADFloat &cosWo,
                            ADFloat &pdf,
                            ADFloat &revPdf);

const ADFloat *SampleBSDF(const bool adjoint,
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
