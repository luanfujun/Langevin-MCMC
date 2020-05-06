#include "lambertian.h"
#include "utils.h"
#include "sampling.h"

int GetLambertianSerializedSize() {
    return 1 +  // type
           3;   // Kd
}

void Lambertian::Serialize(const Vector2 st, Float *buffer) const {
    buffer = ::Serialize((Float)BSDFType::Lambertian, buffer);
    ::Serialize(Kd->Eval(st), buffer);
}

void Lambertian::Evaluate(const Vector3 &wi,
                          const Vector3 &normal,
                          const Vector3 &wo,
                          const Vector2 st,
                          Vector3 &contrib,
                          Float &cosWo,
                          Float &pdf,
                          Float &revPdf) const {
    Float cosWi = Dot(normal, wi);
    Vector3 normal_ = normal;
    if (twoSided && cosWi < Float(0.0)) {
        cosWi = -cosWi;
        normal_ = -normal_;
    }
    cosWo = Dot(normal_, wo);
    contrib.setZero();
    if (cosWi < c_CosEpsilon || cosWo < c_CosEpsilon) {
        return;
    }

    Float fwdScalar = cosWo * c_INVPI;
    Float revScalar = cosWi * c_INVPI;
    contrib = fwdScalar * Kd->Eval(st);
    pdf = fwdScalar;
    revPdf = revScalar;
}


template <typename FloatType>
void Sample(const TVector3<FloatType> &normal,
            const TVector2<FloatType> rndParam,
            TVector3<FloatType> &wo,
            FloatType &cosWo,
            FloatType &pdf) 
{
    TVector3<FloatType> b0;
    TVector3<FloatType> b1;
    CoordinateSystem(normal, b0, b1);
    TVector3<FloatType> ret = SampleCosHemisphere(rndParam);
    wo = ret[0] * b0 + ret[1] * b1 + ret[2] * normal;
    cosWo = ret[2];
    pdf = ret[2] * c_INVPI;
}

bool Lambertian::Sample(const Vector3 &wi,
                        const Vector3 &normal,
                        const Vector2 st,
                        const Vector2 rndParam,
                        const Float /*uDiscrete*/,
                        Vector3 &wo,
                        Vector3 &contrib,
                        Float &cosWo,
                        Float &pdf,
                        Float &revPdf) const {
    Float cosWi = Dot(wi, normal);
    Vector3 normal_ = normal;
    if (fabs(cosWi) < c_CosEpsilon) {
        return false;
    }

    if (cosWi < Float(0.0)) {
        if (twoSided) {
            cosWi = -cosWi;
            normal_ = -normal_;
        } else {
            return false;
        }
    }

    ::Sample(normal_, rndParam, wo, cosWo, pdf);
    if (cosWo < c_CosEpsilon) {
        return false;
    }

    revPdf = cosWi * c_INVPI;
    contrib = Kd->Eval(st);

    return true;
}

void EvaluateLambertian(const bool adjoint,
                        const ADFloat *buffer,
                        const ADVector3 &wi,
                        const ADVector3 &normal,
                        const ADVector3 &wo,
                        const ADVector2 st,
                        ADVector3 &contrib,
                        ADFloat &cosWo,
                        ADFloat &pdf,
                        ADFloat &revPdf) {
    ADVector3 Kd;
    Deserialize(buffer, Kd);
    ADFloat cosWi = Dot(normal, wi);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(4);
    BeginIf(Gt(cosWi, Float(0.0)), ret);
    { SetCondOutput({normal[0], normal[1], normal[2], cosWi}); }
    BeginElse();
    { SetCondOutput({-normal[0], -normal[1], -normal[2], -cosWi}); }
    EndIf();
    ADVector3 normal_(ret[0], ret[1], ret[2]);
    cosWi = ret[3];
    cosWo = Dot(normal_, wo);
    ADFloat fwdScalar = cosWo * c_INVPI;
    ADFloat revScalar = cosWi * c_INVPI;
    contrib = fwdScalar * Kd;
    pdf = fwdScalar;
    revPdf = revScalar;
}

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
                      ADFloat &revPdf) {
    ADVector3 Kd;
    Deserialize(buffer, Kd);
    ADFloat cosWi = Dot(wi, normal);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(4);
    BeginIf(Gt(cosWi, Float(0.0)), ret);
    { SetCondOutput({normal[0], normal[1], normal[2], cosWi}); }
    BeginElse();
    { SetCondOutput({-normal[0], -normal[1], -normal[2], -cosWi}); }
    EndIf();
    ADVector3 normal_(ret[0], ret[1], ret[2]);
    cosWi = ret[3];
    Sample(normal_, rndParam, wo, cosWo, pdf);
    contrib = Kd;
    revPdf = cosWi * c_INVPI;
}
