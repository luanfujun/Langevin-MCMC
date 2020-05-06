#include "roughdielectric.h"
#include "microfacet.h"

int GetRoughDielectricSerializedSize() {
    return 1 +  // type
           3 +  // Ks
           3 +  // Kt
           1 +  // eta
           1 +  // invEta
           1;   // alpha
}

void RoughDielectric::Serialize(const Vector2 st, Float *buffer) const {
    buffer = ::Serialize((Float)BSDFType::RoughDielectric, buffer);
    buffer = ::Serialize(Ks->Eval(st), buffer);
    buffer = ::Serialize(Kt->Eval(st), buffer);
    buffer = ::Serialize(eta, buffer);
    buffer = ::Serialize(invEta, buffer);
    ::Serialize(alpha->Eval(st)[0], buffer);
}

template <bool adjoint>
void Evaluate(const RoughDielectric *bsdf,
              const Vector3 &wi,
              const Vector3 &normal,
              const Vector3 &wo,
              const Vector2 st,
              Vector3 &contrib,
              Float &cosWo,
              Float &pdf,
              Float &revPdf) {
    Float cosWi = Dot(wi, normal);
    contrib.setZero();
    cosWo = Float(0.0);
    pdf = revPdf = Float(0.0);
    if (fabs(cosWi) < c_CosEpsilon) {
        return;
    }
    cosWo = Dot(wo, normal);
    if (fabs(cosWo) < c_CosEpsilon) {
        return;
    }
    bool reflect = cosWi * cosWo > Float(0.0);
    Float eta_ = if_else(Gt(cosWi, Float(0.0)), bsdf->eta, bsdf->invEta);
    Float revEta_ = if_else(Gt(cosWo, Float(0.0)), bsdf->eta, bsdf->invEta);
    Vector3 H;
    if (reflect) {
        H = Normalize(Vector3(wi + wo));
    } else {
        H = Normalize(Vector3(wi + wo * eta_));
    }

    if (Dot(H, normal) < Float(0.0)) {
        H = -H;
    }

    Float cosHWi = Dot(wi, H);
    Float cosHWo = Dot(wo, H);
    if (fabs(cosHWi) < c_CosEpsilon || fabs(cosHWo) < c_CosEpsilon) {
        return;
    }
    contrib = Vector3::Zero();

    // Geometry term
    if (cosHWi * cosWi <= Float(0.0)) {
        return;
    }
    if (cosHWo * cosWo <= Float(0.0)) {
        return;
    }

    Vector3 b0;
    Vector3 b1;
    CoordinateSystem(normal, b0, b1);

    Vector3 localH = Vector3(Dot(b0, H), Dot(b1, H), Dot(normal, H));
    Float alp = bsdf->alpha->Eval(st)[0];
    Float D = BeckmennDistributionTerm(localH, alp, alp);
    if (D <= Float(0.0)) {
        return;
    }

    // This is confusing, but when computing reverse probability,
    // wo and wi are reversed
    Float revCosHWi = cosHWo;
    Float revCosHWo = cosHWi;

    Float F = FresnelDielectricExt(cosHWi, bsdf->eta, bsdf->invEta);

    Float aCosWi = fabs(cosWi);
    Float aCosWo = fabs(cosWo);

    Float G = BeckmennGeometryTerm(alp, aCosWi, aCosWo);
    Float scaledAlpha = alp * (Float(1.2) - Float(0.2) * sqrt(aCosWi));
    Float scaledD = BeckmennDistributionTerm(localH, scaledAlpha, scaledAlpha);
    Float prob = localH[2] * scaledD;

    if (prob < Float(1e-20)) {
        contrib = Vector3::Zero();
        return;
    }

    Float revScaledAlpha = alp * (Float(1.2) - Float(0.2) * sqrt(aCosWo));
    Float revScaledD = BeckmennDistributionTerm(localH, revScaledAlpha, revScaledAlpha);
    Float revProb = localH[2] * revScaledD;
    if (reflect) {
        Float scalar = fabs(F * D * G / (Float(4.0) * cosWi));
        contrib = bsdf->Ks->Eval(st) * scalar;
        pdf = fabs(prob * F / (Float(4.0) * cosHWo));
        revPdf = fabs(revProb * F / (Float(4.0) * revCosHWo));
    } else {
        Float sqrtDenom = cosHWi + eta_ * cosHWo;
        Float revSqrtDenom = revCosHWi + revEta_ * revCosHWo;
        Float factor = adjoint ? Float(1.0) : square(inverse(eta_));
        Float scalar = fabs(factor * ((Float(1.0) - F) * D * G * square(eta_) * cosHWi * cosHWo) /
                            (cosWi * square(sqrtDenom)));
        contrib = bsdf->Kt->Eval(st) * scalar;
        pdf = fabs(prob * (Float(1.0) - F) * (square(eta_) * cosHWo) / (square(sqrtDenom)));
        revPdf = fabs(revProb * (Float(1.0) - F) * (square(revEta_) * revCosHWo) /
                      (square(revSqrtDenom)));
    }
}

void RoughDielectric::Evaluate(const Vector3 &wi,
                               const Vector3 &normal,
                               const Vector3 &wo,
                               const Vector2 st,
                               Vector3 &contrib,
                               Float &cosWo,
                               Float &pdf,
                               Float &revPdf) const {
    ::Evaluate<false>(this, wi, normal, wo, st, contrib, cosWo, pdf, revPdf);
}

void RoughDielectric::EvaluateAdjoint(const Vector3 &wi,
                                      const Vector3 &normal,
                                      const Vector3 &wo,
                                      const Vector2 st,
                                      Vector3 &contrib,
                                      Float &cosWo,
                                      Float &pdf,
                                      Float &revPdf) const {
    ::Evaluate<true>(this, wi, normal, wo, st, contrib, cosWo, pdf, revPdf);
}

template <bool adjoint>
bool Sample(const RoughDielectric *bsdf,
            const Vector3 &wi,
            const Vector3 &normal,
            const Vector2 st,
            const Vector2 rndParam,
            // const Float /*uDiscrete*/,
            const Float uDiscrete,
            Vector3 &wo,
            Vector3 &contrib,
            Float &cosWo,
            Float &pdf,
            Float &revPdf) {
#if 0
    Float cosWi = Dot(wi, normal);
    if (fabs(cosWi) < c_CosEpsilon) {
        return false;
    }
    Float alp = bsdf->alpha->Eval(st)[0];
    Float scaledAlp = alp * (Float(1.2) - Float(0.2) * sqrt(fabs(cosWi)));
    Float mPdf;
    Vector3 b0;
    Vector3 b1;
    CoordinateSystem(normal, b0, b1);

    Float cosThetaT = 0.0;
    Float F0 = FresnelDielectricExt(cosWi, cosThetaT, bsdf->eta, bsdf->invEta);
    F0 = std::min(F0, Float(0.99));
    const Float uDiscrete = rndParam[0];
    bool reflect = uDiscrete <= F0; 
    Vector2 _rndParam = rndParam;
    if (reflect) {
        _rndParam[0] = uDiscrete / (F0 + Float(1e-10));
    } else {
        _rndParam[0] = (uDiscrete - F0) / (Float(1.0) - F0 + Float(1e-10));
    }
    Vector3 localH = SampleMicronormal(_rndParam, scaledAlp, mPdf);
    pdf = mPdf;
    Vector3 H = localH[0] * b0 + localH[1] * b1 + localH[2] * normal;
    Float cosHWi = Dot(wi, H);
    if (fabs(cosHWi) < c_CosEpsilon) {
        return false;
    }
    Float F = FresnelDielectricExt(cosHWi, cosThetaT, bsdf->eta, bsdf->invEta);

    Vector3 refl;
    Float cosHWo;
    Float trueFresnelFactor = Float(1.0);
#endif 

    Float cosWi = Dot(wi, normal);
    if (fabs(cosWi) < c_CosEpsilon) {
        return false;
    }
    Float alp = bsdf->alpha->Eval(st)[0];
    Float scaledAlp = alp * (Float(1.2) - Float(0.2) * sqrt(fabs(cosWi)));
    Float mPdf;
    Vector3 localH = SampleMicronormal(rndParam, scaledAlp, mPdf);
    pdf = mPdf;
    Vector3 b0;
    Vector3 b1;
    CoordinateSystem(normal, b0, b1);
    Vector3 H = localH[0] * b0 + localH[1] * b1 + localH[2] * normal;
    Float cosHWi = Dot(wi, H);
    if (fabs(cosHWi) < c_CosEpsilon) {
        return false;
    }
    Float cosThetaT = 0.0;
    Float F = FresnelDielectricExt(cosHWi, cosThetaT, bsdf->eta, bsdf->invEta);
    bool reflect = uDiscrete <= F;
    Vector3 refl;
    Float cosHWo;

    if (reflect) {
        wo = Reflect(wi, H);
        // side check
        if (F <= Float(0.0) || Dot(normal, wo) * Dot(normal, wi) <= Float(0.0)) {
            return false;
        }
        refl = bsdf->Ks->Eval(st);
        cosHWo = Dot(wo, H);
        pdf = fabs(pdf * F / (Float(4.0) * cosHWo));

        Float revCosHWo = cosHWi;
        Float rev_dwh_dwo = inverse(Float(4.0) * revCosHWo);
        cosWo = Dot(wo, normal);
        if (fabs(cosWo) < c_CosEpsilon) {
            return false;
        }
        Float revScaledAlp = alp * (Float(1.2) - Float(0.2) * sqrt(fabs(cosWo)));
        Float revD = BeckmennDistributionTerm(localH, revScaledAlp, revScaledAlp);
        revPdf = fabs(F * revD * localH[2] * rev_dwh_dwo);
        #if 0
        trueFresnelFactor = F / (F0 + Float(1e-10));
        #endif 
    } else {
        wo = Refract(wi, H, cosThetaT, bsdf->eta, bsdf->invEta);
        // side check
        if (F >= Float(1.0) || cosThetaT == Float(0.0) ||
            Dot(normal, wo) * Dot(normal, wi) >= Float(0.0)) {
            return false;
        }
        Float eta_ = if_else(Gt(cosWi, Float(0.0)), bsdf->eta, bsdf->invEta);
        Float factor = adjoint ? Float(1.0) : square(inverse(eta_));
        refl = bsdf->Kt->Eval(st) * factor;
        cosHWo = Dot(wo, H);
        Float sqrtDenom = cosHWi + eta_ * cosHWo;
        Float dwh_dwo = (square(eta_) * cosHWo) / square(sqrtDenom);
        pdf = fabs(pdf * (Float(1.0) - F) * fabs(dwh_dwo));

        cosWo = Dot(wo, normal);
        if (fabs(cosWo) < c_CosEpsilon) {
            return false;
        }
        Float revEta_ = if_else(Gt(cosWo, Float(0.0)), bsdf->eta, bsdf->invEta);
        Float revCosHWi = cosHWo;
        Float revCosHWo = cosHWi;
        Float revSqrtDenom = revCosHWi + revEta_ * revCosHWo;
        Float rev_dwh_dwo = (square(revEta_) * revCosHWo) / square(revSqrtDenom);
        Float revScaledAlp = alp * (Float(1.2) - Float(0.2) * sqrt(fabs(cosWo)));
        Float revD = BeckmennDistributionTerm(localH, revScaledAlp, revScaledAlp);
        revPdf = fabs((Float(1.0) - F) * revD * localH[2] * rev_dwh_dwo);
        #if 0
        trueFresnelFactor = (Float(1.0) - F) / (Float(1.0) - F0 + Float(1e-10));
        #endif
    }

    if (fabs(cosHWo) < c_CosEpsilon) {
        return false;
    }

    if (pdf < Float(1e-20)) {
        return false;
    }

    // Geometry term
    if (cosHWi * cosWi <= Float(0.0)) {
        return false;
    }
    if (cosHWo * cosWo <= Float(0.0)) {
        return false;
    }

    Float aCosWi = fabs(cosWi);
    Float aCosWo = fabs(cosWo);

    Float D = BeckmennDistributionTerm(localH, alp, alp);
    Float G = BeckmennGeometryTerm(alp, aCosWi, aCosWo);
    Float numerator = D * G * cosHWi;
    Float denominator = mPdf * aCosWi;
    #if 0
    contrib = refl * fabs(numerator / denominator) * trueFresnelFactor;
    #endif 
    contrib = refl * fabs(numerator / denominator);

    return true;
}

bool RoughDielectric::Sample(const Vector3 &wi,
                             const Vector3 &normal,
                             const Vector2 st,
                             const Vector2 rndParam,
                             const Float uDiscrete,
                             Vector3 &wo,
                             Vector3 &contrib,
                             Float &cosWo,
                             Float &pdf,
                             Float &revPdf) const {
    return ::Sample<false>(
        this, wi, normal, st, rndParam, uDiscrete, wo, contrib, cosWo, pdf, revPdf);
}

bool RoughDielectric::SampleAdjoint(const Vector3 &wi,
                                    const Vector3 &normal,
                                    const Vector2 st,
                                    const Vector2 rndParam,
                                    const Float uDiscrete,
                                    Vector3 &wo,
                                    Vector3 &contrib,
                                    Float &cosWo,
                                    Float &pdf,
                                    Float &revPdf) const {
    return ::Sample<true>(
        this, wi, normal, st, rndParam, uDiscrete, wo, contrib, cosWo, pdf, revPdf);
}

void EvaluateRoughDielectric(const bool adjoint,
                             const ADFloat *buffer,
                             const ADVector3 &wi,
                             const ADVector3 &normal,
                             const ADVector3 &wo,
                             const ADVector2 st,
                             ADVector3 &contrib,
                             ADFloat &cosWo,
                             ADFloat &pdf,
                             ADFloat &revPdf) {
    ADVector3 Ks;
    ADVector3 Kt;
    ADFloat eta;
    ADFloat invEta;
    ADFloat alpha;
    buffer = Deserialize(buffer, Ks);
    buffer = Deserialize(buffer, Kt);
    buffer = Deserialize(buffer, eta);
    buffer = Deserialize(buffer, invEta);
    buffer = Deserialize(buffer, alpha);

    ADFloat cosWi = Dot(wi, normal);
    cosWo = Dot(wo, normal);
    ADFloat side = cosWi * cosWo;
    ADFloat eta_ = if_else(Gt(cosWi, Float(0.0)), eta, invEta);
    ADFloat revEta_ = if_else(Gt(cosWo, Float(0.0)), eta, invEta);
    std::vector<CondExprCPtr> vH = CreateCondExprVec(3);
    BooleanCPtr reflect = Gt(side, Float(0.0));
    BeginIf(reflect, vH);
    {
        ADVector3 H = Normalize(ADVector3(wi + wo));
        SetCondOutput({H[0], H[1], H[2]});
    }
    BeginElse();
    {
        ADVector3 H = Normalize(ADVector3(wi + wo * eta_));
        SetCondOutput({H[0], H[1], H[2]});
    }
    EndIf();

    ADVector3 H(vH[0], vH[1], vH[2]);

    ADFloat Hside = Dot(H, normal);
    vH = CreateCondExprVec(3);
    BeginIf(Lt(Hside, Float(0.0)), vH);
    { SetCondOutput({-H[0], -H[1], -H[2]}); }
    BeginElse();
    { SetCondOutput({H[0], H[1], H[2]}); }
    EndIf();

    H = ADVector3(vH[0], vH[1], vH[2]);

    ADFloat cosHWi = Dot(wi, H);
    ADFloat cosHWo = Dot(wo, H);
    contrib = ADVector3(Const<ADFloat>(0.0), Const<ADFloat>(0.0), Const<ADFloat>(0.0));

    ADVector3 b0;
    ADVector3 b1;
    CoordinateSystem(normal, b0, b1);

    ADVector3 localH = ADVector3(Dot(b0, H), Dot(b1, H), Dot(normal, H));
    ADFloat D = BeckmennDistributionTerm(localH, alpha, alpha);
    // This is confusing, but when computing reverse probability,
    // wo and wi are reversed
    ADFloat revCosHWi = cosHWo;
    ADFloat revCosHWo = cosHWi;
    ADFloat F = FresnelDielectricExt(cosHWi, eta, invEta);
    ADFloat aCosWi = fabs(cosWi);
    ADFloat aCosWo = fabs(cosWo);

    ADFloat G = BeckmennGeometryTerm(alpha, aCosWi, aCosWo);
    ADFloat scaledAlpha = alpha * (Float(1.2) - Float(0.2) * sqrt(aCosWi));
    ADFloat scaledD = BeckmennDistributionTerm(localH, scaledAlpha, scaledAlpha);
    ADFloat prob = localH[2] * scaledD;
    ADFloat revScaledAlpha = alpha * (Float(1.2) - Float(0.2) * sqrt(aCosWo));
    ADFloat revScaledD = BeckmennDistributionTerm(localH, revScaledAlpha, revScaledAlpha);
    ADFloat revProb = localH[2] * revScaledD;
    std::vector<CondExprCPtr> ret = CreateCondExprVec(5);
    BeginIf(reflect, ret);
    {
        ADFloat scalar = fabs(F * D * G / (Float(4.0) * cosWi));
        ADVector3 contrib = Ks * scalar;
        ADFloat pdf = fabs(prob * F / (Float(4.0) * cosHWo));
        ADFloat revPdf = fabs(revProb * F / (Float(4.0) * revCosHWo));
        SetCondOutput({contrib[0], contrib[1], contrib[2], pdf, revPdf});
    }
    BeginElse();
    {
        ADFloat sqrtDenom = cosHWi + eta_ * cosHWo;
        ADFloat revSqrtDenom = revCosHWi + revEta_ * revCosHWo;
        ADFloat factor = adjoint ? Const<ADFloat>(1.0) : square(inverse(eta_));
        ADFloat scalar = fabs(factor * ((Float(1.0) - F) * D * G * square(eta_) * cosHWi * cosHWo) /
                              (cosWi * square(sqrtDenom)));
        ADVector3 contrib = Kt * scalar;
        ADFloat pdf = fabs(prob * (Float(1.0) - F) * (square(eta_) * cosHWo) / (square(sqrtDenom)));
        ADFloat revPdf = fabs(revProb * (Float(1.0) - F) * (square(revEta_) * revCosHWo) /
                              (square(revSqrtDenom)));
        SetCondOutput({contrib[0], contrib[1], contrib[2], pdf, revPdf});
    }
    EndIf();
    contrib = ADVector3(ret[0], ret[1], ret[2]);
    pdf = ret[3];
    revPdf = ret[4];
}

void SampleRoughDielectric(const bool adjoint,
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
    ADVector3 Ks;
    ADVector3 Kt;
    ADFloat eta;
    ADFloat invEta;
    ADFloat alpha;
    buffer = Deserialize(buffer, Ks);
    buffer = Deserialize(buffer, Kt);
    buffer = Deserialize(buffer, eta);
    buffer = Deserialize(buffer, invEta);
    buffer = Deserialize(buffer, alpha);

    ADFloat cosWi = Dot(wi, normal);
    ADFloat scaledAlpha = alpha * (Float(1.2) - Float(0.2) * sqrt(fabs(cosWi)));
    ADFloat mPdf;
    ADVector3 localH = SampleMicronormal(rndParam, scaledAlpha, mPdf);
    ADVector3 b0;
    ADVector3 b1;
    CoordinateSystem(normal, b0, b1);
    ADVector3 H = localH[0] * b0 + localH[1] * b1 + localH[2] * normal;
    ADFloat cosHWi = Dot(wi, H);
    ADFloat cosThetaT;
    ADFloat F = FresnelDielectricExt(cosHWi, cosThetaT, eta, invEta);
    BooleanCPtr reflect = Lte(uDiscrete, F);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(9);
    BeginIf(reflect, ret);
    {
        ADVector3 wo = Reflect(wi, H);
        ADVector3 refl = Ks;
        if (fixDiscrete) {
            refl *= F;
        }
        ADFloat cosHWo = Dot(wo, H);
        ADFloat pdf = fabs(mPdf * F / (Float(4.0) * cosHWo));
        ADFloat revCosHWo = cosHWi;
        ADFloat rev_dwh_dwo = inverse(Float(4.0) * revCosHWo);
        ADFloat cosWo = Dot(wo, normal);
        ADFloat revScaledAlp = alpha * (Float(1.2) - Float(0.2) * sqrt(fabs(cosWo)));
        ADFloat revD = BeckmennDistributionTerm(localH, revScaledAlp, revScaledAlp);
        ADFloat revPdf = fabs(F * revD * localH[2] * rev_dwh_dwo);
        SetCondOutput({wo[0], wo[1], wo[2], refl[0], refl[1], refl[2], cosWo, pdf, revPdf});
    }
    BeginElse();
    {
        ADVector3 wo = Refract(wi, H, cosThetaT, eta, invEta);
        ADFloat eta_ = if_else(Gt(cosWi, Float(0.0)), eta, invEta);
        ADFloat factor = adjoint ? Const<ADFloat>(1.0) : square(inverse(eta_));
        ADVector3 refl = Kt * factor;
        if (fixDiscrete) {
            refl *= (Float(1.0) - F);
        }
        ADFloat cosHWo = Dot(wo, H);
        ADFloat sqrtDenom = cosHWi + eta_ * cosHWo;
        ADFloat dwh_dwo = (square(eta_) * cosHWo) / square(sqrtDenom);
        ADFloat pdf = fabs(mPdf * (Float(1.0) - F) * fabs(dwh_dwo));
        ADFloat cosWo = Dot(wo, normal);
        ADFloat revEta_ = if_else(Gt(cosWo, Float(0.0)), eta, invEta);
        ADFloat revCosHWi = cosHWo;
        ADFloat revCosHWo = cosHWi;
        ADFloat revSqrtDenom = revCosHWi + revEta_ * revCosHWo;
        ADFloat rev_dwh_dwo = (square(revEta_) * revCosHWo) / square(revSqrtDenom);
        ADFloat revScaledAlp = alpha * (Float(1.2) - Float(0.2) * sqrt(fabs(cosWo)));
        ADFloat revD = BeckmennDistributionTerm(localH, revScaledAlp, revScaledAlp);
        ADFloat revPdf = fabs((Float(1.0) - F) * revD * localH[2] * rev_dwh_dwo);
        SetCondOutput({wo[0], wo[1], wo[2], refl[0], refl[1], refl[2], cosWo, pdf, revPdf});
    }
    EndIf();
    wo = ADVector3(ret[0], ret[1], ret[2]);
    ADVector3 refl = ADVector3(ret[3], ret[4], ret[5]);
    cosWo = ret[6];
    pdf = ret[7];
    revPdf = ret[8];

    ADFloat aCosWi = fabs(cosWi);
    ADFloat aCosWo = fabs(cosWo);
    ADFloat D = BeckmennDistributionTerm(localH, alpha, alpha);
    ADFloat G = BeckmennGeometryTerm(alpha, aCosWi, aCosWo);
    ADFloat numerator = D * G * cosHWi;
    ADFloat denominator = mPdf * aCosWi;
    contrib = refl * fabs(numerator / denominator);
}