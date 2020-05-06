#pragma once

#include "commondef.h"
#include "utils.h"

template <typename FloatType>
FloatType BeckmennDistributionTerm(const TVector3<FloatType> &localH,
                                   const FloatType &alphaU,
                                   const FloatType &alphaV) {
    const FloatType cosTheta = localH[2];
    const FloatType mu = localH[0];
    const FloatType mv = localH[1];
    FloatType cosTheta2 = square(cosTheta);
    FloatType beckmannExponent =
        (square(mu) / square(alphaU) + square(mv) / square(alphaV)) / cosTheta2;
    FloatType D = exp(-beckmannExponent) / (c_PI * alphaU * alphaV * square(cosTheta2));
    return D;
}

inline Float BeckmennGeometryTerm(const Float alpha, const Float cosTheta) {
    Float tanTheta = sqrt(fabs(Float(1.0) - square(cosTheta))) / cosTheta;
    if (tanTheta <= 0.0)
        return Float(1.0);
    Float G = 0.0;
    Float a = Float(1.0) / (alpha * tanTheta);
    if (a >= Float(1.6)) {
        G = Float(1.0);
    } else {
        /* Use a fast and accurate (<0.35% rel. error) rational
           approximation to the shadowing-masking function */
        // Note: this could be a bad approximation for derivatives,
        //       ideally we should refit the function
        Float aSqr = a * a;
        G = (Float(3.535) * a + Float(2.181) * aSqr) /
            (Float(1.0) + Float(2.276) * a + Float(2.577) * aSqr);
    }
    return G;
}

inline ADFloat BeckmennGeometryTerm(const ADFloat alpha, const ADFloat cosTheta) {
    ADFloat tanTheta = sqrt(fabs((Float(1.0) + Float(1e-6)) - square(cosTheta))) / cosTheta;
    CondExprCPtr ret = CondExpr::Create();
    BeginIf(Lte(tanTheta, Float(0.0)), {ret});
    { SetCondOutput({Const<ADFloat>(1.0)}); }
    BeginElse();
    {
        CondExprCPtr G = CondExpr::Create();
        ADFloat a = inverse(alpha * tanTheta);
        BeginIf(Gte(a, Float(1.6)), {G});
        { SetCondOutput({Const<ADFloat>(1.0)}); }
        BeginElse();
        {
            ADFloat aSqr = square(a);
            ADFloat G_ = (Float(3.535) * a + Float(2.181) * aSqr) /
                         (Float(1.0) + Float(2.276) * a + Float(2.577) * aSqr);
            SetCondOutput({G_});
        }
        EndIf();
        SetCondOutput({G});
    }
    EndIf();
    return ret;
}

template <typename FloatType>
FloatType BeckmennGeometryTerm(const FloatType alpha,
                               const FloatType cosWi,
                               const FloatType cosWo) {
    return BeckmennGeometryTerm(alpha, cosWi) * BeckmennGeometryTerm(alpha, cosWo);
}

inline Float FresnelDielectricExt(const Float cosThetaI_,
                                  Float &cosThetaT_,
                                  const Float eta,
                                  const Float invEta) {
    Float scale = (cosThetaI_ > 0) ? invEta : eta;
    Float cosThetaTSqr = Float(1.0) - (Float(1.0) - square(cosThetaI_)) * square(scale);

    if (cosThetaTSqr <= Float(0.0)) {
        cosThetaT_ = Float(0.0);
        return Float(1.0);
    }

    Float cosThetaI = fabs(cosThetaI_);
    Float cosThetaT = sqrt(cosThetaTSqr);

    Float Rs = (cosThetaI - eta * cosThetaT) / (cosThetaI + eta * cosThetaT);
    Float Rp = (eta * cosThetaI - cosThetaT) / (eta * cosThetaI + cosThetaT);

    cosThetaT_ = (cosThetaI_ > 0) ? -cosThetaT : cosThetaT;

    return Float(0.5) * (square(Rs) + square(Rp));
}

inline Float FresnelDielectricExt(const Float cosThetaI_, const Float eta, const Float invEta) {
    Float scale = (cosThetaI_ > 0) ? invEta : eta;
    Float cosThetaTSqr = Float(1.0) - (Float(1.0) - square(cosThetaI_)) * square(scale);

    if (cosThetaTSqr <= Float(0.0)) {
        return Float(1.0);
    }

    Float cosThetaI = fabs(cosThetaI_);
    Float cosThetaT = sqrt(cosThetaTSqr);

    Float Rs = (cosThetaI - eta * cosThetaT) / (cosThetaI + eta * cosThetaT);
    Float Rp = (eta * cosThetaI - cosThetaT) / (eta * cosThetaI + cosThetaT);

    return Float(0.5) * (square(Rs) + square(Rp));
}

inline ADFloat FresnelDielectricExt(const ADFloat cosThetaI_,
                                    ADFloat &cosThetaT_,
                                    const ADFloat eta,
                                    const ADFloat invEta) {
    ADFloat scale = if_else(Gt(cosThetaI_, Float(0.0)), invEta, eta);
    ADFloat cosThetaTSqr = Float(1.0) - (Float(1.0) - square(cosThetaI_)) * square(scale);

    std::vector<CondExprCPtr> ret = CreateCondExprVec(2);
    BeginIf(Lte(cosThetaTSqr, Float(0.0)), ret);
    { SetCondOutput({Const<ADFloat>(0.0), Const<ADFloat>(1.0)}); }
    BeginElse();
    {
        ADFloat cosThetaI = fabs(cosThetaI_);
        ADFloat cosThetaT = sqrt(cosThetaTSqr);
        ADFloat etaCosThetaT = eta * cosThetaT;
        ADFloat etaCosThetaI = eta * cosThetaI;
        ADFloat Rs = (cosThetaI - etaCosThetaT) / (cosThetaI + etaCosThetaT);
        ADFloat Rp = (etaCosThetaI - cosThetaT) / (etaCosThetaI + cosThetaT);

        ADFloat cosThetaT_ = if_else(Gt(cosThetaI_, Float(0.0)), -cosThetaT, cosThetaT);
        ADFloat F = Float(0.5) * (square(Rs) + square(Rp));
        SetCondOutput({cosThetaT_, F});
    }
    EndIf();

    cosThetaT_ = ret[0];
    return ret[1];
}

inline ADFloat FresnelDielectricExt(const ADFloat cosThetaI_,
                                    const ADFloat eta,
                                    const ADFloat invEta) {
    ADFloat scale = if_else(Gt(cosThetaI_, Float(0.0)), invEta, eta);
    ADFloat cosThetaTSqr = Float(1.0) - (Float(1.0) - square(cosThetaI_)) * square(scale);

    std::vector<CondExprCPtr> ret = CreateCondExprVec(1);
    BeginIf(Lte(cosThetaTSqr, Float(0.0)), ret);
    { SetCondOutput({Const<ADFloat>(1.0)}); }
    BeginElse();
    {
        ADFloat cosThetaI = fabs(cosThetaI_);
        ADFloat cosThetaT = sqrt(cosThetaTSqr);
        ADFloat etaCosThetaT = eta * cosThetaT;
        ADFloat etaCosThetaI = eta * cosThetaI;
        ADFloat Rs = (cosThetaI - etaCosThetaT) / (cosThetaI + etaCosThetaT);
        ADFloat Rp = (etaCosThetaI - cosThetaT) / (etaCosThetaI + cosThetaT);

        ADFloat F = Float(0.5) * (square(Rs) + square(Rp));
        SetCondOutput({F});
    }
    EndIf();
    return ret[0];
}

template <typename FloatType>
TVector3<FloatType> SampleMicronormal(const TVector2<FloatType> rndParam,
                                      const FloatType alpha,
                                      FloatType &pdfW) {
    FloatType phiM = c_TWOPI * rndParam[1];
    FloatType sinPhiM = sin(phiM);
    FloatType cosPhiM = cos(phiM);
    FloatType alphaSqr = square(alpha);

    FloatType tanThetaMSqr = alphaSqr * (-log(fmax(Float(1.0) - rndParam[0], Float(1e-6))));
    FloatType cosThetaM = Float(1.0) / sqrt(Float(1.0) + tanThetaMSqr);
    FloatType cosThetaMSqr = square(cosThetaM);

    pdfW = (Float(1.0) - rndParam[0]) / (c_PI * alphaSqr * cosThetaM * cosThetaMSqr);
    FloatType sinThetaMSq = fmax(Float(1.0) - cosThetaMSqr, ADEpsilon<FloatType>());
    FloatType sinThetaM = sqrt(sinThetaMSq);

    TVector3<FloatType> localH(sinThetaM * cosPhiM, sinThetaM * sinPhiM, cosThetaM);
    return localH;
}
