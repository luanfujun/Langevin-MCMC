#include "bsdf.h"
#include "lambertian.h"
#include "phong.h"
#include "roughdielectric.h"
#include "utils.h"

int GetMaxBSDFSerializedSize() {
    return std::max({GetLambertianSerializedSize(),
                     GetPhongSerializedSize(),
                     GetRoughDielectricSerializedSize()});
}

const ADFloat *EvaluateBSDF(const bool adjoint,
                            const ADFloat *buffer,
                            const ADVector3 &wi,
                            const ADVector3 &normal,
                            const ADVector3 &wo,
                            const ADVector2 st,
                            ADVector3 &contrib,
                            ADFloat &cosWo,
                            ADFloat &pdf,
                            ADFloat &revPdf) {
    ADFloat type;
    buffer = Deserialize(buffer, type);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(6);
    BeginIf(Eq(type, (Float)BSDFType::Phong), ret);
    {
        ADVector3 contrib;
        ADFloat cosWo, pdf, revPdf;
        EvaluatePhong(adjoint, buffer, wi, normal, wo, st, contrib, cosWo, pdf, revPdf);
        SetCondOutput({contrib[0], contrib[1], contrib[2], cosWo, pdf, revPdf});
    }
    BeginElseIf(Eq(type, (Float)BSDFType::RoughDielectric));
    {
        ADVector3 contrib;
        ADFloat cosWo, pdf, revPdf;
        EvaluateRoughDielectric(adjoint, buffer, wi, normal, wo, st, contrib, cosWo, pdf, revPdf);
        SetCondOutput({contrib[0], contrib[1], contrib[2], cosWo, pdf, revPdf});
    }
    BeginElseIf(Eq(type, (Float)BSDFType::Lambertian));
    {
        ADVector3 contrib;
        ADFloat cosWo, pdf, revPdf;
        EvaluateLambertian(adjoint, buffer, wi, normal, wo, st, contrib, cosWo, pdf, revPdf);
        SetCondOutput({contrib[0], contrib[1], contrib[2], cosWo, pdf, revPdf});
    }
    BeginElse();
    {
        SetCondOutput({Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0)});
    }
    EndIf();
    contrib[0] = ret[0];
    contrib[1] = ret[1];
    contrib[2] = ret[2];
    cosWo = ret[3];
    pdf = ret[4];
    revPdf = ret[5];
    buffer += (GetMaxBSDFSerializedSize() - 1);
    return buffer;
}

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
                          ADFloat &revPdf) {
    ADFloat type;
    buffer = Deserialize(buffer, type);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(9);
    BeginIf(Eq(type, (Float)BSDFType::Phong), ret);
    {
        ADVector3 wo;
        ADVector3 contrib;
        ADFloat cosWo, pdf, revPdf;
        SamplePhong(adjoint,
                    buffer,
                    wi,
                    normal,
                    st,
                    rndParam,
                    uDiscrete,
                    fixDiscrete,
                    wo,
                    contrib,
                    cosWo,
                    pdf,
                    revPdf);
        SetCondOutput(
            {wo[0], wo[1], wo[2], contrib[0], contrib[1], contrib[2], cosWo, pdf, revPdf});
    }
    BeginElseIf(Eq(type, (Float)BSDFType::RoughDielectric));
    {
        ADVector3 wo;
        ADVector3 contrib;
        ADFloat cosWo, pdf, revPdf;
        SampleRoughDielectric(adjoint,
                              buffer,
                              wi,
                              normal,
                              st,
                              rndParam,
                              uDiscrete,
                              fixDiscrete,
                              wo,
                              contrib,
                              cosWo,
                              pdf,
                              revPdf);
        SetCondOutput(
            {wo[0], wo[1], wo[2], contrib[0], contrib[1], contrib[2], cosWo, pdf, revPdf});
    }
    BeginElseIf(Eq(type, (Float)BSDFType::Lambertian));
    {
        ADVector3 wo;
        ADVector3 contrib;
        ADFloat cosWo, pdf, revPdf;
        SampleLambertian(adjoint,
                         buffer,
                         wi,
                         normal,
                         st,
                         rndParam,
                         uDiscrete,
                         fixDiscrete,
                         wo,
                         contrib,
                         cosWo,
                         pdf,
                         revPdf);
        SetCondOutput(
            {wo[0], wo[1], wo[2], contrib[0], contrib[1], contrib[2], cosWo, pdf, revPdf});
    }

    BeginElse();
    {
        SetCondOutput({Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0)});
    }
    EndIf();
    wo[0] = ret[0];
    wo[1] = ret[1];
    wo[2] = ret[2];
    contrib[0] = ret[3];
    contrib[1] = ret[4];
    contrib[2] = ret[5];
    cosWo = ret[6];
    pdf = ret[7];
    revPdf = ret[8];
    buffer += (GetMaxBSDFSerializedSize() - 1);
    return buffer;
}
