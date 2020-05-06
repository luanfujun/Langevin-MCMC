#include "light.h"
#include "pointlight.h"
#include "arealight.h"
#include "envlight.h"
#include "utils.h"

int GetMaxLightSerializedSize() {
    return std::max(
        {GetPointLightSerializedSize(), GetAreaLightSerializedSize(), GetEnvLightSerializedSize()});
}

const ADFloat *SampleDirect(const ADFloat *buffer,
                            const ADBSphere &sceneSphere,
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
    ADFloat type;
    buffer = Deserialize(buffer, type);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(9);
    BeginIf(Eq(type, (Float)LightType::PointLight), ret);
    {
        ADVector3 dirToLight;
        ADVector3 lightContrib;
        ADFloat cosAtLight;
        ADFloat directPdf;
        ADFloat emissionPdf;
        SampleDirectPointLight(buffer,
                               sceneSphere,
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
        SetCondOutput({dirToLight[0],
                       dirToLight[1],
                       dirToLight[2],
                       lightContrib[0],
                       lightContrib[1],
                       lightContrib[2],
                       cosAtLight,
                       directPdf,
                       emissionPdf});
    }
    BeginElseIf(Eq(type, (Float)LightType::AreaLight));
    {
        ADVector3 dirToLight;
        ADVector3 lightContrib;
        ADFloat cosAtLight;
        ADFloat directPdf;
        ADFloat emissionPdf;
        SampleDirectAreaLight(buffer,
                              sceneSphere,
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
        SetCondOutput({dirToLight[0],
                       dirToLight[1],
                       dirToLight[2],
                       lightContrib[0],
                       lightContrib[1],
                       lightContrib[2],
                       cosAtLight,
                       directPdf,
                       emissionPdf});
    }
    BeginElseIf(Eq(type, (Float)LightType::EnvLight));
    {
        ADVector3 dirToLight;
        ADVector3 lightContrib;
        ADFloat cosAtLight;
        ADFloat directPdf;
        ADFloat emissionPdf;
        SampleDirectEnvLight(buffer,
                             sceneSphere,
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
        SetCondOutput({dirToLight[0],
                       dirToLight[1],
                       dirToLight[2],
                       lightContrib[0],
                       lightContrib[1],
                       lightContrib[2],
                       cosAtLight,
                       directPdf,
                       emissionPdf});
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
    dirToLight[0] = ret[0];
    dirToLight[1] = ret[1];
    dirToLight[2] = ret[2];
    lightContrib[0] = ret[3];
    lightContrib[1] = ret[4];
    lightContrib[2] = ret[5];
    cosAtLight = ret[6];
    directPdf = ret[7];
    emissionPdf = ret[8];
    buffer += (GetMaxLightSerializedSize() - 1);
    return buffer;
}

const ADFloat *Emission(const ADFloat *buffer,
                        const ADBSphere &sceneSphere,
                        const ADVector3 &dirToLight,
                        const ADVector3 &normalOnLight,
                        const ADFloat time,
                        ADVector3 &emission,
                        ADFloat &directPdf,
                        ADFloat &emissionPdf) {
    ADFloat type;
    buffer = Deserialize(buffer, type);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(5);
    BeginIf(Eq(type, (Float)LightType::AreaLight), ret);
    {
        ADVector3 emission;
        ADFloat directPdf;
        ADFloat emissionPdf;
        EmissionAreaLight(
            buffer, sceneSphere, dirToLight, normalOnLight, time, emission, directPdf, emissionPdf);
        SetCondOutput({emission[0], emission[1], emission[2], directPdf, emissionPdf});
    }
    BeginElseIf(Eq(type, (Float)LightType::EnvLight));
    {
        ADVector3 emission;
        ADFloat directPdf;
        ADFloat emissionPdf;
        EmissionEnvLight(
            buffer, sceneSphere, dirToLight, normalOnLight, time, emission, directPdf, emissionPdf);
        SetCondOutput({emission[0], emission[1], emission[2], directPdf, emissionPdf});
    }
    BeginElse();
    {
        SetCondOutput({Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0)});
    }
    EndIf();
    emission[0] = ret[0];
    emission[1] = ret[1];
    emission[2] = ret[2];
    directPdf = ret[3];
    emissionPdf = ret[4];
    buffer += (GetMaxLightSerializedSize() - 1);
    return buffer;
}

const ADFloat *Emit(const ADFloat *buffer,
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
    ADFloat type;
    buffer = Deserialize(buffer, type);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(12);
    BeginIf(Eq(type, (Float)LightType::PointLight), ret);
    {
        ADRay ray;
        ADVector3 emission;
        ADFloat cosAtLight;
        ADFloat emissionPdf;
        ADFloat directPdf;
        EmitPointLight(buffer,
                       sceneSphere,
                       rndParamPos,
                       rndParamDir,
                       time,
                       isStatic,
                       ray,
                       emission,
                       cosAtLight,
                       emissionPdf,
                       directPdf);
        SetCondOutput({ray.org[0],
                       ray.org[1],
                       ray.org[2],
                       ray.dir[0],
                       ray.dir[1],
                       ray.dir[2],
                       emission[0],
                       emission[1],
                       emission[2],
                       cosAtLight,
                       emissionPdf,
                       directPdf});
    }
    BeginElseIf(Eq(type, (Float)LightType::AreaLight));
    {
        ADRay ray;
        ADVector3 emission;
        ADFloat cosAtLight;
        ADFloat emissionPdf;
        ADFloat directPdf;
        EmitAreaLight(buffer,
                      sceneSphere,
                      rndParamPos,
                      rndParamDir,
                      time,
                      isStatic,
                      ray,
                      emission,
                      cosAtLight,
                      emissionPdf,
                      directPdf);
        SetCondOutput({ray.org[0],
                       ray.org[1],
                       ray.org[2],
                       ray.dir[0],
                       ray.dir[1],
                       ray.dir[2],
                       emission[0],
                       emission[1],
                       emission[2],
                       cosAtLight,
                       emissionPdf,
                       directPdf});
    }
    BeginElseIf(Eq(type, (Float)LightType::EnvLight));
    {
        ADRay ray;
        ADVector3 emission;
        ADFloat cosAtLight;
        ADFloat emissionPdf;
        ADFloat directPdf;
        EmitEnvLight(buffer,
                     sceneSphere,
                     rndParamPos,
                     rndParamDir,
                     time,
                     isStatic,
                     ray,
                     emission,
                     cosAtLight,
                     emissionPdf,
                     directPdf);
        SetCondOutput({ray.org[0],
                       ray.org[1],
                       ray.org[2],
                       ray.dir[0],
                       ray.dir[1],
                       ray.dir[2],
                       emission[0],
                       emission[1],
                       emission[2],
                       cosAtLight,
                       emissionPdf,
                       directPdf});
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
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0),
                       Const<ADFloat>(0.0)});
    }
    EndIf();
    ray.org[0] = ret[0];
    ray.org[1] = ret[1];
    ray.org[2] = ret[2];
    ray.dir[0] = ret[3];
    ray.dir[1] = ret[4];
    ray.dir[2] = ret[5];
    emission[0] = ret[6];
    emission[1] = ret[7];
    emission[2] = ret[8];
    cosAtLight = ret[9];
    emissionPdf = ret[10];
    directPdf = ret[11];
    buffer += (GetMaxLightSerializedSize() - 1);
    return buffer;
}
