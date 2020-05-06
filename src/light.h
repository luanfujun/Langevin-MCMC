#pragma once

#include "commondef.h"
#include "bounds.h"
#include "ray.h"

enum class LightType { PointLight, AreaLight, EnvLight };

int GetMaxLightSerializedSize();

typedef PrimID LightPrimID;
const LightPrimID INVALID_LPRIM_ID = LightPrimID(-1);

struct Light {
    Light(const Float &samplingWeight) : samplingWeight(samplingWeight) {
    }

    virtual LightType GetType() const = 0;
    virtual void Serialize(const LightPrimID &lPrimID, Float *buffer) const = 0;
    virtual LightPrimID SampleDiscrete(const Float uDiscrete) const {
        return INVALID_LPRIM_ID;
    }
    virtual bool SampleDirect(const BSphere &sceneSphere,
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
                              Float &emissionPdf) const = 0;
    virtual void Emission(const BSphere &sceneSphere,
                          const Vector3 &dirToLight,
                          const Vector3 &normalOnLight,
                          const Float time,
                          LightPrimID &lPrimID,
                          Vector3 &emission,
                          Float &directPdf,
                          Float &emissionPdf) const {
        Error("Unimplemented method");
    }
    virtual void Emit(const BSphere &sceneSphere,
                      const Vector2 rndParamPos,
                      const Vector2 rndParamDir,
                      const Float time,
                      LightPrimID &lPrimID,
                      Ray &ray,
                      Vector3 &emission,
                      Float &cosAtLight,
                      Float &emissionPdf,
                      Float &directPdf) const = 0;
    virtual bool IsFinite() const = 0;
    virtual bool IsDelta() const = 0;

    Float samplingWeight;
};

struct LightInst {
    const Light *light;
    LightPrimID lPrimID;
};

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
                            ADFloat &emissionPdf);

const ADFloat *Emission(const ADFloat *buffer,
                        const ADBSphere &sceneSphere,
                        const ADVector3 &dirToLight,
                        const ADVector3 &normalOnLight,
                        const ADFloat time,
                        ADVector3 &emission,
                        ADFloat &directPdf,
                        ADFloat &emissionPdf);

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
                    ADFloat &directPdf);
