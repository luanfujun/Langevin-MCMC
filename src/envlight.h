#pragma once

#include "light.h"
#include "animatedtransform.h"

struct Image3;
struct PiecewiseConstant2D;

int GetEnvLightSerializedSize();

struct EnvmapSampleInfo {
    std::vector<Float> cdfRows;
    std::vector<Float> cdfCols;
    std::vector<Float> rowWeights;
    Float normalization;
    Vector2 pixelSize;
};

struct EnvLight : public Light {
    EnvLight(const Float &samplingWeight,
             const AnimatedTransform &toWorld,
             const std::string &filename);

    LightType GetType() const override {
        return LightType::EnvLight;
    }
    void Serialize(const LightPrimID &lPrimID, Float *buffer) const override;
    bool SampleDirect(const BSphere &sceneSphere,
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
                      Float &emissionPdf) const override;
    void Emission(const BSphere &sceneSphere,
                  const Vector3 &dirToLight,
                  const Vector3 &normalOnLight,
                  const Float time,
                  LightPrimID &lPrimID,
                  Vector3 &emission,
                  Float &directPdf,
                  Float &emissionPdf) const override;
    void Emit(const BSphere &sceneSphere,
              const Vector2 rndParamPos,
              const Vector2 rndParamDir,
              const Float time,
              LightPrimID &lPrimID,
              Ray &ray,
              Vector3 &emission,
              Float &cosAtLight,
              Float &emissionPdf,
              Float &directPdf) const override;
    bool IsFinite() const override {
        return false;
    }
    bool IsDelta() const override {
        return false;
    }

    const AnimatedTransform toWorld;
    const AnimatedTransform toLight;
    const std::unique_ptr<const Image3> image;
    const std::unique_ptr<const EnvmapSampleInfo> sampleInfo;
};

void SampleDirectEnvLight(const ADFloat *buffer,
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

void EmissionEnvLight(const ADFloat *buffer,
                      const ADBSphere &sceneSphere,
                      const ADVector3 &dirToLight,
                      const ADVector3 &normalOnLight,
                      const ADFloat time,
                      ADVector3 &emission,
                      ADFloat &directPdf,
                      ADFloat &emissionPdf);

void EmitEnvLight(const ADFloat *buffer,
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
