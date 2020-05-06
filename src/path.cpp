#include "path.h"
#include "utils.h"
#include "camera.h"
#include "bsdf.h"
#include "shape.h"
#include "light.h"
#include "scene.h"
#include "envlight.h"
#include "arealight.h"
#include "ray.h"
#include "sampling.h"

#include <limits>
#include <random>

void Clear(Path &path) {
    path.camSurfaceVertex.clear();
    path.lgtSurfaceVertex.clear();
    path.envLightInst.light = nullptr;
    path.isSubpath = false;
}

template <typename FloatType>
static inline FloatType MISWeight(const FloatType pdfA, const FloatType pdfB) {
    FloatType ratioSq = square(pdfB / pdfA);
    return Float(1.0) / (Float(1.0) + ratioSq);
}

template <typename FloatType>
static inline FloatType MIS(const FloatType pdf) {
    return square(pdf);
}

template <bool adjoint>
static Float ShadingNormalCorrection(const Vector3 &wi,
                                     const Intersection &isect,
                                     const Vector3 &wo) {
    // prevent light leak
    const Float cosWi = Dot(isect.shadingNormal, wi);
    const Float cosWo = Dot(isect.shadingNormal, wo);
    Float wiDotGeoN = Dot(isect.geomNormal, wi);
    Float woDotGeoN = Dot(isect.geomNormal, wo);
    if (wiDotGeoN * cosWi <= Float(0.0) || woDotGeoN * cosWo <= Float(0.0)) {
        return Float(0.0);
    }

    if (adjoint) {
        // Adjoint BSDF factor for shading normal
        Float factor = fabs((woDotGeoN * cosWi) / (wiDotGeoN * cosWo));
        return factor;
    } else {
        return Float(1.0);
    }
}

template <bool adjoint>
static ADFloat ShadingNormalCorrection(const ADVector3 &wi,
                                       const ADIntersection &isect,
                                       const ADVector3 &wo) {
    if (adjoint) {
        // Adjoint BSDF factor for shading normal
        ADFloat cosWi = Dot(isect.shadingNormal, wi);
        ADFloat cosWo = Dot(isect.shadingNormal, wo);
        ADFloat wiDotGeoN = Dot(isect.geomNormal, wi);
        ADFloat woDotGeoN = Dot(isect.geomNormal, wo);
        return fabs((woDotGeoN * cosWi) / (wiDotGeoN * cosWo));
    } else {
        return Const<ADFloat>(1.0);
    }
}

struct UniPathState {
    RaySegment raySeg;
    Vector3 throughput;
    Vector3 lensThroughput;
    Float lastBsdfPdf;
    Intersection isect;
    Vector3 wi;
    Float ssJacobian;
    Float lcJacobian;
};

void Init(UniPathState &pathState) {
    pathState.throughput = Vector3(Float(1.0), Float(1.0), Float(1.0));
    pathState.lensThroughput = Vector3(Float(1.0), Float(1.0), Float(1.0));
    pathState.lastBsdfPdf = Float(1.0);
    pathState.ssJacobian = Float(1.0);
}

static inline bool Intersect(const Scene *scene,
                             const Float time,
                             const RaySegment &raySeg,
                             ShapeInst &shapeInst,
                             Intersection &isect) {
    bool hit = false;
    if (Intersect(scene, time, raySeg, shapeInst)) {
        if (shapeInst.obj->Intersect(shapeInst.primID, time, raySeg, isect, shapeInst.st)) {
            hit = true;
        }
    }
    return hit;
}

static inline const Light *GetHitLight(const Scene *scene,
                                       const bool hitSurface,
                                       const Shape *shape) {
    const Light *light = nullptr;
    if (!hitSurface) {
        if (scene->envLight != nullptr) {
            light = scene->envLight.get();
        }
    }
    if (hitSurface) {
        if (shape->areaLight != nullptr) {
            light = shape->areaLight;
        }
    }
    return light;
}

static void HandleHitLight(const int camDepth,
                           const Scene *scene,
                           const Light *light,
                           const bool hitSurface,
                           UniPathState &pathState,
                           const Float time,
                           const Vector2 screenPos,
                           LightInst &envLightInst,
                           std::vector<SubpathContrib> &contribs) {
    const Ray &ray = pathState.raySeg.ray;
    const Intersection &isect = pathState.isect;
    const Vector3 &throughput = pathState.throughput;
    const Float lastBsdfPdf = pathState.lastBsdfPdf;

    LightPrimID lPrimID;
    Vector3 emission;
    Float directPdf;
    Float emissionPdf;
    light->Emission(scene->bSphere,
                    ray.dir,
                    isect.shadingNormal,
                    time,
                    lPrimID,
                    emission,
                    directPdf,
                    emissionPdf);

    if (emission.sum() > Float(0.0)) {
        if (hitSurface) {
            Float distSq = DistanceSquared(ray.org, isect.position);
            Float cosTheta = -Dot(ray.dir, isect.shadingNormal);
            directPdf *= (distSq / cosTheta);
        }
        Vector3 contrib = throughput.cwiseProduct(emission);
        Float misWeight = Float(1.0);
        if (camDepth > 0) {
            Float lightPickProb = PickLightProb(scene, light);
            misWeight = MISWeight(lastBsdfPdf, directPdf * lightPickProb);
            contrib *= misWeight;
        }
        const Float score = Luminance(contrib);
        if (score > Float(0.0)) {
            if (!hitSurface) {
                envLightInst = LightInst{light, lPrimID};
            }
            const Float lensScore =
                camDepth >= 2 ? Luminance(pathState.lensThroughput) : Float(0.0);
            assert(std::isfinite(score * pathState.ssJacobian));
            assert(std::isfinite(lensScore));
            contribs.emplace_back(SubpathContrib{
                2 + camDepth,                  // camDepth
                0,                             // lightDepth
                screenPos,                     // screenPos
                contrib,                       // contrib
                score,                         // lsScore
                score * pathState.ssJacobian,  // ssScore
                lensScore,                     // lensScore
                misWeight                      // misWeight
            });
        }
    }
}

static inline void DirectLightingInit(const Scene *scene,
                                      SurfaceVertex &surfaceVertex,
                                      Float &lightPickProb,
                                      RNG &rng) {
    std::uniform_real_distribution<Float> uniDist(Float(0.0), Float(1.0));
    const Light *dirLight = PickLight(scene, uniDist(rng), lightPickProb);
    surfaceVertex.directLightRndParam = Vector2(uniDist(rng), uniDist(rng));
    surfaceVertex.directLightInst.light = dirLight;
    surfaceVertex.directLightInst.lPrimID = dirLight->SampleDiscrete(uniDist(rng));
}

static void DirectLighting(const int camDepth,
                           const Scene *scene,
                           const UniPathState &pathState,
                           const Float time,
                           const Vector2 screenPos,
                           const Float lightPickProb,
                           const bool doOcclusion,
                           SurfaceVertex &surfVertex,
                           std::vector<SubpathContrib> &contribs) {
    const ShapeInst &shapeInst = surfVertex.shapeInst;
    const BSDF *bsdf = shapeInst.obj->bsdf.get();
    const Intersection &isect = pathState.isect;
    const Vector3 &throughput = pathState.throughput;
    const Vector3 &wi = pathState.wi;

    LightInst &lightInst = surfVertex.directLightInst;
    const Light *light = lightInst.light;

    LightPrimID &lPrimID = lightInst.lPrimID;
    Vector3 dirToLight;
    Float distToLight;
    Vector3 lightContrib;
    Float cosAtLight;
    Float directPdf;
    Float emissionPdf;
    if (!light->SampleDirect(scene->bSphere,
                             isect.position,
                             isect.shadingNormal,
                             surfVertex.directLightRndParam,
                             time,
                             lPrimID,
                             dirToLight,
                             distToLight,
                             lightContrib,
                             cosAtLight,
                             directPdf,
                             emissionPdf)) {
        return;
    }

    if (doOcclusion && Occluded(scene, time, Ray{isect.position, dirToLight}, distToLight)) {
        return;
    }

    Vector3 bsdfContrib;
    Float cosWo, bsdfPdf, bsdfRevPdf;
    bsdf->Evaluate(
        wi, isect.shadingNormal, dirToLight, shapeInst.st, bsdfContrib, cosWo, bsdfPdf, bsdfRevPdf);
    if (bsdfContrib.isZero()) {
        return;
    }

    Vector3 lensThroughput = pathState.lensThroughput;
    if (camDepth == 1) {
        bool useAbsoluteParam = bsdf->Roughness(shapeInst.st, surfVertex.bsdfDiscrete) >
                                scene->options->roughnessThreshold;
        if (useAbsoluteParam) {
            Vector3 impBsdfContrib;
            Float impCosWo, impBsdfPdf, impBsdfRevPdf;
            bsdf->EvaluateAdjoint(dirToLight,
                                  isect.shadingNormal,
                                  wi,
                                  shapeInst.st,
                                  impBsdfContrib,
                                  impCosWo,
                                  impBsdfPdf,
                                  impBsdfRevPdf);
            const Float factor = ShadingNormalCorrection<true>(dirToLight, isect, wi);
            impBsdfContrib *= factor;
            lensThroughput = lensThroughput.cwiseProduct(impBsdfContrib);
        } else {
            lensThroughput = Vector3::Zero();
        }
    }

    Vector3 contrib = throughput.cwiseProduct(bsdfContrib);
    contrib = contrib.cwiseProduct(lightContrib) * inverse(lightPickProb);
    Float misWeight = Float(1.0);
    if (!light->IsDelta()) {
        misWeight = MISWeight(directPdf * lightPickProb, bsdfPdf);
        contrib *= misWeight;
    }

    const Float score = Luminance(contrib);
    if (score > Float(0.0)) {
        const Float lensScore = camDepth >= 1 ? Luminance(lensThroughput) : Float(0.0);
        assert(std::isfinite(score * pathState.ssJacobian));
        assert(std::isfinite(lensScore));
        contribs.emplace_back(SubpathContrib{
            2 + camDepth,                  // camDepth
            1,                             // lightDepth
            screenPos,                     // screenPos
            contrib,                       // contrib
            score,                         // lsScore
            score * pathState.ssJacobian,  // ssScore
            lensScore,                     // lensScore
            misWeight                      // misWeight
        });
    }
}

template <bool perturb = false>
static bool BSDFSampling(const int camDepth,
                         const Float roughnessThreshold,
                         UniPathState &pathState,
                         SurfaceVertex &surfVertex,
                         Vector3 &bsdfContrib) {
    const ShapeInst &shapeInst = surfVertex.shapeInst;
    const BSDF *bsdf = shapeInst.obj->bsdf.get();
    const Vector3 &wi = pathState.wi;
    const Intersection &isect = pathState.isect;
    Ray &ray = pathState.raySeg.ray;
    Float &bsdfPdf = pathState.lastBsdfPdf;
    Vector3 &throughput = pathState.throughput;

    Float cosWo;
    Float bsdfPdfRev;
    surfVertex.useAbsoluteParam =
        BoolToFloat(bsdf->Roughness(shapeInst.st, surfVertex.bsdfDiscrete) > roughnessThreshold);
    if (!perturb || surfVertex.useAbsoluteParam == FFALSE) {
        if (!bsdf->Sample(wi,
                          isect.shadingNormal,
                          shapeInst.st,
                          surfVertex.bsdfRndParam,
                          surfVertex.bsdfDiscrete,
                          ray.dir,
                          bsdfContrib,
                          cosWo,
                          bsdfPdf,
                          bsdfPdfRev)) {
            return false;
        }
        if (surfVertex.useAbsoluteParam == FTRUE) {
            Float jacobian;
            surfVertex.bsdfRndParam = ToSphericalCoord(ray.dir, jacobian);
            pathState.lcJacobian = inverse(jacobian);
            jacobian *= bsdfPdf;
            pathState.ssJacobian *= jacobian;
        } else {
            pathState.lcJacobian = bsdfPdf;
        }
    } else {
        Float jacobian;
        ray.dir = SampleSphere(surfVertex.bsdfRndParam, jacobian);
        bsdf->Evaluate(wi,
                       isect.shadingNormal,
                       ray.dir,
                       shapeInst.st,
                       bsdfContrib,
                       cosWo,
                       bsdfPdf,
                       bsdfPdfRev);
        if (bsdfContrib.isZero() || bsdfPdf <= Float(0.0)) {
            return false;
        }

        bsdfContrib *= inverse(bsdfPdf);
        pathState.lcJacobian = inverse(jacobian);
        jacobian *= bsdfPdf;
        pathState.ssJacobian *= jacobian;
    }

    // lensThroughput computation
    if (surfVertex.useAbsoluteParam == FTRUE) {
        if (camDepth == 0) {
            pathState.lensThroughput = pathState.lensThroughput.cwiseProduct(bsdfContrib * bsdfPdf);
        } else if (camDepth == 1) {
            Vector3 impBsdfContrib;
            Float impCosWo, impBsdfPdf, impBsdfPdfRev;
            bsdf->EvaluateAdjoint(ray.dir,
                                  isect.shadingNormal,
                                  wi,
                                  shapeInst.st,
                                  impBsdfContrib,
                                  impCosWo,
                                  impBsdfPdf,
                                  impBsdfPdfRev);
            const Float factor = ShadingNormalCorrection<true>(ray.dir, isect, wi);
            impBsdfContrib *= factor;
            pathState.lensThroughput = pathState.lensThroughput.cwiseProduct(impBsdfContrib);
        }
    } else {  // surfVertex.useAbsoluteParam == FFALSE
        if (camDepth <= 1) {
            pathState.lensThroughput = Vector3::Zero();
        }
    }    

    throughput = throughput.cwiseProduct(bsdfContrib);
    ray.org = isect.position;
    return true;
}

// Probably better to do this before sampling ...
static bool RussianRoulette(
    const int depth, const Vector3 &bsdfContrib, Float &rrWeight, Vector3 &throughput, RNG &rng) {
    std::uniform_real_distribution<Float> uniDist(Float(0.0), Float(1.0));
    Float rrProb = Float(1.0);
    if (depth >= 3) {
        rrProb = std::min(bsdfContrib.maxCoeff(), Float(0.95));
    }

    if (uniDist(rng) > rrProb) {
        return false;
    }

    rrWeight = inverse(rrProb);
    throughput *= rrWeight;

    return true;
}
 
void GeneratePath(const Scene *scene,
                  const Vector2i screenPosi,
                  const int minDepth,
                  const int maxDepth,
                  Path &path,
                  std::vector<SubpathContrib> &contribs,
                  RNG &rng) {
    std::uniform_real_distribution<Float> uniDist(Float(0.0), Float(1.0));

    Float time = uniDist(rng);
    path.time = time;
    path.isSubpath = false;

    const Camera *camera = scene->camera.get();
    Vector2 screenPos = Vector2(
        screenPosi[0] == -1 ? uniDist(rng)
                            : ((screenPosi[0] + uniDist(rng)) / Float(GetPixelWidth(camera))),
        screenPosi[1] == -1 ? uniDist(rng)
                            : ((screenPosi[1] + uniDist(rng)) / Float(GetPixelHeight(camera))));
    path.camVertex = CameraVertex{screenPos};

    UniPathState pathState;
    Init(pathState);
    SamplePrimary(camera, screenPos, time, pathState.raySeg);
    for (int camDepth = 0;; camDepth++) {
        path.camSurfaceVertex.push_back(SurfaceVertex());
        SurfaceVertex &surfVertex = path.camSurfaceVertex.back();
        bool hitSurface =
            Intersect(scene, time, pathState.raySeg, surfVertex.shapeInst, pathState.isect);

        const Light *light = GetHitLight(scene, hitSurface, surfVertex.shapeInst.obj);
        if (light != nullptr) {
            if (camDepth + 1 >= minDepth) {
                if (scene->options->useLightCoordinateSampling && camDepth > 1 &&
                    light->GetType() == LightType::AreaLight) {
                    assert(hitSurface);
                    SurfaceVertex &prevSurfVertex =
                        path.camSurfaceVertex[path.camSurfaceVertex.size() - 2];
                    // Special case for area light: transform the BSDF sampling coordinates to the
                    // light's direct sampling coordinates
                    // Note that we don't handle pure Dirac surfaces here, to do this we need
                    // something like Manifold exploration (and it's derivatives)
                    const ShapeInst &shapeInst = surfVertex.shapeInst;
                    prevSurfVertex.bsdfRndParam = shapeInst.obj->GetSampleParam(
                        shapeInst.primID, pathState.isect.position, path.time);
                    Vector3 dirToPrev = pathState.isect.position - pathState.raySeg.ray.org;
                    const Float distSq = LengthSquared(dirToPrev);
                    const Float invDistSq = inverse(distSq);
                    const Float invDist = sqrt(invDistSq);
                    dirToPrev *= invDist;
                    // Correct the jacobian since we've changed the sampling method
                    pathState.ssJacobian *=
                        fabs(Dot(dirToPrev, pathState.isect.shadingNormal) * invDistSq) *
                        (pathState.lcJacobian / surfVertex.shapeInst.obj->SamplePdf());
                }
                HandleHitLight(camDepth,
                               scene,
                               light,
                               hitSurface,
                               pathState,
                               time,
                               screenPos,
                               path.envLightInst,
                               contribs);
                // Assume lights have zero reflectance, makes light coordinates sampling easier to
                // handle
                return;
            }
        }

        if (!hitSurface || (maxDepth != -1 && camDepth + 1 >= maxDepth)) {
            break;
        }

        surfVertex.bsdfDiscrete = uniDist(rng);

        if (camDepth == 1) {
            path.lensVertexPos = pathState.isect.position;
            const Float distSq =
                DistanceSquared(pathState.isect.position, pathState.raySeg.ray.org);
            if (distSq <= Float(0.0)) {
                contribs.clear();
                return;
            } else {
                pathState.lensThroughput *= inverse(distSq);
            }
        }

        pathState.wi = -pathState.raySeg.ray.dir;

        if (camDepth + 2 >= minDepth) {
            Float directLightPickProb = Float(1.0);
            DirectLightingInit(scene, surfVertex, directLightPickProb, rng);
            DirectLighting(camDepth,
                           scene,
                           pathState,
                           time,
                           screenPos,
                           directLightPickProb,
                           true,
                           surfVertex,
                           contribs);
        }

        surfVertex.bsdfRndParam = Vector2(uniDist(rng), uniDist(rng));

        Vector3 bsdfContrib;

        if (!BSDFSampling(
                camDepth, scene->options->roughnessThreshold, pathState, surfVertex, bsdfContrib)) {
            break;
        }

        if (!RussianRoulette(
                camDepth, bsdfContrib, surfVertex.rrWeight, pathState.throughput, rng)) {
            break;
        }

        pathState.raySeg.minT = c_IsectEpsilon;
        pathState.raySeg.maxT = std::numeric_limits<Float>::infinity();
    }
}

struct BidirPathState {
    Intersection isect;
    Vector3 wi;
    Float accMISWPrev;
    Float accMISWThis;
    Vector3 throughput;
    // For light subpath, we store the bsdf contribution in lensContrib
    Vector3 lensContrib;
    Float ssJacobian;
    Float lcJacobian;
    Float lastBsdfPdf;
};

static inline void EmitFromCameraInit(const Camera *camera,
                                      const Vector2i screenPosi,
                                      CameraVertex &camVertex,
                                      RNG &rng) {
    std::uniform_real_distribution<Float> uniDist(Float(0.0), Float(1.0));
    camVertex.screenPos = Vector2(
        screenPosi[0] == -1 ? uniDist(rng)
                            : ((screenPosi[0] + uniDist(rng)) / Float(GetPixelWidth(camera))),
        screenPosi[1] == -1 ? uniDist(rng)
                            : ((screenPosi[1] + uniDist(rng)) / Float(GetPixelHeight(camera))));
}

static void EmitFromCamera(const Float time,
                           const Camera *camera,
                           const CameraVertex &camVertex,
                           RaySegment &raySeg,
                           BidirPathState &pathState) {
    RaySegment centerRaySeg;
    SamplePrimary(camera, Vector2(Float(0.5), Float(0.5)), time, centerRaySeg);
    SamplePrimary(camera, camVertex.screenPos, time, raySeg);
    const Vector3 camDir = centerRaySeg.ray.dir;
    const Vector3 dir = raySeg.ray.dir;
    const Float cosAtCamera = Dot(camDir, dir);
    const Float imagePointToCameraDist = camera->dist / cosAtCamera;
    const Float imageToSolidAngleFactor = square(imagePointToCameraDist) / cosAtCamera;
    const Float cameraPdfW = imageToSolidAngleFactor;
    const Float screenPixelCount = Float(GetPixelWidth(camera) * GetPixelHeight(camera));
    pathState.throughput = Vector3(Float(1.0), Float(1.0), Float(1.0));
    pathState.accMISWPrev = MIS(screenPixelCount / cameraPdfW);
    pathState.accMISWThis = Float(0.0);
    pathState.ssJacobian = Float(1.0);
    pathState.lensContrib = Vector3(Float(1.0), Float(1.0), Float(1.0));
}

static inline void EmitFromLightInit(const Scene *scene,
                                     LightVertex &lgtVertex,
                                     Float &lightPickProb,
                                     RNG &rng) {
    std::uniform_real_distribution<Float> uniDist(Float(0.0), Float(1.0));
    lgtVertex.rndParamPos = Vector2(uniDist(rng), uniDist(rng));
    lgtVertex.rndParamDir = Vector2(uniDist(rng), uniDist(rng));
    const Light *light = PickLight(scene, uniDist(rng), lightPickProb);
    lgtVertex.lightInst.light = light;
    lgtVertex.lightInst.lPrimID = light->SampleDiscrete(uniDist(rng));
}

static void EmitFromLight(const BSphere &bSphere,
                          const Float lightPickProb,
                          const Float time,
                          LightVertex &lgtVertex,
                          Ray &ray,
                          BidirPathState &pathState) {
    const Light *light = lgtVertex.lightInst.light;
    Float cosLight;
    Float emissionPdf;
    Float directPdf;
    light->Emit(bSphere,
                lgtVertex.rndParamPos,
                lgtVertex.rndParamDir,
                time,
                lgtVertex.lightInst.lPrimID,
                ray,
                pathState.throughput,
                cosLight,
                emissionPdf,
                directPdf);
    emissionPdf *= lightPickProb;
    directPdf *= lightPickProb;
    pathState.throughput *= inverse(lightPickProb);
    pathState.accMISWPrev = MIS(directPdf / emissionPdf);
    if (!light->IsDelta()) {
        pathState.accMISWThis = MIS(cosLight / emissionPdf);
    } else {
        pathState.accMISWThis = Float(0.0);
    }
    pathState.ssJacobian = Float(1.0);
}

static inline void ConvertMIS(const int depth,
                              const Light *light,
                              const Ray &ray,
                              BidirPathState &pathState) {
    if (depth > 0 || (light == nullptr) || (light != nullptr && light->IsFinite())) {
        pathState.accMISWPrev *= MIS(DistanceSquared(ray.org, pathState.isect.position));
    }

    Float invCosTheta = inverse(MIS(fabs(Dot(ray.dir, pathState.isect.shadingNormal))));
    pathState.accMISWPrev *= invCosTheta;
    pathState.accMISWThis *= invCosTheta;
}

static void ConnectToCamera(const int lgtDepth,
                            const Scene *scene,
                            const Camera *camera,
                            const Float time,
                            const BidirPathState &pathState,
                            const SurfaceVertex &lgtVertex,
                            const Vector3 &prevLensContrib,
                            const Vector3 &prevPosition,
                            std::vector<SubpathContrib> &contribs) {
    RaySegment centerRaySeg;
    SamplePrimary(camera, Vector2(Float(0.5), Float(0.5)), time, centerRaySeg);
    const Vector3 camOrg = centerRaySeg.ray.org;
    const Vector3 camDir = centerRaySeg.ray.dir;
    Vector3 dirToCamera = camOrg - pathState.isect.position;
    // Check point is in front of camera
    if (-Dot(camDir, dirToCamera) <= Float(0.0)) {
        return;
    }

    Vector2 screenPos;
    if (!ProjectPoint(camera, pathState.isect.position, time, screenPos)) {
        // If the point is outside of image plane
        return;
    }

    const Float distSq = LengthSquared(dirToCamera);
    const Float dist = sqrt(distSq);
    assert(dist > Float(0.0));
    dirToCamera *= inverse(dist);
    if (Occluded(scene, time, Ray{pathState.isect.position, dirToCamera}, dist)) {
        return;
    }

    const BSDF *bsdf = lgtVertex.shapeInst.obj->bsdf.get();

    Vector3 bsdfContrib;
    Float cosToCamera, bsdfPdf, bsdfRevPdf;
    bsdf->EvaluateAdjoint(pathState.wi,
                          pathState.isect.shadingNormal,
                          dirToCamera,
                          lgtVertex.shapeInst.st,
                          bsdfContrib,
                          cosToCamera,
                          bsdfPdf,
                          bsdfRevPdf);

    if (bsdfContrib.isZero()) {
        return;
    }

    const Float factor = ShadingNormalCorrection<true>(pathState.wi, pathState.isect, dirToCamera);
    if (factor <= Float(0.0)) {
        return;
    }
    bsdfContrib *= factor;

    bool useAbsoluteParam = bsdf->Roughness(lgtVertex.shapeInst.st, lgtVertex.bsdfDiscrete) >
                            scene->options->roughnessThreshold;
    Vector3 lensContrib = Vector3::Zero();
    if (useAbsoluteParam && lgtDepth >= 1) {
        Vector3 bsdfContrib;
        Float cosWo, bsdfPdf, bsdfRevPdf;
        bsdf->Evaluate(dirToCamera,
                       pathState.isect.shadingNormal,
                       pathState.wi,
                       lgtVertex.shapeInst.st,
                       bsdfContrib,
                       cosWo,
                       bsdfPdf,
                       bsdfRevPdf);

        const Float distSq = DistanceSquared(pathState.isect.position, prevPosition);
        if (distSq <= Float(0.0)) {
            contribs.clear();
            return;
        } else {
            lensContrib = bsdfContrib.cwiseProduct(prevLensContrib) * inverse(distSq);
        }
    }

    // Currently we ignore russian roulette in MIS computation, should still be unbiased
    // bsdfPdf *= rrProb;
    // bsdfRevPdf *= rrProb;

    // Compute pdf conversion factor from image plane area to surface area
    const Float cosAtCamera = -Dot(camDir, dirToCamera);
    const Float imagePointToCameraDist = camera->dist / cosAtCamera;
    const Float imageToSolidAngleFactor = square(imagePointToCameraDist) / cosAtCamera;
    const Float imageToSurfaceFactor = imageToSolidAngleFactor * fabs(cosToCamera) / distSq;
    const Float screenPixelCount = Float(GetPixelWidth(camera) * GetPixelHeight(camera));
    const Float cameraPdf = imageToSurfaceFactor;
    const Float wLight = MIS(cameraPdf / screenPixelCount) *
                         (pathState.accMISWPrev + pathState.accMISWThis * MIS(bsdfRevPdf));

    const Float misWeight = inverse(wLight + Float(1.0));
    const Float surfaceToImageFactor = cosToCamera / imageToSurfaceFactor;
    Vector3 contrib = misWeight * bsdfContrib / (screenPixelCount * surfaceToImageFactor);
    contrib = contrib.cwiseProduct(pathState.throughput);
    const Float score = Luminance(contrib);
    if (score > Float(0.0)) {
        const Float lensScore = Luminance(lensContrib);
        contribs.emplace_back(SubpathContrib{
            1,                             // camDepth
            2 + lgtDepth,                  // lightDepth
            screenPos,                     // screenPos
            contrib,                       // contrib
            score,                         // lsScore
            score * pathState.ssJacobian,  // ssScore
            lensScore,                     // lensScore
            misWeight                      // misWeight
        });
    }
}

template <bool adjoint, bool perturb = false>
static bool BSDFSampling(const Float roughnessThreshold,
                         const int depth,
                         const BidirPathState &pathState,
                         SurfaceVertex &surfVertex,
                         BidirPathState &nextPathState,
                         Vector3 &dir,
                         Vector3 &bsdfContrib) {
    const BSDF *bsdf = surfVertex.shapeInst.obj->bsdf.get();

    Float cosWo;
    Float bsdfPdf;
    Float bsdfRevPdf;
    surfVertex.useAbsoluteParam = BoolToFloat(
        bsdf->Roughness(surfVertex.shapeInst.st, surfVertex.bsdfDiscrete) > roughnessThreshold);
    if (!perturb || surfVertex.useAbsoluteParam == FFALSE) {
        if (adjoint) {
            if (!bsdf->SampleAdjoint(pathState.wi,
                                     pathState.isect.shadingNormal,
                                     surfVertex.shapeInst.st,
                                     surfVertex.bsdfRndParam,
                                     surfVertex.bsdfDiscrete,
                                     dir,
                                     bsdfContrib,
                                     cosWo,
                                     bsdfPdf,
                                     bsdfRevPdf)) {
                return false;
            }
        } else {
            if (!bsdf->Sample(pathState.wi,
                              pathState.isect.shadingNormal,
                              surfVertex.shapeInst.st,
                              surfVertex.bsdfRndParam,
                              surfVertex.bsdfDiscrete,
                              dir,
                              bsdfContrib,
                              cosWo,
                              bsdfPdf,
                              bsdfRevPdf)) {
                return false;
            }
        }
        if (surfVertex.useAbsoluteParam == FTRUE) {
            Float jacobian;
            surfVertex.bsdfRndParam = ToSphericalCoord(dir, jacobian);
            nextPathState.lcJacobian = inverse(jacobian);
            jacobian *= bsdfPdf;
            nextPathState.ssJacobian = pathState.ssJacobian * jacobian;
        } else {
            nextPathState.lcJacobian = bsdfPdf;
        }
    } else {
        Float jacobian;
        dir = SampleSphere(surfVertex.bsdfRndParam, jacobian);
        if (adjoint) {
            bsdf->EvaluateAdjoint(pathState.wi,
                                  pathState.isect.shadingNormal,
                                  dir,
                                  surfVertex.shapeInst.st,
                                  bsdfContrib,
                                  cosWo,
                                  bsdfPdf,
                                  bsdfRevPdf);
        } else {
            bsdf->Evaluate(pathState.wi,
                           pathState.isect.shadingNormal,
                           dir,
                           surfVertex.shapeInst.st,
                           bsdfContrib,
                           cosWo,
                           bsdfPdf,
                           bsdfRevPdf);
        }
        if (bsdfContrib.isZero() || bsdfPdf <= Float(0.0)) {
            return false;
        }
        bsdfContrib *= inverse(bsdfPdf);
        nextPathState.lcJacobian = inverse(jacobian);
        jacobian *= bsdfPdf;
        nextPathState.ssJacobian = pathState.ssJacobian * jacobian;
    }

    Float factor = ShadingNormalCorrection<adjoint>(pathState.wi, pathState.isect, dir);
    if (factor <= Float(0.0)) {
        return false;
    }

    // lensContrib computation
    if (surfVertex.useAbsoluteParam == FTRUE) {
        // Be very strict on grazing angle: lens perturbation sometimes produces
        // vertices on the same surfaces
        if (adjoint) {
            if (fabs(cosWo) < Float(1e-2) ||
                fabs(Dot(dir, pathState.isect.geomNormal)) < Float(1e-2)) {
                nextPathState.lensContrib = Vector3::Zero();
            } else {
                nextPathState.lensContrib = bsdfContrib * bsdfPdf;
            }
        } else if (depth == 0) {
            if (fabs(cosWo) < Float(1e-2) ||
                fabs(Dot(dir, pathState.isect.geomNormal)) < Float(1e-2)) {
                nextPathState.lensContrib = Vector3::Zero();
            } else {
                nextPathState.lensContrib =
                    pathState.lensContrib.cwiseProduct(bsdfContrib * bsdfPdf);
            }
        } else if (depth == 1) {
            Vector3 impBsdfContrib;
            Float impCosWo, impBsdfPdf, impBsdfPdfRev;
            bsdf->EvaluateAdjoint(dir,
                                  pathState.isect.shadingNormal,
                                  pathState.wi,
                                  surfVertex.shapeInst.st,
                                  impBsdfContrib,
                                  impCosWo,
                                  impBsdfPdf,
                                  impBsdfPdfRev);
            if (fabs(impCosWo) < Float(1e-2) ||
                fabs(Dot(pathState.wi, pathState.isect.geomNormal)) < Float(1e-2)) {
                nextPathState.lensContrib = Vector3::Zero();
            } else {
                const Float factor =
                    ShadingNormalCorrection<true>(dir, pathState.isect, pathState.wi);
                impBsdfContrib *= factor;
                nextPathState.lensContrib = pathState.lensContrib.cwiseProduct(impBsdfContrib);
            }
        }
    } else {
        assert(surfVertex.useAbsoluteParam == FFALSE);
        if (adjoint || depth <= 1) {
            nextPathState.lensContrib = Vector3::Zero();
        }
    }


    bsdfContrib *= factor;
    if (adjoint) {
        nextPathState.lensContrib *= factor;
    }
    
    // Currently we ignore russian roulette in MIS computation
    // (because it is tricky to get reverse probability correct), should still be unbiased
    // bsdfPdf *= rrProb;
    // bsdfRevPdf *= rrProb;
    
    nextPathState.lastBsdfPdf = bsdfPdf;

    nextPathState.accMISWThis =
        MIS(cosWo / bsdfPdf) * (pathState.accMISWThis * MIS(bsdfRevPdf) + pathState.accMISWPrev);
    nextPathState.accMISWPrev = MIS(inverse(bsdfPdf));
    nextPathState.throughput = pathState.throughput.cwiseProduct(bsdfContrib);
    return true;
}

static void HandleHitLight(const int camDepth,
                           const Scene *scene,
                           const Light *light,
                           const bool hitSurface,
                           const Ray &ray,
                           const Float time,
                           const Vector2 screenPos,
                           const BidirPathState &pathState,
                           const bool bidirMIS,
                           LightInst &envLightInst,
                           std::vector<SubpathContrib> &contribs) {
    LightPrimID lPrimID;
    Vector3 emission;
    Float directPdf;
    Float emissionPdf;
    light->Emission(scene->bSphere,
                    ray.dir,
                    pathState.isect.shadingNormal,
                    time,
                    lPrimID,
                    emission,
                    directPdf,
                    emissionPdf);
    if (emission.sum() > Float(0.0)) {
        Vector3 contrib = pathState.throughput.cwiseProduct(emission);
        Float misWeight = Float(1.0);
        if (camDepth > 0) {
            Float lightPickProb = PickLightProb(scene, light);
            directPdf *= lightPickProb;
            if (bidirMIS) {
                emissionPdf *= lightPickProb;
                // accMISWPrev magically accounts for measure conversion
                Float wCamera = MIS(directPdf) * pathState.accMISWPrev +
                                MIS(emissionPdf) * pathState.accMISWThis;
                misWeight = inverse(Float(1.0) + wCamera);
            } else {
                if (hitSurface) {
                    Float distSq = DistanceSquared(ray.org, pathState.isect.position);
                    Float cosTheta = -Dot(ray.dir, pathState.isect.shadingNormal);
                    directPdf *= (distSq / cosTheta);
                }
                misWeight = MISWeight(pathState.lastBsdfPdf, directPdf);
            }
            contrib *= misWeight;
        }
        Float score = Luminance(contrib);
        if (score > Float(0.0)) {
            if (!hitSurface) {
                envLightInst = LightInst{light, lPrimID};
            }
            const Float lensScore = camDepth >= 2 ? Luminance(pathState.lensContrib) : Float(0.0);
            assert(std::isfinite(score * pathState.ssJacobian));
            assert(std::isfinite(lensScore));
            contribs.emplace_back(SubpathContrib{
                2 + camDepth,                  // camDepth
                0,                             // lightDepth
                screenPos,                     // screenPos
                contrib,                       // contrib
                score,                         // lsScore
                score * pathState.ssJacobian,  // ssScore
                lensScore,                     // lensScore
                misWeight                      // misWeight
            });
        }
    }
}

static void DirectLighting(const int camDepth,
                           const Scene *scene,
                           const Float time,
                           const BidirPathState &pathState,
                           const Vector2 screenPos,
                           const Float lightPickProb,
                           SurfaceVertex &camVertex,
                           const bool doOcclusion,
                           const bool bidirMIS,
                           std::vector<SubpathContrib> &contribs) {
    const BSDF *bsdf = camVertex.shapeInst.obj->bsdf.get();
    LightInst &dirLightInst = camVertex.directLightInst;
    const Light *light = dirLightInst.light;
    LightPrimID &lPrimID = dirLightInst.lPrimID;
    Vector3 dirToLight;
    Float dist;
    Vector3 lightContrib;
    Float cosAtLight;
    Float directPdf;
    Float emissionPdf;
    if (!light->SampleDirect(scene->bSphere,
                             pathState.isect.position,
                             pathState.isect.shadingNormal,
                             camVertex.directLightRndParam,
                             time,
                             lPrimID,
                             dirToLight,
                             dist,
                             lightContrib,
                             cosAtLight,
                             directPdf,
                             emissionPdf)) {
        return;
    }

    if (doOcclusion && Occluded(scene, time, Ray{pathState.isect.position, dirToLight}, dist)) {
        return;
    }

    Vector3 bsdfContrib;
    Float cosToLight;
    Float bsdfPdf;
    Float bsdfRevPdf;
    bsdf->Evaluate(pathState.wi,
                   pathState.isect.shadingNormal,
                   dirToLight,
                   camVertex.shapeInst.st,
                   bsdfContrib,
                   cosToLight,
                   bsdfPdf,
                   bsdfRevPdf);
    if (bsdfContrib.isZero()) {
        return;
    }
    const Float factor = ShadingNormalCorrection<false>(pathState.wi, pathState.isect, dirToLight);
    if (factor <= Float(0.0)) {
        return;
    }
    bsdfContrib *= factor;

    Vector3 lensContrib = pathState.lensContrib;
    if (camDepth == 1) {
        bool useAbsoluteParam = bsdf->Roughness(camVertex.shapeInst.st, camVertex.bsdfDiscrete) >
                                scene->options->roughnessThreshold;
        if (useAbsoluteParam) {
            const Intersection &isect = pathState.isect;
            Vector3 impBsdfContrib;
            Float impCosWo, impBsdfPdf, impBsdfRevPdf;
            bsdf->EvaluateAdjoint(dirToLight,
                                  isect.shadingNormal,
                                  pathState.wi,
                                  camVertex.shapeInst.st,
                                  impBsdfContrib,
                                  impCosWo,
                                  impBsdfPdf,
                                  impBsdfRevPdf);
            const Float factor =
                ShadingNormalCorrection<true>(dirToLight, pathState.isect, pathState.wi);
            impBsdfContrib *= factor;
            lensContrib = lensContrib.cwiseProduct(impBsdfContrib);
        } else {
            lensContrib = Vector3::Zero();
        }
    }

    // Currently we ignore russian roulette in MIS computation
    // (because it is tricky to get reverse probability correct), should still be unbiased
    // bsdfPdf *= rrProb;
    // bsdfRevPdf *= rrProb;

    Vector3 contrib = pathState.throughput.cwiseProduct(bsdfContrib);
    contrib = contrib.cwiseProduct(lightContrib) * inverse(lightPickProb);
    Float misWeight = Float(1.0);
    if (bidirMIS) {
        Float wLight = light->IsDelta() ? Float(0.0) : MIS(bsdfPdf / (lightPickProb * directPdf));
        Float wCamera = MIS(emissionPdf * cosToLight / (directPdf * cosAtLight)) *
                        (pathState.accMISWPrev + pathState.accMISWThis * MIS(bsdfRevPdf));
        misWeight = inverse(wLight + Float(1.0) + wCamera);
        contrib *= misWeight;
    } else if (!light->IsDelta()) {
        misWeight = MISWeight(directPdf * lightPickProb, bsdfPdf);
        contrib *= misWeight;
    }

    const Float score = Luminance(contrib);
    if (score > Float(0.0)) {
        const Float lensScore = camDepth >= 1 ? Luminance(lensContrib) : Float(0.0);
        assert(std::isfinite(score * pathState.ssJacobian));
        assert(std::isfinite(lensScore));
        contribs.emplace_back(SubpathContrib{
            2 + camDepth,                  // camDepth
            1,                             // lightDepth
            screenPos,                     // screenPos
            contrib,                       // contrib
            score,                         // lsScore
            score * pathState.ssJacobian,  // ssScore
            lensScore,                     // lensScore
            misWeight                      // misWeight
        });
    }
}

static void ConnectVertex(const int camDepth,
                          const int lgtDepth,
                          const Scene *scene,
                          const Float time,
                          const BidirPathState &lgtPathState,
                          const SurfaceVertex &lgtVertex,
                          const BidirPathState &camPathState,
                          const SurfaceVertex &camVertex,
                          const Vector2 screenPos,
                          const bool doOcclusion,
                          std::vector<SubpathContrib> &contribs) {
    Vector3 dirToLight = lgtPathState.isect.position - camPathState.isect.position;
    const Float distSq = LengthSquared(dirToLight);
    const Float dist = sqrt(distSq);
    assert(dist > Float(0.0));
    dirToLight *= inverse(dist);

    if (doOcclusion && Occluded(scene, time, Ray{camPathState.isect.position, dirToLight}, dist)) {
        return;
    }

    Vector3 camBsdfFactor;
    Float cosCamera, camBsdfPdf, camBsdfRevPdf;
    const BSDF *camBSDF = camVertex.shapeInst.obj->bsdf.get();
    camBSDF->Evaluate(camPathState.wi,
                      camPathState.isect.shadingNormal,
                      dirToLight,
                      camVertex.shapeInst.st,
                      camBsdfFactor,
                      cosCamera,
                      camBsdfPdf,
                      camBsdfRevPdf);

    if (camBsdfFactor.isZero()) {
        return;
    }

    Float camFactor =
        ShadingNormalCorrection<false>(camPathState.wi, camPathState.isect, dirToLight);
    if (camFactor <= Float(0.0)) {
        return;
    }
    camBsdfFactor *= camFactor;

    // Currently we ignore russian roulette in MIS computation, should still be unbiased
    // camBsdfPdf *= rrProb;
    // camBsdfRevPdf *= rrProb;

    Vector3 lgtBsdfFactor;
    Float cosLight, lgtBsdfPdf, lgtBsdfRevPdf;
    const BSDF *lgtBSDF = lgtVertex.shapeInst.obj->bsdf.get();
    lgtBSDF->EvaluateAdjoint(lgtPathState.wi,
                             lgtPathState.isect.shadingNormal,
                             -dirToLight,
                             lgtVertex.shapeInst.st,
                             lgtBsdfFactor,
                             cosLight,
                             lgtBsdfPdf,
                             lgtBsdfRevPdf);

    if (lgtBsdfFactor.isZero()) {
        return;
    }

    Float lgtFactor =
        ShadingNormalCorrection<true>(lgtPathState.wi, lgtPathState.isect, -dirToLight);
    if (lgtFactor <= Float(0.0)) {
        return;
    }
    lgtBsdfFactor *= lgtFactor;

    const Float geometryTerm = inverse(distSq);

    Vector3 lensContrib = camPathState.lensContrib;
    if (camDepth == 0) {
        bool camUseAbsoluteParam =
            camBSDF->Roughness(camVertex.shapeInst.st, camVertex.bsdfDiscrete) >
            scene->options->roughnessThreshold;
        bool lgtUseAbsoluteParam =
            lgtBSDF->Roughness(lgtVertex.shapeInst.st, lgtVertex.bsdfDiscrete) >
            scene->options->roughnessThreshold;
        if (camUseAbsoluteParam && lgtUseAbsoluteParam) {
            lensContrib = lgtBsdfFactor.cwiseProduct(camBsdfFactor) * geometryTerm;
        } else {
            lensContrib = Vector3::Zero();
        }
    } else if (camDepth == 1) {
        bool camUseAbsoluteParam =
            camBSDF->Roughness(camVertex.shapeInst.st, camVertex.bsdfDiscrete) >
            scene->options->roughnessThreshold;
        if (camUseAbsoluteParam) {
            const Intersection &isect = camPathState.isect;
            Vector3 impBsdfContrib;
            Float impCosWo, impBsdfPdf, impBsdfRevPdf;
            camBSDF->EvaluateAdjoint(dirToLight,
                                     isect.shadingNormal,
                                     camPathState.wi,
                                     camVertex.shapeInst.st,
                                     impBsdfContrib,
                                     impCosWo,
                                     impBsdfPdf,
                                     impBsdfRevPdf);
            const Float factor = ShadingNormalCorrection<true>(dirToLight, isect, camPathState.wi);
            impBsdfContrib *= factor;
            lensContrib = lensContrib.cwiseProduct(impBsdfContrib);
        } else {
            lensContrib = Vector3::Zero();
        }
    }

    // Currently we ignore russian roulette in MIS computation
    // (because it is tricky to get reverse probability correct), should still be unbiased
    // lgtBsdfPdf *= rrProb;
    // lgtBsdfRevPdf *= rrProb;

    // Convert pdfs to area pdf
    const Float camBsdfDirPdfA = camBsdfPdf * cosLight * geometryTerm;
    const Float lgtBsdfDirPdfA = lgtBsdfPdf * cosCamera * geometryTerm;

    const Float wLight = MIS(camBsdfDirPdfA) *
                         (lgtPathState.accMISWPrev + lgtPathState.accMISWThis * MIS(lgtBsdfRevPdf));
    const Float wCamera = MIS(lgtBsdfDirPdfA) * (camPathState.accMISWPrev +
                                                 camPathState.accMISWThis * MIS(camBsdfRevPdf));
    const Float misWeight = inverse(wLight + Float(1.0) + wCamera);

    const Vector3 throughput = lgtPathState.throughput.cwiseProduct(camPathState.throughput);
    Vector3 contrib = throughput.cwiseProduct(camBsdfFactor);
    contrib = contrib.cwiseProduct(lgtBsdfFactor) * geometryTerm;
    contrib *= misWeight;
    const Float ssJacobian = lgtPathState.ssJacobian * camPathState.ssJacobian;
    const Float score = Luminance(contrib);
    if (score > Float(0.0)) {
        const Float lensScore = Luminance(lensContrib);
        contribs.emplace_back(SubpathContrib{
            2 + camDepth,        // camDepth
            2 + lgtDepth,        // lightDepth
            screenPos,           // screenPos
            contrib,             // contrib
            score,               // lsScore
            score * ssJacobian,  // ssScore
            lensScore,           // lensScore
            misWeight            // misWeight
        });
    }
}

void GeneratePathBidir(const Scene *scene,
                       const Vector2i screenPosi,
                       const int minDepth,
                       const int maxDepth,
                       Path &path,
                       std::vector<SubpathContrib> &contribs,
                       RNG &rng) {
    std::uniform_real_distribution<Float> uniDist(Float(0.0), Float(1.0));
    const Camera *camera = scene->camera.get();

    path.time = uniDist(rng);
    std::vector<BidirPathState> lightPathStates;
    lightPathStates.push_back(BidirPathState());
    Float lightPickProb = Float(1.0);
    EmitFromLightInit(scene, path.lgtVertex, lightPickProb, rng);
    RaySegment raySeg;
    EmitFromLight(
        scene->bSphere, lightPickProb, path.time, path.lgtVertex, raySeg.ray, lightPathStates[0]);
    raySeg.minT = c_IsectEpsilon;
    raySeg.maxT = std::numeric_limits<Float>::infinity();
    Vector3 prevLensContrib = Vector3::Zero();
    for (int lgtDepth = 0;; lgtDepth++) {
        path.lgtSurfaceVertex.push_back(SurfaceVertex());
        SurfaceVertex &surfVertex = path.lgtSurfaceVertex.back();
        bool hitSurface = Intersect(
            scene, path.time, raySeg, surfVertex.shapeInst, lightPathStates[lgtDepth].isect);

        // If we do full BDPT with non-pinhole cameras we need to handle camera hit here

        if (!hitSurface) {
            lightPathStates.pop_back();
            path.lgtSurfaceVertex.pop_back();
            break;
        }

        surfVertex.bsdfDiscrete = uniDist(rng);
        lightPathStates[lgtDepth].wi = -raySeg.ray.dir;

        ConvertMIS(lgtDepth, path.lgtVertex.lightInst.light, raySeg.ray, lightPathStates[lgtDepth]);

        if (lgtDepth + 2 >= minDepth) {
            ConnectToCamera(lgtDepth,
                            scene,
                            camera,
                            path.time,
                            lightPathStates[lgtDepth],
                            path.lgtSurfaceVertex[lgtDepth],
                            prevLensContrib,
                            raySeg.ray.org,
                            contribs);
        }

        if (maxDepth != -1 && lgtDepth + 2 >= maxDepth) {
            break;
        }

        lightPathStates.push_back(BidirPathState());
        surfVertex.bsdfRndParam = Vector2(uniDist(rng), uniDist(rng));
        Vector3 bsdfContrib;
        if (!BSDFSampling<true>(scene->options->roughnessThreshold,
                                lgtDepth,
                                lightPathStates[lgtDepth],
                                surfVertex,
                                lightPathStates[lgtDepth + 1],
                                raySeg.ray.dir,
                                bsdfContrib)) {
            lightPathStates.pop_back();
            break;
        }

        if (!RussianRoulette(lgtDepth,
                             bsdfContrib,
                             surfVertex.rrWeight,
                             lightPathStates[lgtDepth + 1].throughput,
                             rng)) {
            lightPathStates.pop_back();
            break;
        }

        prevLensContrib = lightPathStates[lgtDepth + 1].lensContrib;
        raySeg.ray.org = lightPathStates[lgtDepth].isect.position;
    }

    BidirPathState camPathState;
    EmitFromCameraInit(camera, screenPosi, path.camVertex, rng);
    EmitFromCamera(path.time, camera, path.camVertex, raySeg, camPathState);

    for (int camDepth = 0;; camDepth++) {
        path.camSurfaceVertex.push_back(SurfaceVertex());
        SurfaceVertex &surfVertex = path.camSurfaceVertex.back();
        bool hitSurface =
            Intersect(scene, path.time, raySeg, surfVertex.shapeInst, camPathState.isect);

        camPathState.wi = -raySeg.ray.dir;

        if (hitSurface) {
            ConvertMIS(camDepth, nullptr, raySeg.ray, camPathState);
        }

        if (camDepth + 1 >= minDepth) {
            const Light *light = GetHitLight(scene, hitSurface, surfVertex.shapeInst.obj);
            if (light != nullptr) {
                if (scene->options->useLightCoordinateSampling && camDepth > 1 &&
                    light->GetType() == LightType::AreaLight) {
                    assert(hitSurface);
                    SurfaceVertex &prevSurfVertex =
                        path.camSurfaceVertex[path.camSurfaceVertex.size() - 2];
                    // Special case for area light: transform the BSDF sampling coordinates to the
                    // light's direct sampling coordinates
                    // Note that we don't handle pure Dirac surfaces here, to do this we need
                    // something like Manifold exploration (and it's derivatives)
                    const ShapeInst &shapeInst = surfVertex.shapeInst;
                    prevSurfVertex.bsdfRndParam = shapeInst.obj->GetSampleParam(
                        shapeInst.primID, camPathState.isect.position, path.time);
                    // Correct the jacobian since we've changed the sampling method
                    Vector3 dirToPrev = camPathState.isect.position - raySeg.ray.org;
                    const Float distSq = LengthSquared(dirToPrev);
                    const Float invDistSq = inverse(distSq);
                    const Float invDist = sqrt(invDistSq);
                    dirToPrev *= invDist;
                    camPathState.ssJacobian *=
                        fabs(Dot(dirToPrev, camPathState.isect.shadingNormal) * invDistSq) *
                        (camPathState.lcJacobian * surfVertex.shapeInst.obj->SamplePdf());
                }
                HandleHitLight(camDepth,
                               scene,
                               light,
                               hitSurface,
                               raySeg.ray,
                               path.time,
                               path.camVertex.screenPos,
                               camPathState,
                               true,
                               path.envLightInst,
                               contribs);
                // Assume lights have zero reflectance
                return;
            }
        }

        if (!hitSurface || (maxDepth != -1 && camDepth + 1 >= maxDepth)) {
            break;
        }

        if (camDepth == 1) {
            path.lensVertexPos = camPathState.isect.position;
            const Float distSq = DistanceSquared(camPathState.isect.position, raySeg.ray.org);
            if (distSq <= Float(0.0)) {
                contribs.clear();
                return;
            } else {
                camPathState.lensContrib *= inverse(distSq);
            }
        }

        surfVertex.bsdfDiscrete = uniDist(rng);

        if (camDepth + 2 >= minDepth) {
            Float directLightPickProb = Float(1.0);
            DirectLightingInit(scene, surfVertex, directLightPickProb, rng);
            DirectLighting(camDepth,
                           scene,
                           path.time,
                           camPathState,
                           path.camVertex.screenPos,
                           directLightPickProb,
                           surfVertex,
                           true,
                           true,
                           contribs);
        }

        int maxLgtDepth =
            maxDepth == -1 ? ((int)lightPathStates.size() - 1)
                           : std::min((maxDepth - camDepth - 3), ((int)lightPathStates.size() - 1));
        for (int lgtDepth = 0; lgtDepth <= maxLgtDepth; lgtDepth++) {
            if (camDepth + lgtDepth + 3 >= minDepth) {
                ConnectVertex(camDepth,
                              lgtDepth,
                              scene,
                              path.time,
                              lightPathStates[lgtDepth],
                              path.lgtSurfaceVertex[lgtDepth],
                              camPathState,
                              surfVertex,
                              path.camVertex.screenPos,
                              true,
                              contribs);
            }
        }

        surfVertex.bsdfRndParam = Vector2(uniDist(rng), uniDist(rng));
        Vector3 bsdfContrib;
        if (!BSDFSampling<false>(scene->options->roughnessThreshold,
                                 camDepth,
                                 camPathState,
                                 surfVertex,
                                 camPathState,
                                 raySeg.ray.dir,
                                 bsdfContrib)) {
            break;
        }

        if (!RussianRoulette(
                camDepth, bsdfContrib, surfVertex.rrWeight, camPathState.throughput, rng)) {
            break;
        }

        raySeg.ray.org = camPathState.isect.position;
        raySeg.minT = c_IsectEpsilon;
        raySeg.maxT = std::numeric_limits<Float>::infinity();
    }
} 

void GenerateSubpath(const Scene *scene,
                     const Vector2i screenPosi,
                     const int camLength,
                     const int lgtLength,
                     const bool bidirMIS,
                     Path &path,
                     std::vector<SubpathContrib> &contribs,
                     RNG &rng) {
    std::uniform_real_distribution<Float> uniDist(Float(0.0), Float(1.0));
    const Camera *camera = scene->camera.get();

    path.time = uniDist(rng);
    RaySegment raySeg;

    BidirPathState lightPathState;
    if (lgtLength > 1) {
        Float lightPickProb = Float(1.0);
        EmitFromLightInit(scene, path.lgtVertex, lightPickProb, rng);
        EmitFromLight(
            scene->bSphere, lightPickProb, path.time, path.lgtVertex, raySeg.ray, lightPathState);
        raySeg.minT = c_IsectEpsilon;
        raySeg.maxT = std::numeric_limits<Float>::infinity();
        Vector3 prevLensContrib = Vector3::Zero();
        for (int lgtDepth = 0;; lgtDepth++) {
            path.lgtSurfaceVertex.push_back(SurfaceVertex());
            SurfaceVertex &surfVertex = path.lgtSurfaceVertex.back();
            bool hitSurface =
                Intersect(scene, path.time, raySeg, surfVertex.shapeInst, lightPathState.isect);

            // If we do full BDPT with non-pinhole cameras we need to handle camera hit here

            if (!hitSurface) {
                return;
            }

            surfVertex.bsdfDiscrete = uniDist(rng);
            lightPathState.wi = -raySeg.ray.dir;

            if (bidirMIS) {
                ConvertMIS(lgtDepth, path.lgtVertex.lightInst.light, raySeg.ray, lightPathState);
            }

            if (lgtDepth + 2 == lgtLength) {
                if (camLength == 1) {
                    ConnectToCamera(lgtDepth,
                                    scene,
                                    camera,
                                    path.time,
                                    lightPathState,
                                    path.lgtSurfaceVertex[lgtDepth],
                                    prevLensContrib,
                                    raySeg.ray.org,
                                    contribs);
                    return;
                }
                break;
            }

            surfVertex.bsdfRndParam = Vector2(uniDist(rng), uniDist(rng));
            Vector3 bsdfContrib;
            if (!BSDFSampling<true>(scene->options->roughnessThreshold,
                                    lgtDepth,
                                    lightPathState,
                                    surfVertex,
                                    lightPathState,
                                    raySeg.ray.dir,
                                    bsdfContrib)) {
                return;
            }

            // RussianRoulette(lgtDepth, bsdfContrib, surfVertex.rrWeight,
            // lightPathState.throughput);
            surfVertex.rrWeight = Float(1.0);

            prevLensContrib = lightPathState.lensContrib;
            raySeg.ray.org = lightPathState.isect.position;
        }
    }

    assert(camLength > 1);

    BidirPathState camPathState;
    EmitFromCameraInit(camera, screenPosi, path.camVertex, rng);
    EmitFromCamera(path.time, camera, path.camVertex, raySeg, camPathState);

    for (int camDepth = 0;; camDepth++) {
        path.camSurfaceVertex.push_back(SurfaceVertex());
        SurfaceVertex &surfVertex = path.camSurfaceVertex.back();
        bool hitSurface =
            Intersect(scene, path.time, raySeg, surfVertex.shapeInst, camPathState.isect);

        camPathState.wi = -raySeg.ray.dir;

        if (bidirMIS && hitSurface) {
            ConvertMIS(camDepth, nullptr, raySeg.ray, camPathState);
        }

        if (camDepth + 2 >= camLength && lgtLength == 0) {
            const Light *light = GetHitLight(scene, hitSurface, surfVertex.shapeInst.obj);
            if (light != nullptr) {
                if (camDepth > 1 && light->GetType() == LightType::AreaLight) {
                    assert(hitSurface);
                    SurfaceVertex &prevSurfVertex =
                        path.camSurfaceVertex[path.camSurfaceVertex.size() - 2];
                    // Special case for area light: transform the BSDF sampling coordinates to the
                    // light's direct sampling coordinates
                    // Note that we don't handle pure Dirac surfaces here, to do this we need
                    // something like Manifold exploration (and it's derivatives)
                    const ShapeInst &shapeInst = surfVertex.shapeInst;
                    prevSurfVertex.bsdfRndParam = shapeInst.obj->GetSampleParam(
                        shapeInst.primID, camPathState.isect.position, path.time);
                    // Correct the jacobian since we've changed the sampling method
                    Vector3 dirToPrev = camPathState.isect.position - raySeg.ray.org;
                    const Float distSq = LengthSquared(dirToPrev);
                    const Float invDistSq = inverse(distSq);
                    const Float invDist = sqrt(invDistSq);
                    dirToPrev *= invDist;
                    camPathState.ssJacobian *=
                        fabs(Dot(dirToPrev, camPathState.isect.shadingNormal) * invDistSq) *
                        (camPathState.lcJacobian * surfVertex.shapeInst.obj->SamplePdf());
                }
                HandleHitLight(camDepth,
                               scene,
                               light,
                               hitSurface,
                               raySeg.ray,
                               path.time,
                               path.camVertex.screenPos,
                               camPathState,
                               bidirMIS,
                               path.envLightInst,
                               contribs);
            }
            return;
        }

        if (!hitSurface) {
            return;
        }

        if (camDepth == 1) {
            path.lensVertexPos = camPathState.isect.position;
            const Float distSq = DistanceSquared(camPathState.isect.position, raySeg.ray.org);
            if (distSq <= Float(0.0)) {
                return;
            } else {
                camPathState.lensContrib *= inverse(distSq);
            }
        }

        surfVertex.bsdfDiscrete = uniDist(rng);

        if (camDepth + 2 == camLength) {
            assert(lgtLength >= 1);
            if (lgtLength == 1) {
                Float directLightPickProb = Float(1.0);
                DirectLightingInit(scene, surfVertex, directLightPickProb, rng);
                DirectLighting(camDepth,
                               scene,
                               path.time,
                               camPathState,
                               path.camVertex.screenPos,
                               directLightPickProb,
                               surfVertex,
                               true,
                               bidirMIS,
                               contribs);
            } else {
                assert(lgtLength >= 2 && int(path.lgtSurfaceVertex.size()) == lgtLength - 1);
                ConnectVertex(camDepth,
                              lgtLength - 2,
                              scene,
                              path.time,
                              lightPathState,
                              path.lgtSurfaceVertex.back(),
                              camPathState,
                              surfVertex,
                              path.camVertex.screenPos,
                              true,
                              contribs);
            }
            return;
        }

        surfVertex.bsdfRndParam = Vector2(uniDist(rng), uniDist(rng));

        Vector3 bsdfContrib;
        if (!BSDFSampling<false>(scene->options->roughnessThreshold,
                                 camDepth,
                                 camPathState,
                                 surfVertex,
                                 camPathState,
                                 raySeg.ray.dir,
                                 bsdfContrib)) {
            return;
        }

        // RussianRoulette(camDepth, bsdfContrib, surfVertex.rrWeight, camPathState.throughput);
        surfVertex.rrWeight = Float(1.0);

        raySeg.ray.org = camPathState.isect.position;
        raySeg.minT = c_IsectEpsilon;
        raySeg.maxT = std::numeric_limits<Float>::infinity();
    }

    // should never reach here
    assert(false);
}

void ToSubpath(const int camDepth, const int lgtDepth, Path &path) {
    path.camSurfaceVertex.resize(std::max(camDepth - 1, 0));
    path.lgtSurfaceVertex.resize(std::max(lgtDepth - 1, 0));
    if (lgtDepth != 0) {
        path.envLightInst.light = nullptr;
    }
    path.isSubpath = true;
    path.camDepth = camDepth;
    path.lgtDepth = lgtDepth;
}

inline void Perturb(Float &value, const Vector &offset, int &offsetId) {
    value = Modulo(value + offset[offsetId++], Float(1.0));
}

bool LightCoordinateSampling(const int camDepth,
                             const Scene *scene,
                             const Float time,
                             const SurfaceVertex &curSurfVertex,
                             const SurfaceVertex &nextSurfVertex,
                             const bool doOcclusion,
                             UniPathState &pathState,
                             Vector3 &bsdfContrib) {
    const ShapeInst &shapeInst = curSurfVertex.shapeInst;
    const ShapeInst &nextShapeInst = nextSurfVertex.shapeInst;
    const BSDF *bsdf = shapeInst.obj->bsdf.get();
    const Intersection &isect = pathState.isect;
    const Vector3 &wi = pathState.wi;
    Ray &ray = pathState.raySeg.ray;

    Vector3 nextPosition, nextNormal;
    assert(nextShapeInst.obj != nullptr);
    Float shapePdf = Float(0.0);
    nextShapeInst.obj->Sample(curSurfVertex.bsdfRndParam,
                              time,
                              nextSurfVertex.shapeInst.primID,
                              nextPosition,
                              nextNormal,
                              &shapePdf);
    assert(shapePdf > Float(0.0));

    ray.dir = nextPosition - isect.position;
    Float distToLightSq = LengthSquared(ray.dir);
    Float distToLight = sqrt(distToLightSq);
    ray.dir *= inverse(distToLight);

    if (doOcclusion && Occluded(scene, time, Ray{isect.position, ray.dir}, distToLight)) {
        return false;
    }

    Float cosWo, bsdfPdf, bsdfRevPdf;
    bsdf->Evaluate(
        wi, isect.shadingNormal, ray.dir, shapeInst.st, bsdfContrib, cosWo, bsdfPdf, bsdfRevPdf);
    if (bsdfContrib.isZero()) {
        return false;
    }

    bsdfContrib *= inverse(bsdfPdf);

    pathState.throughput = pathState.throughput.cwiseProduct(bsdfContrib);
    pathState.ssJacobian *=
        fabs(Dot(ray.dir, nextNormal) * inverse(distToLightSq)) * bsdfPdf / shapePdf;
    pathState.lastBsdfPdf = bsdfPdf;
    ray.org = isect.position;

    if (camDepth == 1) {
        bool useAbsoluteParam = bsdf->Roughness(shapeInst.st, curSurfVertex.bsdfDiscrete) >
                                scene->options->roughnessThreshold;
        if (useAbsoluteParam) {
            Vector3 impBsdfContrib;
            Float impCosWo, impBsdfPdf, impBsdfRevPdf;
            bsdf->EvaluateAdjoint(ray.dir,
                                  isect.shadingNormal,
                                  wi,
                                  shapeInst.st,
                                  impBsdfContrib,
                                  impCosWo,
                                  impBsdfPdf,
                                  impBsdfRevPdf);
            const Float factor = ShadingNormalCorrection<true>(ray.dir, isect, wi);
            impBsdfContrib *= factor;
            pathState.lensThroughput = pathState.lensThroughput.cwiseProduct(impBsdfContrib);
        } else {
            pathState.lensThroughput = Vector3::Zero();
        }
    }
    return true;
}

void PerturbPath(const Scene *scene,
                 const Vector &offset,
                 Path &path,
                 std::vector<SubpathContrib> &contribs,
                 RNG &rng) {
    std::normal_distribution<Float> normDist(Float(0.0), scene->options->discreteStdDev);
    int offsetId = 0;
 
    path.time = Modulo(path.time + normDist(rng), Float(1.0));
    const Camera *camera = scene->camera.get();
    CameraVertex &camVertex = path.camVertex;
    Perturb(camVertex.screenPos[0], offset, offsetId);
    Perturb(camVertex.screenPos[1], offset, offsetId);

    bool useLightCoordinatesPerturb = false;
    UniPathState pathState;
    Init(pathState);
    SamplePrimary(camera, camVertex.screenPos, path.time, pathState.raySeg);
    for (int camDepth = 0; camDepth < (int)path.camSurfaceVertex.size(); camDepth++) {
        SurfaceVertex &surfVertex = path.camSurfaceVertex[camDepth];
        bool hitSurface = true;
        if (useLightCoordinatesPerturb) {
            ShapeInst &shapeInst = surfVertex.shapeInst;
            hitSurface = shapeInst.obj->Intersect(
                shapeInst.primID, path.time, pathState.raySeg, pathState.isect, shapeInst.st);
            if (!hitSurface) {
                return;
            }
        } else {
            hitSurface = Intersect(
                scene, path.time, pathState.raySeg, surfVertex.shapeInst, pathState.isect);
        }
        pathState.wi = -pathState.raySeg.ray.dir;
        if (camDepth == (int)path.camSurfaceVertex.size() - 1) {
            if (path.lgtDepth == 0) {
                const Light *light = GetHitLight(scene, hitSurface, surfVertex.shapeInst.obj);
                if (light != nullptr) {
                    HandleHitLight(camDepth,
                                   scene,
                                   light,
                                   hitSurface,
                                   pathState,
                                   path.time,
                                   path.camVertex.screenPos,
                                   path.envLightInst,
                                   contribs);
                }
            } else if (path.lgtDepth == 1) {
                if (!hitSurface) {
                    return;
                }

                if (camDepth == 1) {
                    path.lensVertexPos = pathState.isect.position;
                    pathState.lensThroughput *= inverse(
                        DistanceSquared(pathState.isect.position, pathState.raySeg.ray.org));
                }

                Float directLightPickProb = PickLightProb(scene, surfVertex.directLightInst.light);
                Perturb(surfVertex.directLightRndParam[0], offset, offsetId);
                Perturb(surfVertex.directLightRndParam[1], offset, offsetId);
                DirectLighting(camDepth,
                               scene,
                               pathState,
                               path.time,
                               path.camVertex.screenPos,
                               directLightPickProb,
                               true,
                               surfVertex,
                               contribs);
            }
            return;
        }

        if (!hitSurface) {
            return;
        }

        surfVertex.bsdfDiscrete = Modulo(surfVertex.bsdfDiscrete + normDist(rng), Float(1.0));

        if (camDepth == 1) {
            path.lensVertexPos = pathState.isect.position;
            const Float distSq =
                DistanceSquared(pathState.isect.position, pathState.raySeg.ray.org);
            if (distSq <= Float(0.0)) {
                contribs.clear();
                return;
            } else {
                pathState.lensThroughput *= inverse(distSq);
            }
        }

        Perturb(surfVertex.bsdfRndParam[0], offset, offsetId);
        Perturb(surfVertex.bsdfRndParam[1], offset, offsetId);
        if (scene->options->useLightCoordinateSampling) {
            if (camDepth == int(path.camSurfaceVertex.size()) - 2 && path.lgtDepth == 0) {
                const ShapeInst &shapeInst = path.camSurfaceVertex.back().shapeInst;
                if (shapeInst.obj != nullptr && shapeInst.obj->areaLight != nullptr) {
                    useLightCoordinatesPerturb = true;
                }
            }
        }
        Vector3 bsdfContrib;
        if (useLightCoordinatesPerturb) {
            if (!LightCoordinateSampling(camDepth,
                                         scene,
                                         path.time,
                                         surfVertex,
                                         path.camSurfaceVertex.back(),
                                         true,
                                         pathState,
                                         bsdfContrib)) {
                return;
            }
        } else {
            if (!BSDFSampling<true>(camDepth,
                                    scene->options->roughnessThreshold,
                                    pathState,
                                    surfVertex,
                                    bsdfContrib)) {
                return;
            }
        }

        pathState.throughput *= surfVertex.rrWeight;
        // RussianRoulette(camDepth, bsdfContrib, surfVertex.rrWeight, pathState.throughput);

        pathState.raySeg.minT = c_IsectEpsilon;
        pathState.raySeg.maxT = std::numeric_limits<Float>::infinity();
    }
}

bool LightCoordinateSampling(const int camDepth,
                             const Scene *scene,
                             const Float time,
                             const SurfaceVertex &curSurfVertex,
                             const SurfaceVertex &nextSurfVertex,
                             const bool doOcclusion,
                             BidirPathState &pathState,
                             Vector3 &dir,
                             Vector3 &bsdfContrib) {
    const ShapeInst &shapeInst = curSurfVertex.shapeInst;
    const ShapeInst &nextShapeInst = nextSurfVertex.shapeInst;
    const BSDF *bsdf = shapeInst.obj->bsdf.get();
    const Intersection &isect = pathState.isect;
    const Vector3 &wi = pathState.wi;

    Vector3 nextPosition, nextNormal;
    assert(nextShapeInst.obj != nullptr);
    Float shapePdf = Float(0.0);
    nextShapeInst.obj->Sample(curSurfVertex.bsdfRndParam,
                              time,
                              nextSurfVertex.shapeInst.primID,
                              nextPosition,
                              nextNormal,
                              &shapePdf);

    dir = nextPosition - isect.position;
    Float distToLightSq = LengthSquared(dir);
    Float distToLight = sqrt(distToLightSq);
    dir *= inverse(distToLight);

    if (doOcclusion && Occluded(scene, time, Ray{isect.position, dir}, distToLight)) {
        return false;
    }

    Float cosWo, bsdfPdf, bsdfRevPdf;
    bsdf->Evaluate(
        wi, isect.shadingNormal, dir, shapeInst.st, bsdfContrib, cosWo, bsdfPdf, bsdfRevPdf);
    if (bsdfContrib.isZero()) {
        return false;
    }

    bsdfContrib *= inverse(bsdfPdf);
    pathState.throughput = pathState.throughput.cwiseProduct(bsdfContrib);
    pathState.ssJacobian *= fabs(Dot(dir, nextNormal) * inverse(distToLightSq)) * bsdfPdf;
    pathState.accMISWThis =
        MIS(cosWo / bsdfPdf) * (pathState.accMISWThis * MIS(bsdfRevPdf) + pathState.accMISWPrev);
    pathState.accMISWPrev = MIS(inverse(bsdfPdf));

    if (camDepth == 1) {
        bool useAbsoluteParam = bsdf->Roughness(shapeInst.st, curSurfVertex.bsdfDiscrete) >
                                scene->options->roughnessThreshold;
        if (useAbsoluteParam) {
            Vector3 impBsdfContrib;
            Float impCosWo, impBsdfPdf, impBsdfRevPdf;
            bsdf->EvaluateAdjoint(dir,
                                  isect.shadingNormal,
                                  wi,
                                  shapeInst.st,
                                  impBsdfContrib,
                                  impCosWo,
                                  impBsdfPdf,
                                  impBsdfRevPdf);
            const Float factor = ShadingNormalCorrection<true>(dir, isect, wi);
            impBsdfContrib *= factor;
            pathState.lensContrib = pathState.lensContrib.cwiseProduct(impBsdfContrib);
        } else {
            pathState.lensContrib = Vector3::Zero();
        }
    }
    return true;
}

void PerturbPathBidir(const Scene *scene,
                      const Vector &offset,
                      Path &path,
                      std::vector<SubpathContrib> &contribs,
                      RNG &rng) {
    std::normal_distribution<Float> normDist(Float(0.0), scene->options->discreteStdDev);
    const Camera *camera = scene->camera.get();

    assert(path.isSubpath);

    int offsetId = 0;
    path.time = Modulo(path.time + normDist(rng), Float(1.0));
    BidirPathState lightPathState;
    if (path.lgtDepth > 1) {
        const Float lightPickProb = PickLightProb(scene, path.lgtVertex.lightInst.light);
        RaySegment raySeg;
        Perturb(path.lgtVertex.rndParamPos[0], offset, offsetId);
        Perturb(path.lgtVertex.rndParamPos[1], offset, offsetId);
        Perturb(path.lgtVertex.rndParamDir[0], offset, offsetId);
        Perturb(path.lgtVertex.rndParamDir[1], offset, offsetId);
        EmitFromLight(
            scene->bSphere, lightPickProb, path.time, path.lgtVertex, raySeg.ray, lightPathState);
        raySeg.minT = c_IsectEpsilon;
        raySeg.maxT = std::numeric_limits<Float>::infinity();

        Vector3 prevLensContrib = Vector3::Zero();
        for (int lgtDepth = 0; lgtDepth < (int)path.lgtSurfaceVertex.size(); lgtDepth++) {
            SurfaceVertex &surfVertex = path.lgtSurfaceVertex[lgtDepth];
            if (!Intersect(scene, path.time, raySeg, surfVertex.shapeInst, lightPathState.isect)) {
                return;
            }

            // If we do full BDPT with non-pinhole cameras we need to handle camera hit here

            lightPathState.wi = -raySeg.ray.dir;
            surfVertex.bsdfDiscrete = Modulo(surfVertex.bsdfDiscrete + normDist(rng), Float(1.0));

            ConvertMIS(lgtDepth, path.lgtVertex.lightInst.light, raySeg.ray, lightPathState);

            if (lgtDepth == (int)path.lgtSurfaceVertex.size() - 1 && path.camDepth == 1) {
                ConnectToCamera(lgtDepth,
                                scene,
                                camera,
                                path.time,
                                lightPathState,
                                path.lgtSurfaceVertex[lgtDepth],
                                prevLensContrib,
                                raySeg.ray.org,
                                contribs);
                return;
            }

            if (lgtDepth == (int)path.lgtSurfaceVertex.size() - 1) {
                break;
            }

            Perturb(surfVertex.bsdfRndParam[0], offset, offsetId);
            Perturb(surfVertex.bsdfRndParam[1], offset, offsetId);
            Vector3 bsdfContrib;
            if (!BSDFSampling<true, true>(scene->options->roughnessThreshold,
                                          lgtDepth,
                                          lightPathState,
                                          surfVertex,
                                          lightPathState,
                                          raySeg.ray.dir,
                                          bsdfContrib)) {
                return;
            }

            lightPathState.throughput *= surfVertex.rrWeight;
            // RussianRoulette(lgtDepth, bsdfContrib, surfVertex.rrWeight,
            // lightPathState.throughput);

            prevLensContrib = lightPathState.lensContrib;
            raySeg.ray.org = lightPathState.isect.position;
        }
    }

    CameraVertex &camVertex = path.camVertex;
    Perturb(camVertex.screenPos[0], offset, offsetId);
    Perturb(camVertex.screenPos[1], offset, offsetId);
    RaySegment raySeg;
    BidirPathState camPathState;
    EmitFromCamera(path.time, camera, path.camVertex, raySeg, camPathState);
    bool useLightCoordinatesPerturb = false;
    for (int camDepth = 0; camDepth < (int)path.camSurfaceVertex.size(); camDepth++) {
        SurfaceVertex &surfVertex = path.camSurfaceVertex[camDepth];
        bool hitSurface =
            Intersect(scene, path.time, raySeg, surfVertex.shapeInst, camPathState.isect);

        camPathState.wi = -raySeg.ray.dir;

        if (hitSurface) {
            ConvertMIS(camDepth, nullptr, raySeg.ray, camPathState);
        }

        if (camDepth == (int)path.camSurfaceVertex.size() - 1 && path.lgtDepth == 0) {
            const Light *light = GetHitLight(scene, hitSurface, surfVertex.shapeInst.obj);
            if (light != nullptr) {
                HandleHitLight(camDepth,
                               scene,
                               light,
                               hitSurface,
                               raySeg.ray,
                               path.time,
                               path.camVertex.screenPos,
                               camPathState,
                               true,
                               path.envLightInst,
                               contribs);
            }
            return;
        }

        if (!hitSurface) {
            return;
        }

        surfVertex.bsdfDiscrete = Modulo(surfVertex.bsdfDiscrete + normDist(rng), Float(1.0));

        if (camDepth == 1) {
            path.lensVertexPos = camPathState.isect.position;
            const Float distSq = DistanceSquared(camPathState.isect.position, raySeg.ray.org);
            if (distSq <= Float(0.0)) {
                contribs.clear();
                return;
            } else {
                camPathState.lensContrib *= inverse(distSq);
            }
        }

        if (camDepth == (int)path.camSurfaceVertex.size() - 1) {
            if (path.lgtDepth == 1) {
                assert(surfVertex.directLightInst.light != nullptr);
                const Float directLightPickProb =
                    PickLightProb(scene, surfVertex.directLightInst.light);
                Perturb(surfVertex.directLightRndParam[0], offset, offsetId);
                Perturb(surfVertex.directLightRndParam[1], offset, offsetId);
                DirectLighting(camDepth,
                               scene,
                               path.time,
                               camPathState,
                               path.camVertex.screenPos,
                               directLightPickProb,
                               surfVertex,
                               true,
                               true,
                               contribs);
            } else {  // path.lgtDepth > 1
                ConnectVertex(camDepth,
                              path.lgtSurfaceVertex.size() - 1,
                              scene,
                              path.time,
                              lightPathState,
                              path.lgtSurfaceVertex.back(),
                              camPathState,
                              surfVertex,
                              path.camVertex.screenPos,
                              true,
                              contribs);
            }
            return;
        }

        Perturb(surfVertex.bsdfRndParam[0], offset, offsetId);
        Perturb(surfVertex.bsdfRndParam[1], offset, offsetId);

        if (scene->options->useLightCoordinateSampling) {
            if (camDepth == int(path.camSurfaceVertex.size()) - 2 && path.lgtDepth == 0) {
                const ShapeInst &shapeInst = path.camSurfaceVertex.back().shapeInst;
                if (shapeInst.obj != nullptr && shapeInst.obj->areaLight != nullptr) {
                    useLightCoordinatesPerturb = true;
                }
            }
        }
        Vector3 bsdfContrib;
        if (useLightCoordinatesPerturb) {
            if (!LightCoordinateSampling(camDepth,
                                         scene,
                                         path.time,
                                         surfVertex,
                                         path.camSurfaceVertex.back(),
                                         true,
                                         camPathState,
                                         raySeg.ray.dir,
                                         bsdfContrib)) {
                return;
            }
        } else {
            if (!BSDFSampling<false, true>(scene->options->roughnessThreshold,
                                           camDepth,
                                           camPathState,
                                           surfVertex,
                                           camPathState,
                                           raySeg.ray.dir,
                                           bsdfContrib)) {
                return;
            }
        }

        // RussianRoulette(camDepth, bsdfContrib, surfVertex.rrWeight, camPathState.throughput);
        camPathState.throughput *= surfVertex.rrWeight;

        raySeg.ray.org = camPathState.isect.position;
        raySeg.minT = c_IsectEpsilon;
        raySeg.maxT = std::numeric_limits<Float>::infinity();
    }
}

struct ADUniPathState {
    ADRay ray;
    ADVector3 throughput;
    ADFloat lastBsdfPdf;
    ADVector3 lensThroughput;
    ADIntersection isect;
    ADVector3 wi;
    ADVector2 st;
};

static void HandleHitLight(const int camDepth,
                           const ADBSphere &sceneSphere,
                           const ADFloat *camBuffer,
                           ADUniPathState &pathState,
                           const ADFloat time,
                           const ADVector2 screenPos) 
{
    const ADRay &ray = pathState.ray;
    const ADIntersection &isect = pathState.isect;
    ADVector3 &throughput = pathState.throughput;
    const ADFloat lastBsdfPdf = pathState.lastBsdfPdf;

    ADVector3 emission;
    ADFloat directPdf;
    ADFloat emissionPdf;
    Emission(camBuffer,
             sceneSphere,
             ray.dir,
             isect.shadingNormal,
             time,
             emission,
             directPdf,
             emissionPdf);

    ADFloat type;
    Deserialize(camBuffer, type);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(2);
    BeginIf(Eq(type, Float(LightType::AreaLight)), ret);
    {
        ADFloat distSq = DistanceSquared(ray.org, isect.position);
        ADFloat cosTheta = -Dot(ray.dir, isect.shadingNormal);
        ADFloat pdfFactor = (distSq / cosTheta);
        SetCondOutput({pdfFactor, inverse(distSq)});
    }
    BeginElse();
    { SetCondOutput({Const<ADFloat>(1.0), Const<ADFloat>(1.0)}); }
    EndIf();
    ADFloat pdfFactor = ret[0];
    ADFloat invDistSq = ret[1];
    directPdf *= pdfFactor;
    if (camDepth == 1) {
        pathState.lensThroughput *= invDistSq;
    }
    throughput = throughput.cwiseProduct(emission);
    ADFloat misWeight = Const<ADFloat>(1.0);
    ADFloat lightPickProb;
    camBuffer += GetMaxLightSerializedSize();
    camBuffer = Deserialize(camBuffer, lightPickProb);
    if (camDepth > 0) {
        misWeight = MISWeight(lastBsdfPdf, directPdf * lightPickProb);
        throughput *= misWeight;
    }
}

static void DirectLighting(const int camDepth,
                           const ADBSphere &sceneSphere,
                           const ADFloat *camBuffer,
                           ADUniPathState &pathState,
                           const ADVector2 rndParam,
                           const ADFloat time,
                           const ADVector2 screenPos,
                           const bool isStatic) 
{
    const ADIntersection &isect = pathState.isect;
    ADVector3 &throughput = pathState.throughput;
    const ADVector3 &wi = pathState.wi;

    ADVector3 dirToLight;
    ADVector3 lightContrib;
    ADFloat cosAtLight;
    ADFloat directPdf;
    ADFloat emissionPdf;
    ADFloat lightType;
    Deserialize(camBuffer, lightType);
    camBuffer = SampleDirect(camBuffer,
                             sceneSphere,
                             isect.position,
                             isect.shadingNormal,
                             rndParam,
                             time,
                             isStatic,
                             dirToLight,
                             lightContrib,
                             cosAtLight,
                             directPdf,
                             emissionPdf);

    ADVector3 bsdfContrib;
    ADFloat cosWo;
    ADFloat bsdfPdf;
    ADFloat bsdfRevPdf;
    const ADFloat *bsdfBuffer = camBuffer;
    camBuffer = EvaluateBSDF(false,
                             camBuffer,
                             wi,
                             isect.shadingNormal,
                             dirToLight,
                             pathState.st,
                             bsdfContrib,
                             cosWo,
                             bsdfPdf,
                             bsdfRevPdf);

    // Safe to assume that surfVertex uses absolute param
    if (camDepth == 0) {
        pathState.lensThroughput = pathState.lensThroughput.cwiseProduct(bsdfContrib);
    } else if (camDepth == 1) {
        ADVector3 impBsdfContrib;
        ADFloat impCosWo, impBsdfPdf, impBsdfRevPdf;
        EvaluateBSDF(true,
                     bsdfBuffer,
                     dirToLight,
                     isect.shadingNormal,
                     wi,
                     pathState.st,
                     impBsdfContrib,
                     impCosWo,
                     impBsdfPdf,
                     impBsdfRevPdf);

        // Adjoint BSDF correction
        const ADFloat factor = ShadingNormalCorrection<true>(dirToLight, isect, wi);
        impBsdfContrib *= factor;
        pathState.lensThroughput = pathState.lensThroughput.cwiseProduct(impBsdfContrib);
    }
 
    ADFloat lightPickProb;
    camBuffer = Deserialize(camBuffer, lightPickProb);
    throughput = throughput.cwiseProduct(bsdfContrib);
    throughput = throughput.cwiseProduct(lightContrib) * inverse(lightPickProb);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(1);
    BeginIf(Eq(lightType, Float(LightType::PointLight)), ret);
    { SetCondOutput({Const<ADFloat>(1.0)}); }
    BeginElse();
    {
        ADFloat misWeight = MISWeight(directPdf * lightPickProb, bsdfPdf);
        SetCondOutput({misWeight});
    }
    EndIf();
    ADFloat misWeight = ret[0];
    throughput *= misWeight;
}

template <bool fixedDiscrete = false>
static const ADFloat *BSDFSampling(const int camDepth,
                                   const ADFloat *buffer,
                                   const ADVector2 bsdfRndParam,
                                   const ADFloat bsdfDiscrete,
                                   const ADFloat useAbsoluteParam,
                                   const ADFloat time,
                                   const bool isStatic,
                                   const ADFloat useLightCoordinateSampling,
                                   const bool doLightCoordinateSampling,
                                   ADUniPathState &pathState) 
{
    const ADVector3 &wi = pathState.wi;
    const ADIntersection &isect = pathState.isect;
    ADRay &ray = pathState.ray;
    ADVector3 &throughput = pathState.throughput;
    ADFloat cosWo;
    ADFloat bsdfPdfRev;
    const ADFloat *bsdfBuffer = buffer;
    std::vector<CondExprCPtr> ret = CreateCondExprVec(8);
    if (doLightCoordinateSampling) {
        const ADFloat *nextShapeBuffer = buffer + GetMaxBSDFSerializedSize() + 1;
        const ADFloat *nextLightBuffer = nextShapeBuffer + GetMaxShapeSerializedSize();
        ADFloat lightType;
        Deserialize(nextLightBuffer, lightType);
        BeginIf(
            And(Eq(useLightCoordinateSampling, FTRUE), Eq(lightType, Float(LightType::AreaLight))),
            ret);
        {
            ADVector3 dir;
            ADVector3 bsdfContrib;
            ADFloat cosWo;
            ADFloat bsdfPdf;
            ADFloat bsdfPdfRev;

            ADVector3 nextPosition, nextNormal;
            ADFloat shapePdf;
            SampleShape(
                nextShapeBuffer, bsdfRndParam, time, isStatic, nextPosition, nextNormal, shapePdf);
            dir = nextPosition - isect.position;
            ADFloat distSq = LengthSquared(dir);
            ADFloat invDistSq = inverse(distSq);
            ADFloat invDist = sqrt(invDistSq);
            dir *= invDist;
            EvaluateBSDF(false,
                         buffer,
                         wi,
                         isect.shadingNormal,
                         dir,
                         pathState.st,
                         bsdfContrib,
                         cosWo,
                         bsdfPdf,
                         bsdfPdfRev);
            ADFloat jacobian = fabs(Dot(nextNormal, dir) * invDistSq) / shapePdf;
            SetCondOutput({dir[0],
                           dir[1],
                           dir[2],
                           bsdfContrib[0],
                           bsdfContrib[1],
                           bsdfContrib[2],
                           bsdfPdf,
                           jacobian});
        }
        BeginElseIf(Eq(useAbsoluteParam, FFALSE));
    } else {
        BeginIf(Eq(useAbsoluteParam, FFALSE), ret);
    }
    {
        ADVector3 dir;
        ADVector3 bsdfContrib;
        ADFloat cosWo;
        ADFloat bsdfPdf;
        ADFloat bsdfPdfRev;
        SampleBSDF(false,
                   buffer,
                   wi,
                   isect.shadingNormal,
                   pathState.st,
                   bsdfRndParam,
                   bsdfDiscrete,
                   fixedDiscrete,
                   dir,
                   bsdfContrib,
                   cosWo,
                   bsdfPdf,
                   bsdfPdfRev);
        SetCondOutput({dir[0],
                       dir[1],
                       dir[2],
                       bsdfContrib[0],
                       bsdfContrib[1],
                       bsdfContrib[2],
                       bsdfPdf,
                       Const<ADFloat>(1.0)});
    }
    BeginElse();
    {
        ADVector3 dir;
        ADVector3 bsdfContrib;
        ADFloat cosWo;
        ADFloat bsdfPdf;
        ADFloat bsdfPdfRev;
        ADFloat jacobian;
        dir = SampleSphere(bsdfRndParam, jacobian);
        EvaluateBSDF(false,
                     buffer,
                     wi,
                     isect.shadingNormal,
                     dir,
                     pathState.st,
                     bsdfContrib,
                     cosWo,
                     bsdfPdf,
                     bsdfPdfRev);
        SetCondOutput({dir[0],
                       dir[1],
                       dir[2],
                       bsdfContrib[0],
                       bsdfContrib[1],
                       bsdfContrib[2],
                       bsdfPdf,
                       jacobian});
    }
    EndIf();
    buffer += GetMaxBSDFSerializedSize();
    ray.dir[0] = ret[0];
    ray.dir[1] = ret[1];
    ray.dir[2] = ret[2];
    ADVector3 bsdfContrib;
    bsdfContrib[0] = ret[3];
    bsdfContrib[1] = ret[4];
    bsdfContrib[2] = ret[5];
    pathState.lastBsdfPdf = ret[6];
    ADFloat jacobian = ret[7];

    // lensThroughput computation
    if (camDepth == 0) {
        pathState.lensThroughput = pathState.lensThroughput.cwiseProduct(bsdfContrib);
    } else if (camDepth == 1) {
        ADVector3 impBsdfContrib;
        ADFloat impCosWo;
        ADFloat impBsdfPdf;
        ADFloat impBsdfPdfRev;
        EvaluateBSDF(true,
                     bsdfBuffer,
                     wi,
                     isect.shadingNormal,
                     ray.dir,
                     pathState.st,
                     impBsdfContrib,
                     impCosWo,
                     impBsdfPdf,
                     impBsdfPdfRev);
        ADFloat factor = ShadingNormalCorrection<true>(wi, isect, ray.dir);
        impBsdfContrib *= factor;
        pathState.lensThroughput = pathState.lensThroughput.cwiseProduct(impBsdfContrib);
    }

    bsdfContrib *= jacobian;
    throughput = throughput.cwiseProduct(bsdfContrib);
    ray.org = isect.position;
    return buffer;
}


size_t GetPrimaryParamSize(const int camDepth, const int lightDepth) {
    return std::max(camDepth + lightDepth - 1, 2) * 2 + 1;
}

size_t GetVertParamSize(const int maxCamDepth, const int maxLgtDepth) 
{
    // Crude upper bound, should fix this
    const int maxDepth = maxCamDepth + maxLgtDepth;
    return maxDepth * GetMaxShapeSerializedSize() + maxDepth * GetMaxBSDFSerializedSize() +
           GetMaxLightSerializedSize() + maxDepth * 2 +  // bsdfDiscrete & useAbsoluteParam
           maxDepth * 1 +                                // rrWeight
           3 +                                           // lensVertexPos
           1 +                                           // useLightCoord
           1;                                            // picklightpropb
}

void Serialize(const Scene *scene, const Path &path, SerializedSubpath &subPath) 
{
    int primaryIdx = 0;
    subPath.primary[primaryIdx++] = path.time;
    Float *buffer = &subPath.vertParams[0];
    buffer = Serialize(path.lensVertexPos, buffer);
    if (path.lgtDepth > 1) {
        const Light *light = path.lgtVertex.lightInst.light;
        subPath.primary[primaryIdx++] = path.lgtVertex.rndParamPos[0];
        subPath.primary[primaryIdx++] = path.lgtVertex.rndParamPos[1];
        subPath.primary[primaryIdx++] = path.lgtVertex.rndParamDir[0];
        subPath.primary[primaryIdx++] = path.lgtVertex.rndParamDir[1];
        buffer = Serialize(PickLightProb(scene, light), buffer);
        light->Serialize(path.lgtVertex.lightInst.lPrimID, buffer);
        buffer += GetMaxLightSerializedSize();

        for (int lgtDepth = 0; lgtDepth < (int)path.lgtSurfaceVertex.size(); lgtDepth++) {
            const SurfaceVertex &surfVertex = path.lgtSurfaceVertex[lgtDepth];
            const ShapeInst &shapeInst = surfVertex.shapeInst;
            shapeInst.obj->Serialize(shapeInst.primID, buffer);
            buffer += GetMaxShapeSerializedSize();
            buffer = Serialize(surfVertex.bsdfDiscrete, buffer);
            buffer = Serialize(surfVertex.useAbsoluteParam, buffer);
            shapeInst.obj->bsdf->Serialize(shapeInst.st, buffer);
            buffer += GetMaxBSDFSerializedSize();

            // If we do full BDPT with non-pinhole cameras we need to handle camera hit here
            if (lgtDepth == (int)path.lgtSurfaceVertex.size() - 1 && path.camDepth == 1) {
                return;
            }

            if (lgtDepth == (int)path.lgtSurfaceVertex.size() - 1) {
                break;
            }

            subPath.primary[primaryIdx++] = surfVertex.bsdfRndParam[0];
            subPath.primary[primaryIdx++] = surfVertex.bsdfRndParam[1];
            buffer = Serialize(surfVertex.rrWeight, buffer);
        }
    }

    const CameraVertex &camVertex = path.camVertex;
    subPath.primary[primaryIdx++] = camVertex.screenPos[0];
    subPath.primary[primaryIdx++] = camVertex.screenPos[1];

    for (int camDepth = 0; camDepth < (int)path.camSurfaceVertex.size(); camDepth++) {
        const SurfaceVertex &surfVertex = path.camSurfaceVertex[camDepth];
        const ShapeInst &shapeInst = surfVertex.shapeInst;
        if (shapeInst.obj != nullptr) {
            shapeInst.obj->Serialize(shapeInst.primID, buffer);
        }
        buffer += GetMaxShapeSerializedSize();
        if (camDepth == (int)path.camSurfaceVertex.size() - 1) {
            if (path.lgtDepth == 0) {
                if (path.envLightInst.light != nullptr) {
                    path.envLightInst.light->Serialize(path.envLightInst.lPrimID, buffer);
                    buffer += GetMaxLightSerializedSize();
                    buffer = Serialize(PickLightProb(scene, path.envLightInst.light), buffer);
                } else {
                    assert(shapeInst.obj != nullptr);
                    assert(shapeInst.obj->areaLight != nullptr);
                    shapeInst.obj->areaLight->Serialize(shapeInst.primID, buffer);
                    buffer += GetMaxLightSerializedSize();
                    buffer = Serialize(PickLightProb(scene, shapeInst.obj->areaLight), buffer);
                }
            } else if (path.lgtDepth == 1) {
                subPath.primary[primaryIdx++] = surfVertex.directLightRndParam[0];
                subPath.primary[primaryIdx++] = surfVertex.directLightRndParam[1];
                surfVertex.directLightInst.light->Serialize(surfVertex.directLightInst.lPrimID,
                                                            buffer);
                buffer += GetMaxLightSerializedSize();
                shapeInst.obj->bsdf->Serialize(shapeInst.st, buffer);
                buffer += GetMaxBSDFSerializedSize();
                buffer = Serialize(PickLightProb(scene, surfVertex.directLightInst.light), buffer);
            } else {  // path.lgtDepth >= 2
                shapeInst.obj->bsdf->Serialize(shapeInst.st, buffer);
                buffer += GetMaxBSDFSerializedSize();
            }
            return;
        }

        subPath.primary[primaryIdx++] = surfVertex.bsdfRndParam[0];
        subPath.primary[primaryIdx++] = surfVertex.bsdfRndParam[1];
        buffer = Serialize(surfVertex.bsdfDiscrete, buffer);
        buffer = Serialize(surfVertex.useAbsoluteParam, buffer);
        shapeInst.obj->bsdf->Serialize(shapeInst.st, buffer);
        buffer += GetMaxBSDFSerializedSize();
        buffer = Serialize(surfVertex.rrWeight, buffer);
    }
}

void GetPathPss(const Path &path, std::vector<Float> &pss) {
    assert(path.isSubpath);
    const int dim = GetDimension(path); 
    if (!pss.size())    pss.resize(dim); 

    int primaryIdx = 0;
    if (path.lgtDepth > 1) {
        pss[primaryIdx++] = path.lgtVertex.rndParamPos[0];
        pss[primaryIdx++] = path.lgtVertex.rndParamPos[1];
        pss[primaryIdx++] = path.lgtVertex.rndParamDir[0];
        pss[primaryIdx++] = path.lgtVertex.rndParamDir[1];
        for (int lgtDepth = 0; lgtDepth < (int)path.lgtSurfaceVertex.size(); lgtDepth++) {
            const SurfaceVertex &surfVertex = path.lgtSurfaceVertex[lgtDepth];
            // If we do full BDPT with non-pinhole cameras we need to handle camera hit here
            if (lgtDepth == (int)path.lgtSurfaceVertex.size() - 1 && path.camDepth == 1) {
                return ;
            }
            if (lgtDepth == (int)path.lgtSurfaceVertex.size() - 1) {
                break;
            }
            pss[primaryIdx++] = surfVertex.bsdfRndParam[0];
            pss[primaryIdx++] = surfVertex.bsdfRndParam[1];
        }
    }

    const CameraVertex &camVertex = path.camVertex;
    pss[primaryIdx++] = camVertex.screenPos[0];
    pss[primaryIdx++] = camVertex.screenPos[1];

    for (int camDepth = 0; camDepth < (int)path.camSurfaceVertex.size(); camDepth++) {
        const SurfaceVertex &surfVertex = path.camSurfaceVertex[camDepth];
        if (camDepth == (int)path.camSurfaceVertex.size() - 1) {
            if (path.lgtDepth == 1) {
                pss[primaryIdx++] = surfVertex.directLightRndParam[0];
                pss[primaryIdx++] = surfVertex.directLightRndParam[1];
            }
            return ;
        }

        pss[primaryIdx++] = surfVertex.bsdfRndParam[0];
        pss[primaryIdx++] = surfVertex.bsdfRndParam[1];
    }
    assert(false);
    return ;
}

enum class PathFuncMode { Full, Static, Lens };

std::string GetFuncName(const int camDepth, const int lightDepth, const PathFuncMode mode) 
{
    std::string baseName =
        "evaluate_path_" + std::to_string(camDepth) + "_" + std::to_string(lightDepth);
    if (mode == PathFuncMode::Full) {
        return baseName + "_full";
    } else if (mode == PathFuncMode::Static) {
        return baseName + "_static";
    } else {
        return baseName + "_lens";
    }
    assert(false);
    return baseName;
}


std::string RegisterPathFunc(const int maxCamDepth,
                             const int maxLightDepth,
                             const PathFuncMode mode,
                             Library &lib) 
{
    auto lensParamsArg = Argument("lens", 2);
    auto primaryParamsArg = Argument("primary", GetPrimaryParamSize(maxCamDepth, maxLightDepth));
    auto sceneParamsArg = Argument("scene", GetSceneSerializedSize());
    auto vertParamsArg = Argument("vertParams", GetVertParamSize(maxCamDepth, maxLightDepth));
    std::string funcName = GetFuncName(maxCamDepth, maxLightDepth, mode);
    std::cout << "Compiling function " << funcName << std::endl;
    auto func = BeginFunction(
        funcName,
        std::vector<Argument>{lensParamsArg, primaryParamsArg, sceneParamsArg, vertParamsArg});
    auto lensParams = lensParamsArg.GetExprVec();
    auto primaryParams = primaryParamsArg.GetExprVec();
    auto sceneParams = sceneParamsArg.GetExprVec();
    auto vertParams = vertParamsArg.GetExprVec();

    const ADFloat *sceneBuffer = &sceneParams[0];
    ADFloat useLightCoordinateSampling;
    sceneBuffer = Deserialize(sceneBuffer, useLightCoordinateSampling);
    ADMatrix4x4 sampleToCam;
    sceneBuffer = Deserialize(sceneBuffer, sampleToCam);
    ADAnimatedTransform camToWorld;
    sceneBuffer = Deserialize(sceneBuffer, camToWorld);
    ADFloat screenPixelCount;
    sceneBuffer = Deserialize(sceneBuffer, screenPixelCount);
    ADFloat camDist;
    sceneBuffer = Deserialize(sceneBuffer, camDist);
    ADBSphere sceneSphere;
    sceneBuffer = Deserialize(sceneBuffer, sceneSphere);

    const ADFloat *buffer = &vertParams[0];
    int primaryIdx = 0;
    ADFloat time = primaryParams[primaryIdx++];
    ADVector2 screenPos;
    if (mode == PathFuncMode::Lens) {
        screenPos[0] = lensParams[0];
        screenPos[1] = lensParams[1];
        primaryIdx += 2;
    } else {
        screenPos[0] = primaryParams[primaryIdx++];
        screenPos[1] = primaryParams[primaryIdx++];
    }

    const bool isStatic = mode == PathFuncMode::Static;

    ADUniPathState pathState;
    SamplePrimary(sampleToCam, camToWorld, screenPos, time, isStatic, pathState.ray);
    pathState.throughput = ADVector3(Const<ADFloat>(1.0), Const<ADFloat>(1.0), Const<ADFloat>(1.0));
    pathState.lensThroughput =
        ADVector3(Const<ADFloat>(1.0), Const<ADFloat>(1.0), Const<ADFloat>(1.0));
    pathState.lastBsdfPdf = Const<ADFloat>(1.0);

    ADVector3 lensVertexPos;
    buffer = Deserialize(buffer, lensVertexPos);
    for (int camDepth = 0; camDepth < maxCamDepth - 1; camDepth++) {
        buffer = Intersect(buffer, pathState.ray, time, isStatic, pathState.isect, pathState.st);
        pathState.wi = -pathState.ray.dir;

        if (camDepth == maxCamDepth - 2) {
            if (maxLightDepth == 0) {
                HandleHitLight(camDepth, sceneSphere, buffer, pathState, time, screenPos);
            } else {
                assert(maxLightDepth == 1);

                if (camDepth == 1) {
                    pathState.lensThroughput *=
                        inverse(DistanceSquared(pathState.isect.position, pathState.ray.org));
                }

                ADVector2 directLightRndParam;
                directLightRndParam[0] = primaryParams[primaryIdx++];
                directLightRndParam[1] = primaryParams[primaryIdx++];
                DirectLighting(camDepth,
                               sceneSphere,
                               buffer,
                               pathState,
                               directLightRndParam,
                               time,
                               screenPos,
                               isStatic);
            }
            break;
        }

        if (camDepth == 1) {
            pathState.lensThroughput *=
                inverse(DistanceSquared(pathState.isect.position, pathState.ray.org));
        }

        ADVector2 bsdfRndParam;
        bsdfRndParam[0] = primaryParams[primaryIdx++];
        bsdfRndParam[1] = primaryParams[primaryIdx++];
        ADFloat bsdfDiscrete;
        buffer = Deserialize(buffer, bsdfDiscrete);
        ADFloat useAbsoluteParam;
        buffer = Deserialize(buffer, useAbsoluteParam);
        if (mode == PathFuncMode::Lens && camDepth == 0) {
            //buffer = PerturbLensFirstVertex(buffer, lensVertexPos, pathState);
        } else {
            bool doLightCoordinateSampling = maxLightDepth == 0 && camDepth == maxCamDepth - 3;
            buffer = BSDFSampling<false>(camDepth,
                                         buffer,
                                         bsdfRndParam,
                                         bsdfDiscrete,
                                         useAbsoluteParam,
                                         time,
                                         isStatic,
                                         useLightCoordinateSampling,
                                         doLightCoordinateSampling,
                                         pathState);
        }
        ADFloat rrWeight;
        buffer = Deserialize(buffer, rrWeight);
        pathState.throughput *= rrWeight;
    }

    ADFloat logLumValue = log(
        Luminance(mode == PathFuncMode::Lens ? pathState.lensThroughput : pathState.throughput));
    EndFunction({{"logLumValue", {logLumValue}}});

    ExpressionCPtrVec wrt;
    if (mode == PathFuncMode::Full) {
        wrt = primaryParams;
    } else if (mode == PathFuncMode::Static) {
        wrt = ExpressionCPtrVec(primaryParams.begin() + 1, primaryParams.end());
    } else {
        assert(mode == PathFuncMode::Lens);
        wrt = lensParams;
    }
    lib.RegisterFunc(func);
    lib.RegisterFuncDerv(func, wrt, logLumValue);
    return func->name;
}

struct ADBidirPathState {
    ADIntersection isect;
    ADVector3 wi;
    ADFloat accMISWPrev;
    ADFloat accMISWThis;
    ADVector3 throughput;
    ADVector2 st;
    ADVector3 lensContrib;
};

static const ADFloat *EmitFromLight(const ADFloat *buffer,
                                    const ADBSphere &bSphere,
                                    const ADFloat lightPickProb,
                                    const ADFloat time,
                                    const bool isStatic,
                                    const ADVector2 rndParamPos,
                                    const ADVector2 rndParamDir,
                                    ADRay &ray,
                                    ADBidirPathState &pathState) 
{
    ADFloat lightType;
    Deserialize(buffer, lightType);
    ADFloat cosLight;
    ADFloat emissionPdf;
    ADFloat directPdf;
    buffer = Emit(buffer,
                  bSphere,
                  rndParamPos,
                  rndParamDir,
                  time,
                  isStatic,
                  ray,
                  pathState.throughput,
                  cosLight,
                  emissionPdf,
                  directPdf);
    emissionPdf *= lightPickProb;
    directPdf *= lightPickProb;
    pathState.throughput *= inverse(lightPickProb);
    pathState.accMISWPrev = MIS(directPdf / emissionPdf);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(1);
    BeginIf(Eq(lightType, Float(LightType::PointLight)), ret);
    { SetCondOutput({MIS(cosLight / emissionPdf)}); }
    BeginElse();
    { SetCondOutput({Const<ADFloat>(0.0)}); }
    EndIf();
    pathState.accMISWThis = ret[0];
    pathState.lensContrib =
        ADVector3(Const<ADFloat>(0.0), Const<ADFloat>(0.0), Const<ADFloat>(0.0));
    return buffer;
}

static inline void ConvertMISLightEmit(const ADFloat lightType,
                                       const ADRay &ray,
                                       ADBidirPathState &pathState) 
{
    std::vector<CondExprCPtr> ret = CreateCondExprVec(1);
    BeginIf(Eq(lightType, Float(LightType::EnvLight)), ret);
    { SetCondOutput({Const<ADFloat>(1.0)}); }
    BeginElse();
    { SetCondOutput({MIS(DistanceSquared(ray.org, pathState.isect.position))}); }
    EndIf();
    ADFloat invCosTheta = inverse(MIS(fabs(Dot(ray.dir, pathState.isect.shadingNormal))));
    pathState.accMISWPrev *= (invCosTheta * ret[0]);
    pathState.accMISWThis *= invCosTheta;
}

static inline void ConvertMISLightHit(const ADFloat lightType,
                                      const ADRay &ray,
                                      ADBidirPathState &pathState) 
{
    std::vector<CondExprCPtr> ret = CreateCondExprVec(2);
    BeginIf(Eq(lightType, Float(LightType::EnvLight)), ret);
    { SetCondOutput({Const<ADFloat>(1.0), Const<ADFloat>(1.0)}); }
    BeginElse();
    {
        ADFloat distSq = MIS(DistanceSquared(ray.org, pathState.isect.position));
        ADFloat invCosTheta = inverse(MIS(fabs(Dot(ray.dir, pathState.isect.shadingNormal))));
        SetCondOutput({invCosTheta, distSq});
    }
    EndIf();
    pathState.accMISWPrev *= ret[0] * ret[1];
    pathState.accMISWThis *= ret[0];
}

static inline void ConvertMIS(const ADRay &ray, ADBidirPathState &pathState) 
{
    pathState.accMISWPrev *= MIS(DistanceSquared(ray.org, pathState.isect.position));
    ADFloat invCosTheta = inverse(MIS(fabs(Dot(ray.dir, pathState.isect.shadingNormal))));
    pathState.accMISWPrev *= invCosTheta;
    pathState.accMISWThis *= invCosTheta;
}

static void ConnectToCamera(const ADMatrix4x4 &sampleToCam,
                            const ADAnimatedTransform &camToWorld,
                            const ADFloat screenPixelCount,
                            const ADFloat camDist,
                            const ADFloat *buffer,
                            const ADFloat time,
                            const bool isStatic,
                            const ADVector3 &prevLensContrib,
                            const ADVector3 &prevPosition,
                            ADBidirPathState &pathState) 
{
    ADRay centerRay;
    SamplePrimary(sampleToCam,
                  camToWorld,
                  ADVector2(Const<ADFloat>(0.5), Const<ADFloat>(0.5)),
                  time,
                  isStatic,
                  centerRay);
    ADVector3 camOrg = centerRay.org;
    ADVector3 camDir = centerRay.dir;
    ADVector3 dirToCamera = camOrg - pathState.isect.position;

    ADFloat distSq = LengthSquared(dirToCamera);
    ADFloat dist = sqrt(distSq);
    dirToCamera *= inverse(dist);

    ADVector3 bsdfContrib;
    ADFloat cosToCamera, bsdfPdf, bsdfRevPdf;
    const ADFloat *bsdfBuffer = buffer;
    buffer = EvaluateBSDF(true,
                          bsdfBuffer,
                          pathState.wi,
                          pathState.isect.shadingNormal,
                          dirToCamera,
                          pathState.st,
                          bsdfContrib,
                          cosToCamera,
                          bsdfPdf,
                          bsdfRevPdf);
    ADFloat factor = ShadingNormalCorrection<true>(pathState.wi, pathState.isect, dirToCamera);
    bsdfContrib *= factor;
    // Currently we ignore russian roulette in MIS computation, should still be unbiased
    // bsdfPdf *= rrProb;
    // bsdfRevPdf *= rrProb;

    {
        ADVector3 bsdfContrib;
        ADFloat cosWo, bsdfPdf, bsdfRevPdf;
        EvaluateBSDF(false,
                     bsdfBuffer,
                     dirToCamera,
                     pathState.isect.shadingNormal,
                     pathState.wi,
                     pathState.st,
                     bsdfContrib,
                     cosWo,
                     bsdfPdf,
                     bsdfRevPdf);

        pathState.lensContrib = bsdfContrib.cwiseProduct(prevLensContrib) *
                                inverse(DistanceSquared(pathState.isect.position, prevPosition));
    }

    // Compute pdf conversion factor from image plane area to surface area
    ADFloat invCosAtCamera = -inverse(Dot(camDir, dirToCamera));
    ADFloat imagePointToCameraDist = camDist * invCosAtCamera;
    ADFloat imageToSolidAngleFactor = square(imagePointToCameraDist) * invCosAtCamera;
    ADFloat imageToSurfaceFactor = imageToSolidAngleFactor * fabs(cosToCamera) / distSq;
    ADFloat cameraPdf = imageToSurfaceFactor;
    ADFloat wLight = MIS(cameraPdf / screenPixelCount) *
                     (pathState.accMISWPrev + pathState.accMISWThis * MIS(bsdfRevPdf));

    ADFloat misWeight = inverse(wLight + Float(1.0));
    ADFloat surfaceToImageFactor = cosToCamera / imageToSurfaceFactor;
    ADVector3 contrib = misWeight * bsdfContrib / (screenPixelCount * surfaceToImageFactor);
    pathState.throughput = contrib.cwiseProduct(pathState.throughput);
}

template <bool adjoint, bool fixedDiscrete>
static const ADFloat *BSDFSampling(const int depth,
                                   const ADFloat *buffer,
                                   const ADVector2 bsdfRndParam,
                                   const ADFloat bsdfDiscrete,
                                   const ADFloat useAbsoluteParam,
                                   const ADFloat time,
                                   const bool isStatic,
                                   const ADFloat useLightCoordinateSampling,
                                   const bool doLightCoordinateSampling,
                                   ADBidirPathState &pathState,
                                   ADVector3 &dir) 
{
    const ADIntersection &isect = pathState.isect;
    ADVector3 bsdfContrib;
    ADFloat cosWo, bsdfPdf, bsdfRevPdf;
    const ADFloat *bsdfBuffer = buffer;
    std::vector<CondExprCPtr> ret = CreateCondExprVec(10);
    if (doLightCoordinateSampling) {
        const ADFloat *nextShapeBuffer = buffer + GetMaxBSDFSerializedSize() + 1;
        const ADFloat *nextLightBuffer = nextShapeBuffer + GetMaxShapeSerializedSize();
        ADFloat lightType;
        Deserialize(nextLightBuffer, lightType);
        BeginIf(
            And(Eq(useLightCoordinateSampling, FTRUE), Eq(lightType, Float(LightType::AreaLight))),
            ret);
        {
            ADVector3 dir;
            ADVector3 bsdfContrib;
            ADFloat cosWo;
            ADFloat bsdfPdf;
            ADFloat bsdfRevPdf;
            ADFloat jacobian;

            ADVector3 nextPosition, nextNormal;
            ADFloat shapePdf;
            SampleShape(
                nextShapeBuffer, bsdfRndParam, time, isStatic, nextPosition, nextNormal, shapePdf);
            dir = nextPosition - isect.position;
            ADFloat distSq = LengthSquared(dir);
            ADFloat invDistSq = inverse(distSq);
            ADFloat invDist = sqrt(invDistSq);
            dir *= invDist;
            EvaluateBSDF(false,
                         buffer,
                         pathState.wi,
                         isect.shadingNormal,
                         dir,
                         pathState.st,
                         bsdfContrib,
                         cosWo,
                         bsdfPdf,
                         bsdfRevPdf);
            jacobian = fabs(Dot(nextNormal, dir) * invDistSq) / shapePdf;
            SetCondOutput({dir[0],
                           dir[1],
                           dir[2],
                           bsdfContrib[0],
                           bsdfContrib[1],
                           bsdfContrib[2],
                           cosWo,
                           bsdfPdf,
                           bsdfRevPdf,
                           jacobian});
        }
        BeginElseIf(Eq(useAbsoluteParam, FFALSE));
    } else {
        BeginIf(Eq(useAbsoluteParam, FFALSE), ret);
    }
    {
        ADVector3 dir;
        ADVector3 bsdfContrib;
        ADFloat cosWo;
        ADFloat bsdfPdf;
        ADFloat bsdfRevPdf;
        SampleBSDF(adjoint,
                   buffer,
                   pathState.wi,
                   isect.shadingNormal,
                   pathState.st,
                   bsdfRndParam,
                   bsdfDiscrete,
                   fixedDiscrete,
                   dir,
                   bsdfContrib,
                   cosWo,
                   bsdfPdf,
                   bsdfRevPdf);
        SetCondOutput({dir[0],
                       dir[1],
                       dir[2],
                       bsdfContrib[0],
                       bsdfContrib[1],
                       bsdfContrib[2],
                       cosWo,
                       bsdfPdf,
                       bsdfRevPdf,
                       Const<ADFloat>(1.0)});
    }
    BeginElse();
    {
        ADVector3 dir;
        ADVector3 bsdfContrib;
        ADFloat cosWo;
        ADFloat bsdfPdf;
        ADFloat bsdfRevPdf;
        ADFloat jacobian;
        dir = SampleSphere(bsdfRndParam, jacobian);
        EvaluateBSDF(adjoint,
                     buffer,
                     pathState.wi,
                     isect.shadingNormal,
                     dir,
                     pathState.st,
                     bsdfContrib,
                     cosWo,
                     bsdfPdf,
                     bsdfRevPdf);
        SetCondOutput({dir[0],
                       dir[1],
                       dir[2],
                       bsdfContrib[0],
                       bsdfContrib[1],
                       bsdfContrib[2],
                       cosWo,
                       bsdfPdf,
                       bsdfRevPdf,
                       jacobian});
    }
    EndIf();
    buffer += GetMaxBSDFSerializedSize();
    dir[0] = ret[0];
    dir[1] = ret[1];
    dir[2] = ret[2];
    bsdfContrib[0] = ret[3];
    bsdfContrib[1] = ret[4];
    bsdfContrib[2] = ret[5];
    cosWo = ret[6];
    bsdfPdf = ret[7];
    bsdfRevPdf = ret[8];
    ADFloat jacobian = ret[9];

    if (adjoint) {
        ADFloat factor = ShadingNormalCorrection<true>(pathState.wi, isect, dir);
        bsdfContrib *= factor;
    }

    if (adjoint) {
        pathState.lensContrib = bsdfContrib;
    } else if (depth == 1) {
        ADVector3 impBsdfContrib;
        ADFloat impCosWo;
        ADFloat impBsdfPdf;
        ADFloat impBsdfPdfRev;
        EvaluateBSDF(true,
                     bsdfBuffer,
                     dir,
                     isect.shadingNormal,
                     pathState.wi,
                     pathState.st,
                     impBsdfContrib,
                     impCosWo,
                     impBsdfPdf,
                     impBsdfPdfRev);
        ADFloat factor = ShadingNormalCorrection<true>(dir, isect, pathState.wi);
        impBsdfContrib *= factor;
        pathState.lensContrib = pathState.lensContrib.cwiseProduct(impBsdfContrib);
    }

    bsdfContrib *= jacobian;

    // Currently we ignore russian roulette in MIS computation
    // (because it is tricky to get reverse probability correct), should still be unbiased
    // bsdfPdf *= rrProb;
    // bsdfRevPdf *= rrProb;

    pathState.accMISWThis =
        MIS(cosWo / bsdfPdf) * (pathState.accMISWThis * MIS(bsdfRevPdf) + pathState.accMISWPrev);
    pathState.accMISWPrev = MIS(inverse(bsdfPdf));
    pathState.throughput = pathState.throughput.cwiseProduct(bsdfContrib);
    return buffer;
}


static void EmitFromCamera(const ADFloat time,
                           const bool isStatic,
                           const ADMatrix4x4 &sampleToCam,
                           const ADAnimatedTransform &camToWorld,
                           const ADFloat screenPixelCount,
                           const ADFloat camDist,
                           const ADVector2 screenPos,
                           ADRay &ray,
                           ADBidirPathState &pathState) 
{
    ADRay centerRay;
    SamplePrimary(sampleToCam,
                  camToWorld,
                  ADVector2(Const<ADFloat>(0.5), Const<ADFloat>(0.5)),
                  time,
                  isStatic,
                  centerRay);
    SamplePrimary(sampleToCam, camToWorld, screenPos, time, isStatic, ray);
    ADVector3 camOrg = centerRay.org;
    ADVector3 camDir = centerRay.dir;
    ADVector3 org = ray.org;
    ADVector3 dir = ray.dir;
    ADFloat cosAtCamera = Dot(camDir, dir);
    ADFloat imagePointToCameraDist = camDist / cosAtCamera;
    ADFloat imageToSolidAngleFactor = square(imagePointToCameraDist) / cosAtCamera;
    ADFloat cameraPdf = imageToSolidAngleFactor;
    pathState.throughput = ADVector3(Const<ADFloat>(1.0), Const<ADFloat>(1.0), Const<ADFloat>(1.0));
    pathState.accMISWPrev = MIS(screenPixelCount / cameraPdf);
    pathState.accMISWThis = Const<ADFloat>(0.0);
    pathState.lensContrib =
        ADVector3(Const<ADFloat>(1.0), Const<ADFloat>(1.0), Const<ADFloat>(1.0));
}

static void HandleHitLight(const ADBSphere &sceneSphere,
                           const ADFloat *buffer,
                           const ADVector3 &dir,
                           const ADFloat time,
                           ADBidirPathState &pathState) 
{
    ADVector3 emission;
    ADFloat directPdf;
    ADFloat emissionPdf;
    buffer = Emission(buffer,
                      sceneSphere,
                      dir,
                      pathState.isect.shadingNormal,
                      time,
                      emission,
                      directPdf,
                      emissionPdf);

    pathState.throughput = pathState.throughput.cwiseProduct(emission);
    ADFloat lightPickProb;
    buffer = Deserialize(buffer, lightPickProb);
    directPdf *= lightPickProb;
    emissionPdf *= lightPickProb;

    // accMISWPrev magically accounts for measure conversion
    ADFloat wCamera =
        MIS(directPdf) * pathState.accMISWPrev + MIS(emissionPdf) * pathState.accMISWThis;
    ADFloat misWeight = inverse(Float(1.0) + wCamera);
    pathState.throughput *= misWeight;
}

static void DirectLighting(const int camDepth,
                           const ADBSphere &sceneSphere,
                           const ADFloat *buffer,
                           const ADFloat time,
                           const bool isStatic,
                           ADBidirPathState &pathState,
                           const ADVector2 directLightRndParam) 
{
    ADVector3 dirToLight;
    ADVector3 lightContrib;
    ADFloat cosAtLight;
    ADFloat directPdf;
    ADFloat emissionPdf;
    ADFloat lightType;
    Deserialize(buffer, lightType);
    buffer = SampleDirect(buffer,
                          sceneSphere,
                          pathState.isect.position,
                          pathState.isect.shadingNormal,
                          directLightRndParam,
                          time,
                          isStatic,
                          dirToLight,
                          lightContrib,
                          cosAtLight,
                          directPdf,
                          emissionPdf);

    ADVector3 bsdfContrib;
    ADFloat cosToLight;
    ADFloat bsdfPdf;
    ADFloat bsdfRevPdf;
    const ADFloat *bsdfBuffer = buffer;
    buffer = EvaluateBSDF(false,
                          bsdfBuffer,
                          pathState.wi,
                          pathState.isect.shadingNormal,
                          dirToLight,
                          pathState.st,
                          bsdfContrib,
                          cosToLight,
                          bsdfPdf,
                          bsdfRevPdf);

    ADFloat lightPickProb;
    buffer = Deserialize(buffer, lightPickProb);

    // Currently we ignore russian roulette in MIS computation
    // (because it is tricky to get reverse probability correct), should still be unbiased
    // bsdfPdf *= rrProb;
    // bsdfRevPdf *= rrProb;

    ADVector3 lensContrib = pathState.lensContrib;
    if (camDepth == 1) {
        const ADIntersection &isect = pathState.isect;
        ADVector3 impBsdfContrib;
        ADFloat impCosWo, impBsdfPdf, impBsdfRevPdf;
        EvaluateBSDF(true,
                     bsdfBuffer,
                     dirToLight,
                     pathState.isect.shadingNormal,
                     pathState.wi,
                     pathState.st,
                     impBsdfContrib,
                     impCosWo,
                     impBsdfPdf,
                     impBsdfRevPdf);
        // Adjoint BSDF correction
        const ADFloat factor = ShadingNormalCorrection<true>(dirToLight, isect, pathState.wi);
        impBsdfContrib *= factor;
        lensContrib = lensContrib.cwiseProduct(impBsdfContrib);
    }

    pathState.throughput = pathState.throughput.cwiseProduct(bsdfContrib);
    pathState.throughput = pathState.throughput.cwiseProduct(lightContrib) * inverse(lightPickProb);
    std::vector<CondExprCPtr> ret = CreateCondExprVec(1);
    BeginIf(Eq(lightType, Float(LightType::PointLight)), ret);
    { SetCondOutput({Const<ADFloat>(0.0)}); }
    BeginElse();
    {
        ADFloat wLight = MIS(bsdfPdf / (lightPickProb * directPdf));
        SetCondOutput({wLight});
    }
    EndIf();
    ADFloat wLight = ret[0];
    ADFloat wCamera = MIS(emissionPdf * cosToLight / (directPdf * cosAtLight)) *
                      (pathState.accMISWPrev + pathState.accMISWThis * MIS(bsdfRevPdf));
    ADFloat misWeight = inverse(wLight + Float(1.0) + wCamera);
    pathState.throughput *= misWeight;
}

static void ConnectVertex(const int camDepth,
                          const ADFloat *buffer,
                          const ADFloat *lgtBSDFBuffer,
                          const ADBidirPathState &lgtPathState,
                          ADBidirPathState &camPathState) 
{
    ADVector3 dirToLight = lgtPathState.isect.position - camPathState.isect.position;
    ADFloat distSq = LengthSquared(dirToLight);
    ADFloat dist = sqrt(distSq);
    dirToLight *= inverse(dist);

    ADVector3 camBsdfFactor;
    ADFloat cosCamera, camBsdfPdf, camBsdfRevPdf;
    const ADFloat *camBsdfBuffer = buffer;
    buffer = EvaluateBSDF(false,
                          camBsdfBuffer,
                          camPathState.wi,
                          camPathState.isect.shadingNormal,
                          dirToLight,
                          camPathState.st,
                          camBsdfFactor,
                          cosCamera,
                          camBsdfPdf,
                          camBsdfRevPdf);

    // Currently we ignore russian roulette in MIS computation, should still be unbiased
    // camBsdfPdf *= rrProb;
    // camBsdfRevPdf *= rrProb;

    ADVector3 lgtBsdfFactor;
    ADFloat cosLight, lgtBsdfPdf, lgtBsdfRevPdf;
    EvaluateBSDF(true,
                 lgtBSDFBuffer,
                 lgtPathState.wi,
                 lgtPathState.isect.shadingNormal,
                 -dirToLight,
                 lgtPathState.st,
                 lgtBsdfFactor,
                 cosLight,
                 lgtBsdfPdf,
                 lgtBsdfRevPdf);

    ADFloat lgtFactor =
        ShadingNormalCorrection<true>(lgtPathState.wi, lgtPathState.isect, -dirToLight);
    lgtBsdfFactor *= lgtFactor;

    // Currently we ignore russian roulette in MIS computation
    // (because it is tricky to get reverse probability correct), should still be unbiased
    // lgtBsdfPdf *= rrProb;
    // lgtBsdfRevPdf *= rrProb;

    ADFloat geometryTerm = inverse(distSq);
    if (camDepth == 0) {
        camPathState.lensContrib = lgtBsdfFactor.cwiseProduct(camBsdfFactor) * geometryTerm;
    } else if (camDepth == 1) {
        const ADIntersection &isect = camPathState.isect;
        ADVector3 impBsdfContrib;
        ADFloat impCosWo, impBsdfPdf, impBsdfRevPdf;
        EvaluateBSDF(true,
                     camBsdfBuffer,
                     dirToLight,
                     isect.shadingNormal,
                     camPathState.wi,
                     camPathState.st,
                     impBsdfContrib,
                     impCosWo,
                     impBsdfPdf,
                     impBsdfRevPdf);
        // Adjoint BSDF correction
        const ADFloat factor = ShadingNormalCorrection<true>(dirToLight, isect, camPathState.wi);
        impBsdfContrib *= factor;
        camPathState.lensContrib = camPathState.lensContrib.cwiseProduct(impBsdfContrib);
    }

    // Convert pdfs to area pdf
    ADFloat camBsdfDirPdfA = camBsdfPdf * cosLight * geometryTerm;
    ADFloat lgtBsdfDirPdfA = lgtBsdfPdf * cosCamera * geometryTerm;

    ADFloat wLight = MIS(camBsdfDirPdfA) *
                     (lgtPathState.accMISWPrev + lgtPathState.accMISWThis * MIS(lgtBsdfRevPdf));
    ADFloat wCamera = MIS(lgtBsdfDirPdfA) *
                      (camPathState.accMISWPrev + camPathState.accMISWThis * MIS(camBsdfRevPdf));
    ADFloat misWeight = inverse(wLight + Float(1.0) + wCamera);

    camPathState.throughput = lgtPathState.throughput.cwiseProduct(camPathState.throughput);
    camPathState.throughput = camPathState.throughput.cwiseProduct(camBsdfFactor);
    camPathState.throughput =
        camPathState.throughput.cwiseProduct(lgtBsdfFactor) * (geometryTerm * misWeight);
}

std::string GetFuncNameBidir(const int camDepth, const int lightDepth, const PathFuncMode mode) 
{
    std::string baseName =
        "evaluate_path_bidir_" + std::to_string(camDepth) + "_" + std::to_string(lightDepth);
    if (mode == PathFuncMode::Full) {
        return baseName + "_full";
    } else if (mode == PathFuncMode::Static) {
        return baseName + "_static";
    } else {
        return baseName + "_lens";
    }
    assert(false);
    return baseName;
}

std::string GetFuncNameBidirMALA(const int camDepth, const int lightDepth, const PathFuncMode mode) 
{
    std::string baseName = 
        "evaluate_path_bidir_mala_" + std::to_string(camDepth) + "_" + std::to_string(lightDepth);
    if (mode == PathFuncMode::Full) {
        return baseName + "_full";
    } else if (mode == PathFuncMode::Static) {
        return baseName + "_static";
    } else {
        return baseName + "_lens";
    }
    assert(false);
    return baseName;
}

std::string RegisterPathFuncBidir(const int maxCamDepth,
                                  const int maxLightDepth,
                                  const PathFuncMode mode,
                                  Library &lib) 
{
    auto lensParamsArg = Argument("lens", 2);
    auto primaryParamsArg = Argument("primary", GetPrimaryParamSize(maxCamDepth, maxLightDepth));
    auto sceneParamsArg = Argument("scene", GetSceneSerializedSize());
    auto vertParamsArg = Argument("vertParams", GetVertParamSize(maxCamDepth, maxLightDepth));
    const std::string funcName = GetFuncNameBidir(maxCamDepth, maxLightDepth, mode);
    std::cout << "Compiling function " << funcName << std::endl;
    auto func =
        BeginFunction(funcName, {lensParamsArg, primaryParamsArg, sceneParamsArg, vertParamsArg});
    auto lensParams = lensParamsArg.GetExprVec();
    auto primaryParams = primaryParamsArg.GetExprVec();
    auto sceneParams = sceneParamsArg.GetExprVec();
    auto vertParams = vertParamsArg.GetExprVec();

    const ADFloat *sceneBuffer = &sceneParams[0];
    ADFloat useLightCoordinateSampling;
    sceneBuffer = Deserialize(sceneBuffer, useLightCoordinateSampling);
    ADMatrix4x4 sampleToCam;
    sceneBuffer = Deserialize(sceneBuffer, sampleToCam);
    ADAnimatedTransform camToWorld;
    sceneBuffer = Deserialize(sceneBuffer, camToWorld);
    ADFloat screenPixelCount;
    sceneBuffer = Deserialize(sceneBuffer, screenPixelCount);
    ADFloat camDist;
    sceneBuffer = Deserialize(sceneBuffer, camDist);
    ADBSphere sceneSphere;
    sceneBuffer = Deserialize(sceneBuffer, sceneSphere);

    const ADFloat *buffer = &vertParams[0];
    int primaryIdx = 0;
    ADFloat time = primaryParams[primaryIdx++];

    const bool isStatic = mode == PathFuncMode::Static;

    ADVector3 lensVertexPos;
    buffer = Deserialize(buffer, lensVertexPos);
    const ADFloat *lgtBSDFBuffer = nullptr;
    ADBidirPathState lightPathState;
    ADVector3 contrib(Const<ADFloat>(0.0), Const<ADFloat>(0.0), Const<ADFloat>(0.0));
    ADVector2 screenPos{lensParams[0], lensParams[1]};
    if (maxLightDepth > 1) {
        ADFloat lightPickProb;
        buffer = Deserialize(buffer, lightPickProb);
        ADRay ray;
        ADVector2 rndParamPos, rndParamDir;
        rndParamPos[0] = primaryParams[primaryIdx++];
        rndParamPos[1] = primaryParams[primaryIdx++];
        rndParamDir[0] = primaryParams[primaryIdx++];
        rndParamDir[1] = primaryParams[primaryIdx++];
        ADFloat lightType;
        Deserialize(buffer, lightType);
        buffer = EmitFromLight(buffer,
                               sceneSphere,
                               lightPickProb,
                               time,
                               isStatic,
                               rndParamPos,
                               rndParamDir,
                               ray,
                               lightPathState);
        ADVector3 prevLensContrib(Const<ADFloat>(0.0), Const<ADFloat>(0.0), Const<ADFloat>(0.0));
        for (int lgtDepth = 0; lgtDepth < maxLightDepth - 1; lgtDepth++) {
            buffer =
                Intersect(buffer, ray, time, isStatic, lightPathState.isect, lightPathState.st);
            ADFloat bsdfDiscrete, useAbsoluteParam;
            buffer = Deserialize(buffer, bsdfDiscrete);
            buffer = Deserialize(buffer, useAbsoluteParam);

            // If we do full BDPT with non-pinhole cameras we need to handle camera hit here
            lightPathState.wi = -ray.dir;

            if (lgtDepth == 0) {
                ConvertMISLightEmit(lightType, ray, lightPathState);
            } else {
                ConvertMIS(ray, lightPathState);
            }

            if (lgtDepth == maxLightDepth - 2) {
                if (maxCamDepth == 1) {
                    ConnectToCamera(sampleToCam,
                                    camToWorld,
                                    screenPixelCount,
                                    camDist,
                                    buffer,
                                    time,
                                    isStatic,
                                    prevLensContrib,
                                    ray.org,
                                    lightPathState);
                    if (mode == PathFuncMode::Lens) {
                        contrib = lightPathState.lensContrib;
                    } else {
                        contrib = lightPathState.throughput;
                    }
                }
                lgtBSDFBuffer = buffer;
                buffer += GetMaxBSDFSerializedSize();
                break;
            }

            ADVector2 bsdfRndParam;
            bsdfRndParam[0] = primaryParams[primaryIdx++];
            bsdfRndParam[1] = primaryParams[primaryIdx++];
            if (mode == PathFuncMode::Lens && lgtDepth == maxLightDepth - 3 && maxCamDepth == 1) {
                // NO-OP
            } else {
                buffer = BSDFSampling<true, false>(lgtDepth,
                                                   buffer,
                                                   bsdfRndParam,
                                                   bsdfDiscrete,
                                                   useAbsoluteParam,
                                                   time,
                                                   isStatic,
                                                   useLightCoordinateSampling,
                                                   false,
                                                   lightPathState,
                                                   ray.dir);
            }
            prevLensContrib = lightPathState.lensContrib;
            ADFloat rrWeight;
            buffer = Deserialize(buffer, rrWeight);
            lightPathState.throughput *= rrWeight;
            ray.org = lightPathState.isect.position;
        }
    }

    if (maxCamDepth > 1) {
        ADVector2 screenPos;
        if (mode == PathFuncMode::Lens) {
            screenPos[0] = lensParams[0];
            screenPos[1] = lensParams[1];
            primaryIdx += 2;
        } else {
            screenPos[0] = primaryParams[primaryIdx++];
            screenPos[1] = primaryParams[primaryIdx++];
        }
        ADRay ray;
        ADBidirPathState camPathState;
        EmitFromCamera(time,
                       isStatic,
                       sampleToCam,
                       camToWorld,
                       screenPixelCount,
                       camDist,
                       screenPos,
                       ray,
                       camPathState);

        for (int camDepth = 0; camDepth < maxCamDepth - 1; camDepth++) {
            buffer = Intersect(buffer, ray, time, isStatic, camPathState.isect, camPathState.st);
            camPathState.wi = -ray.dir;

            if (camDepth == maxCamDepth - 2 && maxLightDepth == 0) {
                ADFloat lightType;
                Deserialize(buffer, lightType);
                ConvertMISLightHit(lightType, ray, camPathState);
                HandleHitLight(sceneSphere, buffer, ray.dir, time, camPathState);
                if (mode == PathFuncMode::Lens) {
                    contrib = camPathState.lensContrib;
                } else {
                    contrib = camPathState.throughput;
                }
                break;
            }

            ConvertMIS(ray, camPathState);

            if (mode == PathFuncMode::Lens && camDepth == 1) {
                camPathState.lensContrib *=
                    inverse(DistanceSquared(camPathState.isect.position, ray.org));
            }

            if (camDepth == maxCamDepth - 2) {
                if (maxLightDepth == 1) {
                    ADVector2 directLightRndParam;
                    directLightRndParam[0] = primaryParams[primaryIdx++];
                    directLightRndParam[1] = primaryParams[primaryIdx++];
                    DirectLighting(camDepth,
                                   sceneSphere,
                                   buffer,
                                   time,
                                   isStatic,
                                   camPathState,
                                   directLightRndParam);
                } else {  // maxLgtDepth > 1
                    ConnectVertex(camDepth, buffer, lgtBSDFBuffer, lightPathState, camPathState);
                }
                if (mode == PathFuncMode::Lens) {
                    contrib = camPathState.lensContrib;
                } else {
                    contrib = camPathState.throughput;
                }
                break;
            }

            ADVector2 bsdfRndParam;
            bsdfRndParam[0] = primaryParams[primaryIdx++];
            bsdfRndParam[1] = primaryParams[primaryIdx++];
            ADFloat bsdfDiscrete, useAbsoluteParam;
            buffer = Deserialize(buffer, bsdfDiscrete);
            buffer = Deserialize(buffer, useAbsoluteParam);
            if (mode == PathFuncMode::Lens && camDepth == 0) {
                // buffer = PerturbLensFirstVertex(buffer, lensVertexPos, camPathState, ray.dir);
            } else {
                bool doLightCoordinateSampling = maxLightDepth == 0 && camDepth == maxCamDepth - 3;
                buffer = BSDFSampling<false, false>(camDepth,
                                                    buffer,
                                                    bsdfRndParam,
                                                    bsdfDiscrete,
                                                    useAbsoluteParam,
                                                    time,
                                                    isStatic,
                                                    useLightCoordinateSampling,
                                                    doLightCoordinateSampling,
                                                    camPathState,
                                                    ray.dir);
            }
            ADFloat rrWeight;
            buffer = Deserialize(buffer, rrWeight);
            camPathState.throughput *= rrWeight;
            ray.org = camPathState.isect.position;
        }
    }
    ADFloat logLumValue = log(Luminance(contrib));
    EndFunction({{"logLumValue", {logLumValue}}});

    ExpressionCPtrVec wrt;
    if (mode == PathFuncMode::Full) {
        wrt = primaryParams;
    } else if (mode == PathFuncMode::Static) {
        wrt = ExpressionCPtrVec(primaryParams.begin() + 1, primaryParams.end());
    } else {
        assert(mode == PathFuncMode::Lens);
        wrt = lensParams;
    }

    lib.RegisterFunc(func);
    lib.RegisterFuncDerv(func, wrt, logLumValue);
    return func->name;
}

std::string RegisterPathFuncBidirMALA(const int maxCamDepth, 
                                   const int maxLightDepth, 
                                   const PathFuncMode mode, 
                                   Library &lib)
{
    auto lensParamsArg = Argument("lens", 2);
    auto primaryParamsArg = Argument("primary", GetPrimaryParamSize(maxCamDepth, maxLightDepth));
    auto sceneParamsArg = Argument("scene", GetSceneSerializedSize());
    auto vertParamsArg = Argument("vertParams", GetVertParamSize(maxCamDepth, maxLightDepth));
    const std::string funcName = GetFuncNameBidirMALA(maxCamDepth, maxLightDepth, mode);
    std::cout << "Compiling function (MALA) " << funcName << std::endl;
    auto func =
        BeginFunction(funcName, {lensParamsArg, primaryParamsArg, sceneParamsArg, vertParamsArg});
    auto lensParams = lensParamsArg.GetExprVec();
    auto primaryParams = primaryParamsArg.GetExprVec();
    auto sceneParams = sceneParamsArg.GetExprVec();
    auto vertParams = vertParamsArg.GetExprVec();

    const ADFloat *sceneBuffer = &sceneParams[0];
    ADFloat useLightCoordinateSampling;
    sceneBuffer = Deserialize(sceneBuffer, useLightCoordinateSampling);
    ADMatrix4x4 sampleToCam;
    sceneBuffer = Deserialize(sceneBuffer, sampleToCam);
    ADAnimatedTransform camToWorld;
    sceneBuffer = Deserialize(sceneBuffer, camToWorld);
    ADFloat screenPixelCount;
    sceneBuffer = Deserialize(sceneBuffer, screenPixelCount);
    ADFloat camDist;
    sceneBuffer = Deserialize(sceneBuffer, camDist);
    ADBSphere sceneSphere;
    sceneBuffer = Deserialize(sceneBuffer, sceneSphere);

    const ADFloat *buffer = &vertParams[0];
    int primaryIdx = 0;
    ADFloat time = primaryParams[primaryIdx++];

    const bool isStatic = mode == PathFuncMode::Static;

    ADVector3 lensVertexPos;
    buffer = Deserialize(buffer, lensVertexPos);
    const ADFloat *lgtBSDFBuffer = nullptr;
    ADBidirPathState lightPathState;
    ADVector3 contrib(Const<ADFloat>(0.0), Const<ADFloat>(0.0), Const<ADFloat>(0.0));
    ADVector2 screenPos{lensParams[0], lensParams[1]};
    if (maxLightDepth > 1) {
        ADFloat lightPickProb;
        buffer = Deserialize(buffer, lightPickProb);
        ADRay ray;
        ADVector2 rndParamPos, rndParamDir;
        rndParamPos[0] = primaryParams[primaryIdx++];
        rndParamPos[1] = primaryParams[primaryIdx++];
        rndParamDir[0] = primaryParams[primaryIdx++];
        rndParamDir[1] = primaryParams[primaryIdx++];
        ADFloat lightType;
        Deserialize(buffer, lightType);
        buffer = EmitFromLight(buffer,
                               sceneSphere,
                               lightPickProb,
                               time,
                               isStatic,
                               rndParamPos,
                               rndParamDir,
                               ray,
                               lightPathState);
        ADVector3 prevLensContrib(Const<ADFloat>(0.0), Const<ADFloat>(0.0), Const<ADFloat>(0.0));
        for (int lgtDepth = 0; lgtDepth < maxLightDepth - 1; lgtDepth++) {
            buffer =
                Intersect(buffer, ray, time, isStatic, lightPathState.isect, lightPathState.st);
            ADFloat bsdfDiscrete, useAbsoluteParam;
            buffer = Deserialize(buffer, bsdfDiscrete);
            buffer = Deserialize(buffer, useAbsoluteParam);

            // If we do full BDPT with non-pinhole cameras we need to handle camera hit here
            lightPathState.wi = -ray.dir;

            if (lgtDepth == 0) {
                ConvertMISLightEmit(lightType, ray, lightPathState);
            } else {
                ConvertMIS(ray, lightPathState);
            }

            if (lgtDepth == maxLightDepth - 2) {
                if (maxCamDepth == 1) {
                    ConnectToCamera(sampleToCam,
                                    camToWorld,
                                    screenPixelCount,
                                    camDist,
                                    buffer,
                                    time,
                                    isStatic,
                                    prevLensContrib,
                                    ray.org,
                                    lightPathState);
                    if (mode == PathFuncMode::Lens) {
                        contrib = lightPathState.lensContrib;
                    } else {
                        contrib = lightPathState.throughput;
                    }
                }
                lgtBSDFBuffer = buffer;
                buffer += GetMaxBSDFSerializedSize();
                break;
            }

            ADVector2 bsdfRndParam;
            bsdfRndParam[0] = primaryParams[primaryIdx++];
            bsdfRndParam[1] = primaryParams[primaryIdx++];
            if (mode == PathFuncMode::Lens && lgtDepth == maxLightDepth - 3 && maxCamDepth == 1) {
                // NO-OP
            } else {
                buffer = BSDFSampling<true, false>(lgtDepth,
                                                   buffer,
                                                   bsdfRndParam,
                                                   bsdfDiscrete,
                                                   useAbsoluteParam,
                                                   time,
                                                   isStatic,
                                                   useLightCoordinateSampling,
                                                   false,
                                                   lightPathState,
                                                   ray.dir);
            }
            prevLensContrib = lightPathState.lensContrib;
            ADFloat rrWeight;
            buffer = Deserialize(buffer, rrWeight);
            lightPathState.throughput *= rrWeight;
            ray.org = lightPathState.isect.position;
        }
    }

    if (maxCamDepth > 1) {
        ADVector2 screenPos;
        if (mode == PathFuncMode::Lens) {
            screenPos[0] = lensParams[0];
            screenPos[1] = lensParams[1];
            primaryIdx += 2;
        } else {
            screenPos[0] = primaryParams[primaryIdx++];
            screenPos[1] = primaryParams[primaryIdx++];
        }
        ADRay ray;
        ADBidirPathState camPathState;
        EmitFromCamera(time,
                       isStatic,
                       sampleToCam,
                       camToWorld,
                       screenPixelCount,
                       camDist,
                       screenPos,
                       ray,
                       camPathState);

        for (int camDepth = 0; camDepth < maxCamDepth - 1; camDepth++) {
            buffer = Intersect(buffer, ray, time, isStatic, camPathState.isect, camPathState.st);
            camPathState.wi = -ray.dir;

            if (camDepth == maxCamDepth - 2 && maxLightDepth == 0) {
                ADFloat lightType;
                Deserialize(buffer, lightType);
                ConvertMISLightHit(lightType, ray, camPathState);
                HandleHitLight(sceneSphere, buffer, ray.dir, time, camPathState);
                if (mode == PathFuncMode::Lens) {
                    contrib = camPathState.lensContrib;
                } else {
                    contrib = camPathState.throughput;
                }
                break;
            }

            ConvertMIS(ray, camPathState);

            if (mode == PathFuncMode::Lens && camDepth == 1) {
                camPathState.lensContrib *=
                    inverse(DistanceSquared(camPathState.isect.position, ray.org));
            }

            if (camDepth == maxCamDepth - 2) {
                if (maxLightDepth == 1) {
                    ADVector2 directLightRndParam;
                    directLightRndParam[0] = primaryParams[primaryIdx++];
                    directLightRndParam[1] = primaryParams[primaryIdx++];
                    DirectLighting(camDepth,
                                   sceneSphere,
                                   buffer,
                                   time,
                                   isStatic,
                                   camPathState,
                                   directLightRndParam);
                } else {  // maxLgtDepth > 1
                    ConnectVertex(camDepth, buffer, lgtBSDFBuffer, lightPathState, camPathState);
                }
                if (mode == PathFuncMode::Lens) {
                    contrib = camPathState.lensContrib;
                } else {
                    contrib = camPathState.throughput;
                }
                break;
            }

            ADVector2 bsdfRndParam;
            bsdfRndParam[0] = primaryParams[primaryIdx++];
            bsdfRndParam[1] = primaryParams[primaryIdx++];
            ADFloat bsdfDiscrete, useAbsoluteParam;
            buffer = Deserialize(buffer, bsdfDiscrete);
            buffer = Deserialize(buffer, useAbsoluteParam);
            if (mode == PathFuncMode::Lens && camDepth == 0) {
                //buffer = PerturbLensFirstVertex(buffer, lensVertexPos, camPathState, ray.dir);
            } else {
                bool doLightCoordinateSampling = maxLightDepth == 0 && camDepth == maxCamDepth - 3;
                buffer = BSDFSampling<false, false>(camDepth,
                                                    buffer,
                                                    bsdfRndParam,
                                                    bsdfDiscrete,
                                                    useAbsoluteParam,
                                                    time,
                                                    isStatic,
                                                    useLightCoordinateSampling,
                                                    doLightCoordinateSampling,
                                                    camPathState,
                                                    ray.dir);
            }
            ADFloat rrWeight;
            buffer = Deserialize(buffer, rrWeight);
            camPathState.throughput *= rrWeight;
            ray.org = camPathState.isect.position;
        }
    }



    ADFloat logLumValue = log(Luminance(contrib));
    EndFunction({{"logLumValue", {logLumValue}}});

    ExpressionCPtrVec wrt;
    if (mode == PathFuncMode::Full) {
        wrt = primaryParams;
    } else if (mode == PathFuncMode::Static) {
        wrt = ExpressionCPtrVec(primaryParams.begin() + 1, primaryParams.end());
    } else {
        assert(mode == PathFuncMode::Lens);
        wrt = lensParams;
    }

    lib.RegisterFunc2(func);
    lib.RegisterFuncDerv2(func, wrt, logLumValue);
    return func->name;

}

std::shared_ptr<Library> CompilePathFuncLibrary(const bool bidirectional,
                                                const int maxDepth,
                                                std::shared_ptr<Library> *library) 
{
    std::shared_ptr<Library> pathLib =
        library == nullptr
            ? std::make_shared<Library>(GetLibPath(), bidirectional ? "pathlibbidir" : "pathlib")
            : *library;
    std::cout << "Compiling path function libraries..." << std::endl;
    for (int maxCamDepth = 1; maxCamDepth <= maxDepth + 1; maxCamDepth++) {
        for (int maxLgtDepth = 0; maxLgtDepth <= (bidirectional ? maxDepth : 1); maxLgtDepth++) {
            if (maxCamDepth + maxLgtDepth <= 2 || (maxCamDepth + maxLgtDepth - 1) > maxDepth) {
                continue;
            }

            if (bidirectional) {
                // RegisterPathFuncBidir(maxCamDepth, maxLgtDepth, PathFuncMode::Full, *pathLib);
                RegisterPathFuncBidir(maxCamDepth, maxLgtDepth, PathFuncMode::Static, *pathLib);
                if (maxCamDepth + maxLgtDepth > 3) {
                    // RegisterPathFuncBidir(maxCamDepth, maxLgtDepth, PathFuncMode::Lens, *pathLib);
                }
            } else {
                // RegisterPathFunc(maxCamDepth, maxLgtDepth, PathFuncMode::Full, *pathLib);
                RegisterPathFunc(maxCamDepth, maxLgtDepth, PathFuncMode::Static, *pathLib);
                if (maxCamDepth + maxLgtDepth > 3) {
                    // RegisterPathFunc(maxCamDepth, maxLgtDepth, PathFuncMode::Lens, *pathLib);
                }
            }
        }
    }
    pathLib->Link();
    std::cout << "Compiled" << std::endl;
    return pathLib;
}

std::shared_ptr<Library> CompilePathFuncLibrary2(const int maxDepth, 
                                                 std::shared_ptr<Library> *library) {
    std::shared_ptr<Library> pathLib = 
        library == nullptr 
            ? std::make_shared<Library>(GetLibPath(), "pathlibbidir_mala")
            : *library;
    std::cout << "Compiling path function libraries (MALA)..." << std::endl; 
    for (int maxCamDepth = 1; maxCamDepth <= maxDepth + 1; maxCamDepth++) {
        for (int maxLgtDepth = 0; maxLgtDepth <= maxDepth; maxLgtDepth++) {
            if (maxCamDepth + maxLgtDepth <= 2 || (maxCamDepth + maxLgtDepth - 1) > maxDepth) {
                continue;
            }

            // RegisterPathFuncBidirMALA(maxCamDepth, maxLgtDepth, PathFuncMode::Full, *pathLib);
            RegisterPathFuncBidirMALA(maxCamDepth, maxLgtDepth, PathFuncMode::Static, *pathLib);
            if (maxCamDepth + maxLgtDepth > 3) {
                // RegisterPathFuncBidirMALA(maxCamDepth, maxLgtDepth, PathFuncMode::Lens, *pathLib);
            }
           
        }
    }
    pathLib->Link();
    std::cout << "Compiled (MALA) " << std::endl;
    return pathLib;
}

std::shared_ptr<const PathFuncLib> BuildPathFuncLibrary(const bool bidirectional,
                                                        const int maxDepth) 
{
    std::shared_ptr<Library> pathLib =
        std::make_shared<Library>(GetLibPath(), bidirectional ? "pathlibbidir" : "pathlib");
    if (!pathLib->IsLinked()) {
        pathLib = CompilePathFuncLibrary(bidirectional, maxDepth, &pathLib);
    }

    PathFuncMap staticFuncMap;
    PathFuncDervMap staticFuncDervMap;
    for (int maxCamDepth = 1; maxCamDepth <= maxDepth + 1; maxCamDepth++) {
        for (int maxLgtDepth = 0; maxLgtDepth <= (bidirectional ? maxDepth : 1); maxLgtDepth++) {
            if (maxCamDepth + maxLgtDepth <= 2 || (maxCamDepth + maxLgtDepth - 1) > maxDepth) {
                continue;
            }

            std::string staticFuncName =
                bidirectional ? GetFuncNameBidir(maxCamDepth, maxLgtDepth, PathFuncMode::Static)
                              : GetFuncName(maxCamDepth, maxLgtDepth, PathFuncMode::Static);
            {
                PathFunc f = (PathFunc)pathLib->GetFunc(staticFuncName);
                if (f != nullptr) {
                    staticFuncMap[{maxCamDepth, maxLgtDepth}] = f;
                } else {
                    std::cout << "[Warning] Can't find function:" << staticFuncName << std::endl;
                }
            }
            {
                PathFuncDerv f = (PathFuncDerv)pathLib->GetFuncDerv(staticFuncName);
                if (f != nullptr) {
                    staticFuncDervMap[{maxCamDepth, maxLgtDepth}] = f;
                } else {
                    std::cout << "[Warning] Can't find derivatives function:" << staticFuncName
                              << std::endl;
                }
            }

        }
    }

    return std::make_shared<PathFuncLib>(PathFuncLib{maxDepth,
                                                     pathLib,
                                                     staticFuncMap,
                                                     staticFuncDervMap});
}

std::shared_ptr<const PathFuncLib> BuildPathFuncLibrary2(const int maxDepth) {
    std::shared_ptr<Library> pathLib = 
        std::make_shared<Library>(GetLibPath(), "pathlibbidir_mala");
    if (!pathLib->IsLinked()) {
        pathLib = CompilePathFuncLibrary2(maxDepth, &pathLib);
    }

    PathFuncMap staticFuncMap;
    PathFuncDervMap staticFuncDervMap;
    for (int maxCamDepth = 1; maxCamDepth <= maxDepth + 1; maxCamDepth++) {
        for (int maxLgtDepth = 0; maxLgtDepth <= maxDepth; maxLgtDepth++) {
            if (maxCamDepth + maxLgtDepth <= 2 || (maxCamDepth + maxLgtDepth - 1) > maxDepth) {
                continue;
            }
 

            std::string staticFuncName = GetFuncNameBidirMALA(maxCamDepth, maxLgtDepth, PathFuncMode::Static);
            {
                PathFunc f = (PathFunc)pathLib->GetFunc(staticFuncName);
                if (f != nullptr) {
                    staticFuncMap[{maxCamDepth, maxLgtDepth}] = f;
                } else {
                    std::cout << "[Warning] Can't find function:" << staticFuncName << std::endl;
                }
            }
            {
                PathFuncDerv f = (PathFuncDerv)pathLib->GetFuncDerv(staticFuncName);
                if (f != nullptr) {
                    staticFuncDervMap[{maxCamDepth, maxLgtDepth}] = f;
                } else {
                    std::cout << "[Warning] Can't find derivatives function:" << staticFuncName
                              << std::endl;
                }
            } 

        }
    }

    return std::make_shared<PathFuncLib>(PathFuncLib{maxDepth, 
                                                     pathLib,
                                                     staticFuncMap,
                                                     staticFuncDervMap});
}