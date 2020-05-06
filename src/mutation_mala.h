#pragma once 

#include "mutation.h"
#include "mutation_small.h"
#include "h2mc.h"
#include "mala.h"
#include "fastmath.h"

using DervFuncMap = std::unordered_map<std::pair<int, int>, PathFuncDerv>;
struct MALASmallStep : public Mutation {
    MALASmallStep(const Scene *scene,
                  const int maxDervDepth);
    Float Mutate(const MLTState &mltState,
                 const Float normalization,
                 MarkovState &currentState,
                 MarkovState &proposalState,
                 RNG &rng,
                 Chain *chain = NULL) override;
    SmallStep isotropicSmallStep;
    std::vector<SubpathContrib> spContribs;
    AlignedStdVector sceneParams;
    SerializedSubpath ssubPath;
    AlignedStdVector vGrad;
};

MALASmallStep::MALASmallStep(const Scene *scene,
                             const int maxDervDepth) 
{
    sceneParams.resize(GetSceneSerializedSize());
    Serialize(scene, &sceneParams[0]);
    ssubPath.primary.resize(GetPrimaryParamSize(maxDervDepth, maxDervDepth));
    ssubPath.vertParams.resize(GetVertParamSize(maxDervDepth, maxDervDepth));
}

Float MALASmallStep::Mutate(const MLTState &mltState,
                            const Float normalization,
                            MarkovState &currentState,
                            MarkovState &proposalState,
                            RNG &rng,
                            Chain *chain) 
{
    const Scene *scene = mltState.scene;
    // Sometimes the derivatives are noisy so that the light paths
    // will "stuck" in some regions, we probabilistically switch to
    // uniform sampling to avoid stucking
    std::uniform_real_distribution<Float> uniDist(Float(0.0), Float(1.0));
    if (uniDist(rng) < scene->options->uniformMixingProbability) {
        Float a = isotropicSmallStep.Mutate(mltState, normalization, currentState, proposalState, rng);
        lastMutationType = isotropicSmallStep.lastMutationType;
        return a;
    }
    spContribs.clear();
    Float a = Float(1.0);
    assert(currentState.valid);
    lastMutationType = MutationType::MALASmall;
    const auto perturbPathFunc = mltState.perturbPathFunc;
    const int dim = GetDimension(currentState.path);
    std::normal_distribution<Float> normDist(Float(0.0), Float(1.0));
    if (!chain->buffered) {
        const int maxdim = 2 * scene->options->maxDepth;
        if (!chain->M.size()) { // malloc
            chain->M.resize(maxdim);  chain->pss.resize(maxdim);         chain->last_pss.resize(maxdim);
            chain->g.resize(maxdim);  chain->curr_new_g.resize(maxdim);  chain->prop_new_g.resize(maxdim);
            chain->v1.resize(maxdim); chain->curr_new_v1.resize(maxdim); chain->prop_new_v1.resize(maxdim);
            chain->v2.resize(maxdim); chain->curr_new_v2.resize(maxdim); chain->prop_new_v2.resize(maxdim);
        }
        std::fill(chain->pss.begin(), chain->pss.end(), Float(0.0));
        std::fill(chain->last_pss.begin(), chain->last_pss.end(), Float(0.0));
        std::fill(chain->g.begin(), chain->g.end(), Float(0.0));
        std::fill(chain->curr_new_g.begin(), chain->curr_new_g.end(), Float(0.0));  
        std::fill(chain->prop_new_g.begin(), chain->prop_new_g.end(), Float(0.0));
        std::fill(chain->v1.begin(), chain->v1.end(), Float(0.0));
        std::fill(chain->curr_new_v1.begin(), chain->curr_new_v1.end(), Float(0.0));
        std::fill(chain->prop_new_v1.begin(), chain->prop_new_v1.end(), Float(0.0));
        std::fill(chain->v2.begin(), chain->v2.end(), Float(0.0));
        std::fill(chain->curr_new_v2.begin(), chain->curr_new_v2.end(), Float(0.0));
        std::fill(chain->prop_new_v2.begin(), chain->prop_new_v2.end(), Float(0.0));
        std::fill(chain->M.begin(), chain->M.end(), Float(0.0));
        chain->buffered = true;   
        chain->queried = false;
    }

    if (!currentState.gaussianInitialized) {
        const SubpathContrib &cspContrib = currentState.spContrib;
        const auto &fmap = mltState.staticFuncDervMap;
        auto funcIt = fmap.find({cspContrib.camDepth, cspContrib.lightDepth});
        const int dim = GetDimension(currentState.path);
        
        GetPathPss(currentState.path, chain->pss);
        chain->path = currentState.path;
        chain->spContrib = currentState.spContrib;
        chain->pathWeight = currentState.spContrib.lsScore; 

        if (dim >= PSS_MIN_LENGTH && dim <= PSS_MAX_LENGTH && 
            !chain->globalCache->isReady(dim) &&
            funcIt != fmap.end()) {
            vGrad.resize(dim, Float(0.0));
            if (cspContrib.ssScore > Float(1e-10)) {
                PathFuncDerv dervFunc = funcIt->second;
                assert(dervFunc != nullptr);
                Serialize(scene, currentState.path, ssubPath);
                dervFunc(&cspContrib.screenPos[0],
                         &ssubPath.primary[0],
                         &sceneParams[0],
                         &ssubPath.vertParams[0],
                         &vGrad[0],
                         NULL);
                if (!IsFinite(vGrad)) {
                    std::fill(vGrad.begin(), vGrad.end(), Float(0.0));
                }
                assert(IsFinite(vGrad));
            }
            Float norm(0.0), drift(scene->options->malaGN);    // bounded drift for Truncated MALA
            for (int i = 0; i < dim; i++) { norm += vGrad[i] * vGrad[i]; } norm = sqrt(norm);
            for (int i = 0; i < dim; i++) { vGrad[i] *= drift / std::max(drift, norm); }
            bool first = true; 
            for (int i = 0; i < dim; i++) 
                if (chain->curr_new_v2[i] > Float(1e-10)) {
                    first = false;    break;
                }
            for (int i = 0; i < dim; i++) {
                Float g = vGrad[i]; 
                chain->curr_new_g[i] = g;
                chain->curr_new_v1[i] = first ? g : Float(0.9)   * chain->v1[i] + Float(0.1)   * g;
                chain->curr_new_v2[i] = first ? g * g : Float(0.999) * chain->v2[i] + Float(0.001) * g * g; 
                chain->M[i] = Clamp(Float(1.0) / Float(Float(1e-3) + sqrt(chain->curr_new_v2[i])), PCD_MIN, PCD_MAX);
            }
            ComputeGaussian(dim, chain->curr_new_v1, chain->curr_new_v2, chain->ss, scene->options->malaStdDev, \
                chain->M, chain->t, cspContrib.ssScore, currentState.gaussian);

        } else {
            if (dim >= PSS_MIN_LENGTH && dim <= PSS_MAX_LENGTH && 
                chain->globalCache->isReady(dim)) {
                bool reuse = false;
                if (chain->queried) {
                    Float dist_sqr(0.f);
                    for (int i = 0; i < dim; i++) {
                        Float diff = chain->pss[i] - chain->last_pss[i];
                        dist_sqr += diff * diff;
                    }
                    if (dist_sqr < dim * (PSS_REUSE_DIST * PSS_REUSE_DIST)) {
                        reuse = true; 
                    }
                }
                if (reuse) {
                    for (int i = 0; i < dim; i++) {
                        chain->M[i] = Clamp(Float(1.0) / Float(Float(1e-3) + sqrt(chain->v2[i])), PCD_MIN, PCD_MAX);
                    }
                    ComputeGaussian(dim, chain->v1, chain->v2, chain->ss, scene->options->malaStdDev, \
                        chain->M, chain->t, cspContrib.ssScore, currentState.gaussian);
                } else if (chain->globalCache->query(dim, chain->pss, chain->v1, chain->v2)) {
                    chain->queried = true; 
                    chain->last_pss = chain->pss;
                    for (int i = 0; i < dim; i++) {
                        chain->M[i] = Clamp(Float(1.0) / Float(Float(1e-3) + sqrt(chain->v2[i])), PCD_MIN, PCD_MAX);
                    }
                    ComputeGaussian(dim, chain->v1, chain->v2, chain->ss, scene->options->malaStdDev, \
                        chain->M, chain->t, cspContrib.ssScore, currentState.gaussian);
                } else {
                    IsotropicGaussian(dim, scene->options->malaStdDev, currentState.gaussian);
                }
            } else 
                IsotropicGaussian(dim, scene->options->malaStdDev, currentState.gaussian);
        }
        currentState.gaussianInitialized = true;
    }
    assert(currentState.gaussianInitialized);
    Vector offset(dim);
    GenerateSample(currentState.gaussian, offset, rng);
    proposalState.path = currentState.path;

    perturbPathFunc(scene, offset, proposalState.path, spContribs, rng);
  
    if (spContribs.size() > 0) {
        assert(spContribs.size() == 1);
        proposalState.spContrib = spContribs[0];
        
        {
            const SubpathContrib &cspContrib = proposalState.spContrib;
            const auto &fmap =  mltState.staticFuncDervMap;
            auto funcIt = fmap.find({cspContrib.camDepth, cspContrib.lightDepth});
            const int dim = GetDimension(proposalState.path);

            GetPathPss(proposalState.path, chain->pss);
            chain->path = proposalState.path; 
            chain->spContrib = proposalState.spContrib;
            chain->pathWeight = proposalState.spContrib.lsScore; 
            
            if (dim >= PSS_MIN_LENGTH && dim <= PSS_MAX_LENGTH && 
                !chain->globalCache->isReady(dim) &&
                funcIt != fmap.end()) {
                vGrad.resize(dim, Float(0.0));
                if (cspContrib.ssScore > Float(1e-10)) {
                    PathFuncDerv dervFunc = funcIt->second;
                    assert(dervFunc != nullptr);
                    Serialize(scene, proposalState.path, ssubPath);
                    dervFunc(&cspContrib.screenPos[0],
                             &ssubPath.primary[0],
                             &sceneParams[0],
                             &ssubPath.vertParams[0],
                             &vGrad[0],
                             NULL);
                    if (!IsFinite(vGrad)) {
                        std::fill(vGrad.begin(), vGrad.end(), Float(0.0));
                    }
                    assert(IsFinite(vGrad));
                }
                Float norm(0.0), drift(scene->options->malaGN);    // bounded drift for Truncated MALA
                for (int i = 0; i < dim; i++) { norm += vGrad[i] * vGrad[i]; } norm = sqrt(norm);
                for (int i = 0; i < dim; i++) { vGrad[i] *= drift / std::max(drift, norm); }
                bool first = true; 
                for (int i = 0; i < dim; i++)
                    if (chain->prop_new_v2[i] > Float(1e-10)) {
                        first = false;    break; 
                    }
                for (int i = 0; i < dim; i++) {
                    Float g = vGrad[i];
                    chain->prop_new_g[i] = g;
                    chain->prop_new_v1[i] = first ? g : Float(0.9)   * chain->v1[i] + Float(0.1)   * g;
                    chain->prop_new_v2[i] = first ? g * g : Float(0.999) * chain->v2[i] + Float(0.001) * g * g; 
                    chain->M[i] = Clamp(Float(1.0) / Float(Float(1e-3) + sqrt(chain->prop_new_v2[i])), PCD_MIN, PCD_MAX);
                }
                ComputeGaussian(dim, chain->prop_new_v1, chain->prop_new_v2, chain->ss, scene->options->malaStdDev, \
                    chain->M, chain->t, cspContrib.ssScore, proposalState.gaussian);
            } else {
                if (dim >= PSS_MIN_LENGTH && dim <= PSS_MAX_LENGTH && 
                    chain->globalCache->isReady(dim)) {
                    bool reuse = false;
                    if (chain->queried) {
                        Float dist_sqr(0.f);
                        for (int i = 0; i < dim; i++) {
                            Float diff = chain->pss[i] - chain->last_pss[i];
                            dist_sqr += diff * diff;
                        }
                        if (dist_sqr < dim * (PSS_REUSE_DIST * PSS_REUSE_DIST)) {
                            reuse = true; 
                        }
                    }
                    if (reuse) {
                        for (int i = 0; i < dim; i++) {
                            chain->M[i] = Clamp(Float(1.0) / Float(Float(1e-3) + sqrt(chain->v2[i])), PCD_MIN, PCD_MAX);
                        }
                        ComputeGaussian(dim, chain->v1, chain->v2, chain->ss, scene->options->malaStdDev, \
                            chain->M, chain->t, cspContrib.ssScore, proposalState.gaussian);
                    } else if (chain->globalCache->query(dim, chain->pss, chain->v1, chain->v2)) {
                        chain->queried = true; 
                        chain->last_pss = chain->pss;
                        for (int i = 0; i < dim; i++) {
                            chain->M[i] = Clamp(Float(1.0) / Float(Float(1e-3) + sqrt(chain->v2[i])), PCD_MIN, PCD_MAX);
                        }
                        ComputeGaussian(dim, chain->v1, chain->v2, chain->ss, scene->options->malaStdDev, \
                            chain->M, chain->t, cspContrib.ssScore, proposalState.gaussian);
                    } else {
                        IsotropicGaussian(dim, scene->options->malaStdDev, proposalState.gaussian);
                    }
                } else 
                    IsotropicGaussian(dim, scene->options->malaStdDev, proposalState.gaussian);
            }
            proposalState.gaussianInitialized = true;
        }

        Float py = GaussianLogPdf(offset, currentState.gaussian);
        Float px = GaussianLogPdf(-offset, proposalState.gaussian);
        a = Clamp(std::exp(px - py) * proposalState.spContrib.ssScore /
                      currentState.spContrib.ssScore,
                  Float(0.0),
                  Float(1.0));
        proposalState.toSplat.clear();
        for (const auto &spContrib : spContribs) {
            proposalState.toSplat.push_back(SplatSample{
                spContrib.screenPos, spContrib.contrib * normalization / spContrib.lsScore});
        }

    } else {
        a = Float(0.0);
    }
    return a;
}
