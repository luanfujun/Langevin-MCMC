#pragma once 
#include "mutation_large.h"
#include "global_cache.h"

struct LargeStepCache : public LargeStep {
    LargeStepCache(std::shared_ptr<PiecewiseConstant1D> lengthDist, const int maxDervDepth) 
    : LargeStep(lengthDist) {
        ssubPath.primary.resize(GetPrimaryParamSize(maxDervDepth, maxDervDepth));
        ssubPath.vertParams.resize(GetVertParamSize(maxDervDepth, maxDervDepth));
    }
    Float Mutate(const MLTState &mltState,
                 const Float normalization,
                 MarkovState &currentState,
                 MarkovState &proposalState,
                 RNG &rng,
                 Chain *chain = NULL) override;
    SerializedSubpath ssubPath;
    Vector offset;
};

 
Float LargeStepCache::Mutate(const MLTState &mltState,
                        const Float normalization,
                        MarkovState &currentState,
                        MarkovState &proposalState,
                        RNG &rng,
                        Chain *chain) 
{   
    const Scene *scene = mltState.scene;
    const auto genPathFunc = mltState.genPathFunc;
    const auto perturbPathFunc = mltState.perturbPathFunc;
    assert(scene->options->bidirectional);
    assert(scene->options->largeStepMultiplexed);
    assert(scene->options->sampleFromGlobalCache);
    assert(scene->options->mala);
    lastMutationType = MutationType::Large;
    std::uniform_real_distribution<Float> uniDist(Float(0.0), Float(1.0));
    Float a = Float(1.0);
    spContribs.clear();
    Clear(proposalState.path);

    /* sample a fixed path length first */
    const int proposalLength = lengthDist->SampleDiscrete(uniDist(rng), nullptr); 
    const int proposalDim = proposalLength * 2; /* primary sample dimension */
    const int currentLength = GetPathLength(currentState.spContrib.camDepth, currentState.spContrib.lightDepth);
    const int currentDim = currentLength * 2;   /* primary sample dimension */

    /* check cache availability */
    bool proposalCacheAvailable = false; 
    bool currentCacheAvailable = false; 
    if (proposalDim >= PSS_MIN_LENGTH && proposalDim <= PSS_MAX_LENGTH) {
        if (chain->globalCache->isReady(proposalDim))    proposalCacheAvailable = true; 
    }
    if (currentDim >= PSS_MIN_LENGTH && currentDim <= PSS_MAX_LENGTH) {
        if (chain->globalCache->isReady(currentDim))     currentCacheAvailable = true; 
    }

    if (!proposalCacheAvailable || uniDist(rng) > CACHE_PROB) {
        /* sampling uniformly as in multiplexed MLT */
        int lgtLength = Clamp(int(uniDist(rng) * (proposalLength + 1)), 0, proposalLength);
        int camLength = proposalLength - lgtLength + 1;
        GenerateSubpath(scene,
                        Vector2i(-1, -1),
                        camLength,
                        lgtLength,
                        scene->options->bidirectional,
                        proposalState.path,
                        spContribs,
                        rng);
        if (spContribs.size() > 0) {
            assert(spContribs.size() <= 1);
            ToSubpath(spContribs[0].camDepth, spContribs[0].lightDepth, proposalState.path);
            GetPathPss(proposalState.path, proposalState.pss);
        }
    } else { /* sampling from the global cache */
        std::vector<Float> pss; Float pathWeight; SubpathContrib spContrib;
        chain->globalCache->sampleCache(proposalDim, 
            proposalState.path, pss, spContrib, pathWeight, rng);
        ToSubpath(spContrib.camDepth, spContrib.lightDepth, proposalState.path);
        
        std::normal_distribution<Float> normDist(Float(0.0), CACHE_SIG);
        proposalState.pss.resize(proposalDim);
        if (!offset.size())    offset.resize(2 * scene->options->maxDepth);
        for (int i = 0; i < proposalDim; i++) {
            offset[i] = normDist(rng);
            proposalState.pss[i] = Modulo(pss[i] + offset[i], Float(1.0));
        }
        perturbPathFunc(scene, offset, proposalState.path, spContribs, rng);
        assert(spContribs.size() <= 1);
    }
    proposalState.gaussianInitialized = false;
    if (spContribs.size() > 0) {
        proposalState.spContrib = spContribs[0];
        proposalState.scoreSum = spContribs[0].lsScore;

        if (currentState.valid) {
            {
                ToSubpath(currentState.spContrib.camDepth, currentState.spContrib.lightDepth, currentState.path);
                GetPathPss(currentState.path, currentState.pss);
            }
            const Float proposalJacobian = proposalState.spContrib.ssScore / proposalState.spContrib.lsScore;  
            const Float currentJacobian = currentState.spContrib.ssScore / currentState.spContrib.lsScore;
            Float proposalTechniquePickProb = inverse(Float(proposalLength) + Float(1.0));
            Float currentTechniquePickProb = inverse(Float(currentLength) + Float(1.0));
            /* Uniform sampling this path probability */
            Float proposalUniformPdf = 1.0 * proposalTechniquePickProb * proposalJacobian;
            Float currentUniformPdf = 1.0 * currentTechniquePickProb * currentJacobian;
            /* Cache sampilng this path probability */
            Float proposalCachePdf = proposalCacheAvailable ? 
                chain->globalCache->evalPdfCache(proposalDim, proposalState.pss, proposalState.path) : Float(0.0);
            Float currentCachePdf = currentCacheAvailable ? 
                chain->globalCache->evalPdfCache(currentDim, currentState.pss, currentState.path) : Float(0.0);
            /* MIS */
            Float proposalPdf = !proposalCacheAvailable ? proposalUniformPdf : 
                (1 - CACHE_PROB) * proposalUniformPdf + CACHE_PROB * proposalCachePdf;
            Float currentPdf = !currentCacheAvailable ? currentUniformPdf : 
                (1 - CACHE_PROB) * currentUniformPdf + CACHE_PROB * currentCachePdf;
            // /* M-H acceptance probability */
            a = Clamp(proposalState.spContrib.ssScore * currentPdf * lengthDist->Pmf(currentLength)
                        / (currentState.spContrib.ssScore * proposalPdf * lengthDist->Pmf(proposalLength)), 
                    Float(0.0), 
                    Float(1.0));
        }

        proposalState.toSplat.clear();
        for (const auto &spContrib : spContribs) {
            proposalState.toSplat.push_back(SplatSample{
                spContrib.screenPos, spContrib.contrib * (normalization / spContrib.lsScore)});
        }
    } else {
        a = Float(0.0);
    }
    return a;   
}