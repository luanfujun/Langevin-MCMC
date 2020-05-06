#pragma once 

#include "mutation.h"
#include "mutation_small.h"

using DervFuncMap = std::unordered_map<std::pair<int, int>, PathFuncDerv>;
struct H2MCSmallStep : public Mutation {
    H2MCSmallStep(const Scene *scene,
                  const int maxDervDepth,
                  const Float sigma);
    Float Mutate(const MLTState &mltState,
                 const Float normalization,
                 MarkovState &currentState,
                 MarkovState &proposalState,
                 RNG &rng, 
                 Chain *chain = NULL) override;
    std::vector<SubpathContrib> spContribs;
    H2MCParam h2mcParam;
    AlignedStdVector sceneParams;
    SerializedSubpath ssubPath;
    SmallStep isotropicSmallStep;

    AlignedStdVector vGrad;
    AlignedStdVector vHess;
};

H2MCSmallStep::H2MCSmallStep(const Scene *scene,
                             const int maxDervDepth,
                             const Float sigma) 
    : h2mcParam(sigma)
{
    sceneParams.resize(GetSceneSerializedSize());
    Serialize(scene, &sceneParams[0]);
    ssubPath.primary.resize(GetPrimaryParamSize(maxDervDepth, maxDervDepth));
    ssubPath.vertParams.resize(GetVertParamSize(maxDervDepth, maxDervDepth));
}

Float H2MCSmallStep::Mutate(const MLTState &mltState,
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
        Float a =
            isotropicSmallStep.Mutate(mltState, normalization, currentState, proposalState, rng);
        lastMutationType = isotropicSmallStep.lastMutationType;
        return a;
    }
    spContribs.clear();
    Float a = Float(1.0);
    assert(currentState.valid);
    lastMutationType = MutationType::H2MCSmall;
    const auto perturbPathFunc = mltState.perturbPathFunc;
    const int dim = GetDimension(currentState.path);
    auto initGaussian = [&](MarkovState &state) {
        const SubpathContrib &cspContrib = state.spContrib;
        const auto &fmap = mltState.staticFuncDervMap;
        auto funcIt = fmap.find({cspContrib.camDepth, cspContrib.lightDepth});
        const int dim = GetDimension(state.path);
        if (funcIt != fmap.end()) {
            vGrad.resize(dim, Float(0.0));
            vHess.resize(dim * dim, Float(0.0));
            if (cspContrib.ssScore > Float(1e-15)) {
                PathFuncDerv dervFunc = funcIt->second;
                assert(dervFunc != nullptr);
                Serialize(scene, state.path, ssubPath);
                dervFunc(&cspContrib.screenPos[0],
                         &ssubPath.primary[0],
                         &sceneParams[0],
                         &ssubPath.vertParams[0],
                         &vGrad[0],
                         &vHess[0]);
                if (!IsFinite(vGrad) || !IsFinite(vHess)) {
                    // Usually caused by floating point round-off error
                    // (or, of course, bugs)
                    std::fill(vGrad.begin(), vGrad.end(), Float(0.0));
                    std::fill(vHess.begin(), vHess.end(), Float(0.0));
                }
                assert(IsFinite(vGrad));
                assert(IsFinite(vHess));
            }
            ComputeGaussian(h2mcParam, cspContrib.ssScore, vGrad, vHess, state.gaussian);
        } else {
            IsotropicGaussian(dim, h2mcParam.sigma, state.gaussian);
        }
        state.gaussianInitialized = true;
    };

    if (!currentState.gaussianInitialized) {
        initGaussian(currentState);
    }

    assert(currentState.gaussianInitialized);

    Vector offset(dim);
    GenerateSample(currentState.gaussian, offset, rng);
    proposalState.path = currentState.path;
    perturbPathFunc(scene, offset, proposalState.path, spContribs, rng);
    if (spContribs.size() > 0) {
        assert(spContribs.size() == 1);
        proposalState.spContrib = spContribs[0];
        initGaussian(proposalState);
        Float py = GaussianLogPdf(offset, currentState.gaussian);
        Float px = GaussianLogPdf(-offset, proposalState.gaussian);

        a = Clamp(std::exp(px - py) * proposalState.spContrib.ssScore /
                      currentState.spContrib.ssScore,
                  Float(0.0),
                  Float(1.0));

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