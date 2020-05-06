#pragma once 
#include "mutation.h"

struct SmallStep : public Mutation {
    Float Mutate(const MLTState &mltState,
                 const Float normalization,
                 MarkovState &currentState,
                 MarkovState &proposalState,
                 RNG &rng,
                 Chain *chain = NULL) override;

    std::vector<SubpathContrib> spContribs;
    Vector offset;
};

Float SmallStep::Mutate(const MLTState &mltState,
                        const Float normalization,
                        MarkovState &currentState,
                        MarkovState &proposalState,
                        RNG &rng,
                        Chain *chain) 
{
    const Scene *scene = mltState.scene;
    spContribs.clear();

    Float a = Float(1.0);
    assert(currentState.valid);
    proposalState.path = currentState.path;
    const Float stdDev = scene->options->perturbStdDev;
    std::normal_distribution<Float> normDist(Float(0.0), stdDev);
    lastMutationType = MutationType::Small;
    const auto perturbPathFunc = mltState.perturbPathFunc;
    const int dim = GetDimension(currentState.path); 
    if (!offset.size())    offset.resize(2 * scene->options->maxDepth);
    for (int i = 0; i < dim; i++) {
        offset[i] = normDist(rng);
    }
    perturbPathFunc(scene, offset, proposalState.path, spContribs, rng);
    proposalState.gaussianInitialized = false;
    if (spContribs.size() > 0) {
        assert(spContribs.size() == 1);
        proposalState.spContrib = spContribs[0];
        a = Clamp(proposalState.spContrib.ssScore / currentState.spContrib.ssScore,
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