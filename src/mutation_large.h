#pragma once 
#include "mutation.h"

struct LargeStep : public Mutation {
    LargeStep(std::shared_ptr<PiecewiseConstant1D> lengthDist) : lengthDist(lengthDist) {
    }
    Float Mutate(const MLTState &mltState,
                 const Float normalization,
                 MarkovState &currentState,
                 MarkovState &proposalState,
                 RNG &rng,
                 Chain *chain = NULL) override;
    std::shared_ptr<PiecewiseConstant1D> lengthDist;
    std::vector<SubpathContrib> spContribs;
    std::vector<Float> contribCdf;
    Float lastScoreSum = Float(1.0);
    Float lastScore = Float(1.0);
};


/**
 *  We implement a hybrid algorithm that combines Primary Sample Space MLT [Kelemen et al. 2002]
 *  and Multiplxed MLT (MMLT) [Hachisuka et al. 2014].  Specifically, the state of our Markov
 *  chain only represents one of the N^2 pairs connection as in MMLT.  During the "large
 *  step" mutations, instead of choosing the camera and light subpath lengths a priori as in
 *  MMLT, we sample all pairs of connections, and probabilistically pick one based on their
 *  contributions (similar to Multiple-try Metropolis).  During the "small step" mutations,
 *  we fix the camera and light subpath lengths of the state.
 */

Float LargeStep::Mutate(const MLTState &mltState,
                        const Float normalization,
                        MarkovState &currentState,
                        MarkovState &proposalState,
                        RNG &rng,
                        Chain *chain) 
{
    lastMutationType = MutationType::Large;
    std::uniform_real_distribution<Float> uniDist(Float(0.0), Float(1.0));
    const Scene *scene = mltState.scene;
    const auto genPathFunc = mltState.genPathFunc;
    Float a = Float(1.0);
    spContribs.clear();
    Clear(proposalState.path);
    if (scene->options->largeStepMultiplexed) {
        int length = lengthDist->SampleDiscrete(uniDist(rng), nullptr);
        int lgtLength = scene->options->bidirectional
                            ? Clamp(int(uniDist(rng) * (length + 1)), 0, length)
                            : Clamp(int(uniDist(rng) * 2), 0, 1);
        int camLength = length - lgtLength + 1;
        GenerateSubpath(scene,
                        Vector2i(-1, -1),
                        camLength,
                        lgtLength,
                        scene->options->bidirectional,
                        proposalState.path,
                        spContribs,
                        rng);
        assert(spContribs.size() <= 1);
    } else {
        genPathFunc(scene,
                    Vector2i(-1, -1),
                    std::max(scene->options->minDepth, 3),
                    scene->options->maxDepth,
                    proposalState.path,
                    spContribs,
                    rng);
    }
    proposalState.gaussianInitialized = false;
    if (spContribs.size() > 0) {
        contribCdf.clear();
        contribCdf.push_back(Float(0.0));
        for (const auto &spContrib : spContribs) {
            contribCdf.push_back(contribCdf.back() + spContrib.lsScore);
        }
        const Float scoreSum = contribCdf.back();
        const Float invSc = inverse(scoreSum);
        std::for_each(contribCdf.begin(), contribCdf.end(), [invSc](Float &cdf) { cdf *= invSc; });

        const auto it = std::upper_bound(contribCdf.begin(), contribCdf.end(), uniDist(rng));
        int64_t contribId =
            Clamp(int64_t(it - contribCdf.begin() - 1), int64_t(0), int64_t(spContribs.size() - 1));
        proposalState.spContrib = spContribs[contribId];
        proposalState.scoreSum = scoreSum;

        if (currentState.valid) {
            if (scene->options->largeStepMultiplexed) {
                int currentLength = GetPathLength(currentState.spContrib.camDepth,
                                                  currentState.spContrib.lightDepth);
                int proposalLength = GetPathLength(proposalState.spContrib.camDepth,
                                                   proposalState.spContrib.lightDepth);
                Float invProposalTechniquesPmf = scene->options->bidirectional
                                                     ? (Float(proposalLength) + Float(1.0))
                                                     : Float(2.0);
                Float invCurrentTechniquesPmf = scene->options->bidirectional
                                                    ? (Float(currentLength) + Float(1.0))
                                                    : Float(2.0);
                a = Clamp((invProposalTechniquesPmf * proposalState.spContrib.lsScore /
                           lengthDist->Pmf(proposalLength)) /
                              (invCurrentTechniquesPmf * currentState.spContrib.lsScore /
                               lengthDist->Pmf(currentLength)),
                          Float(0.0),
                          Float(1.0));
            } else {
                // In general, we do not have the "scoreSum" of currentState, since small steps only
                // mutate one subpath
                // To address this, we introduce an augmented space that only contains large step
                // states.
                const Float probProposal =
                    (proposalState.spContrib.lsScore / proposalState.scoreSum);
                const Float probLast = (lastScore / lastScoreSum);
                a = Clamp((proposalState.spContrib.lsScore * probLast) /
                              (currentState.spContrib.lsScore * probProposal),
                          Float(0.0),
                          Float(1.0));
            }
        }

        proposalState.toSplat.clear();
        for (const auto &spContrib : spContribs) {
            proposalState.toSplat.push_back(
                SplatSample{spContrib.screenPos, spContrib.contrib * (normalization / scoreSum)});
        }
    } else {
        a = Float(0.0);
    }
    return a;
}