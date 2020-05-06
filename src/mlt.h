#pragma once

#include "direct.h"
#include "parallel.h"
#include "h2mc.h"
#include "mala.h"
#include "gaussian.h"
#include "alignedallocator.h"
#include "distribution.h"
#include <vector>
#include <deque>
#include <mutex>
#include <memory>

struct PathFuncLib;

struct MLTState {
    const Scene *scene;
    const decltype(&GeneratePathBidir) genPathFunc;
    const decltype(&PerturbPathBidir) perturbPathFunc;
    PathFuncMap staticFuncMap;
    PathFuncDervMap staticFuncDervMap;
};

struct SplatSample {
    Vector2 screenPos;
    Vector3 contrib;
};

struct MarkovState {
    bool valid;
    SubpathContrib spContrib;
    Path path;
    Float scoreSum;         
    std::vector<Float> pss;
    bool gaussianInitialized;
    Gaussian gaussian;    
    std::vector<SplatSample> toSplat;
};

static Float MLTInit(const MLTState &mltState,
                     const int64_t numInitSamples,
                     const int numChains,
                     std::vector<MarkovState> &initStates,
                     std::shared_ptr<PiecewiseConstant1D> &lengthDist) 
{
    std::cout << "Initializing mlt" << std::endl;
    Timer timer;
    Tick(timer);

    const int64_t numSamplesPerThread = numInitSamples / NumSystemCores();
    const int64_t threadsNeedExtraSamples = numInitSamples % NumSystemCores();
    const Scene *scene = mltState.scene;
    auto genPathFunc = mltState.genPathFunc;

    std::mutex mStateMutex;
    struct LightMarkovState {
        RNG rng;
        int camDepth;
        int lightDepth;
        Float lsScore;
    };
    std::vector<LightMarkovState> mStates;
    Float totalScore(Float(0.0));
    std::vector<Float> lengthContrib;
    ParallelFor([&](const int threadId) {
        RNG rng(threadId + scene->options->seedOffset);
        int64_t numSamplesThisThread =
            numSamplesPerThread + ((threadIndex < threadsNeedExtraSamples) ? 1 : 0);
        std::vector<SubpathContrib> spContribs;
        Path path;
        for (int sampleIdx = 0; sampleIdx < numSamplesThisThread; sampleIdx++) {
            spContribs.clear();
            RNG rngCheckpoint = rng;
            Clear(path);
            const int minPathLength = std::max(scene->options->minDepth, 3);
            genPathFunc(scene,
                        Vector2i(-1, -1),
                        minPathLength,
                        scene->options->maxDepth,
                        path,
                        spContribs,
                        rng);

            std::lock_guard<std::mutex> lock(mStateMutex);
            for (const auto &spContrib : spContribs) {
                totalScore += spContrib.lsScore;
                const int pathLength = GetPathLength(spContrib.camDepth, spContrib.lightDepth);
                if (pathLength >= int(lengthContrib.size())) {
                    lengthContrib.resize(pathLength + 1, Float(0.0));
                }
                lengthContrib[pathLength] += spContrib.lsScore;
                mStates.emplace_back(LightMarkovState{
                    rngCheckpoint, spContrib.camDepth, spContrib.lightDepth, spContrib.lsScore});
            }
        }
    }, NumSystemCores());

    lengthDist = std::make_shared<PiecewiseConstant1D>(&lengthContrib[0], lengthContrib.size());

    if (int(mStates.size()) < numChains) {
        Error(
            "MLT initialization failed, consider using a larger number of initial samples or "
            "smaller number of chains");
    }

    // Equal-spaced seeding (See p.340 in Veach's thesis)
    std::vector<Float> cdf(mStates.size() + 1);
    cdf[0] = Float(0.0);
    for (int i = 0; i < (int)mStates.size(); i++) {
        cdf[i + 1] = cdf[i] + mStates[i].lsScore;
    }
    const Float interval = cdf.back() / Float(numChains);
    std::uniform_real_distribution<Float> uniDist(Float(0.0), interval);
    RNG rng(mStates.size());
    Float pos = uniDist(rng);
    int cdfPos = 0;
    initStates.reserve(numChains);
    std::vector<SubpathContrib> spContribs;
    for (int i = 0; i < (int)numChains; i++) {
        while (pos > cdf[cdfPos]) {
            cdfPos = std::min(cdfPos + 1, int(mStates.size()) - 1);
        }
        initStates.push_back(MarkovState{false});
        MarkovState &state = initStates.back();
        spContribs.clear();
        Clear(state.path);
        RNG rngCheckpoint = mStates[cdfPos - 1].rng;
        genPathFunc(scene,
                    Vector2i(-1, -1),
                    std::max(scene->options->minDepth, 3),
                    scene->options->maxDepth,
                    state.path,
                    spContribs,
                    rngCheckpoint);
        state.scoreSum = Float(0.0);
        for (const auto &spContrib : spContribs) {
            state.scoreSum += spContrib.lsScore;
            if (spContrib.camDepth == mStates[cdfPos - 1].camDepth &&
                spContrib.lightDepth == mStates[cdfPos - 1].lightDepth) {
                state.spContrib = spContrib;
            }
        }
        ToSubpath(state.spContrib.camDepth, state.spContrib.lightDepth, state.path);
        GetPathPss(state.path, state.pss);
        state.gaussianInitialized = false;
        pos += interval;
    }

    Float invNumInitSamples = inverse(Float(numInitSamples));
    Float elapsed = Tick(timer);
    std::cout << "Elapsed time:" << elapsed << std::endl;
    return Float(totalScore) * invNumInitSamples;
}

void MLT(const Scene *scene, const std::shared_ptr<const PathFuncLib> pathFuncLib);
