#include "mlt.h"
#include "mutation.h"
#include "mutation_large.h"
#include "mutation_large_cache.h"
#include "mutation_small.h"
#include "mutation_h2mc.h"
#include "mutation_mala.h"
#include "fastmath.h"
#include <omp.h>
/**
 *  We implement a hybrid algorithm that combines Primary Sample Space MLT [Kelemen et al. 2002]
 *  and Multiplxed MLT (MMLT) [Hachisuka et al. 2014].  Specifically, the state of our Markov
 *  chain only represents one of the N^2 pairs connection as in MMLT.  During the "large
 *  step" mutations, instead of choosing the camera and light subpath lengthes a priori as in
 *  MMLT, we sample all pairs of connections, and probabilistically pick one based on their
 *  contributions (similar to Multiple-try Metropolis).  During the "small step" mutations,
 *  we fix the camera and light subpath lengthes of the state.
 */

void MLT(const Scene *scene, const std::shared_ptr<const PathFuncLib> pathFuncLib) {
    const MLTState mltState{scene,
                            GeneratePathBidir,
                            PerturbPathBidir,
                            pathFuncLib->staticFuncMap,
                            pathFuncLib->staticDervFuncMap};
    const int spp = scene->options->spp;
    std::shared_ptr<const Camera> camera = scene->camera;
    const Float largeStepProb = scene->options->largeStepProbability;
    std::shared_ptr<Image3> film = camera->film;
    film->Clear();
    const int pixelHeight = GetPixelHeight(camera.get());
    const int pixelWidth = GetPixelWidth(camera.get());
    SampleBuffer directBuffer(pixelWidth, pixelHeight);
    DirectLighting(scene, directBuffer);

    const int64_t numPixels = int64_t(pixelWidth) * int64_t(pixelHeight);
    const int64_t totalSamples = int64_t(spp) * numPixels;
    const int64_t numChains = scene->options->numChains;
    const int64_t numSamplesPerChain = totalSamples / numChains;
    const int64_t chainsNeedExtraSamples = numSamplesPerChain % numChains;

    std::vector<MarkovState> initStates;
    std::shared_ptr<PiecewiseConstant1D> lengthDist;
    const Float avgScore =
        MLTInit(mltState, scene->options->numInitSamples, numChains, initStates, lengthDist);
    std::cout << "Average brightness:" << avgScore << std::endl;
    const Float normalization = avgScore;

    ProgressReporter reporter(totalSamples);
    const int reportInterval = 16384;
    int intervalImgId = 1;

    GlobalCache globalCache; 

    SampleBuffer indirectBuffer(pixelWidth, pixelHeight);
    Timer timer;
    Tick(timer);

    std::chrono::time_point<std::chrono::system_clock> _start_t = std::chrono::system_clock::now();
    ParallelFor([&](const int chainId) {
        const int seed = chainId + scene->options->seedOffset;
        RNG rng(seed);
        std::uniform_real_distribution<Float> uniDist(Float(0.0), Float(1.0));

        int64_t numSamplesThisChain =
            numSamplesPerChain + ((chainId < chainsNeedExtraSamples) ? 1 : 0);
        std::vector<Float> contribCdf;
        MarkovState currentState = initStates[chainId];
        MarkovState proposalState{false};
        int64_t adjacentReject = 0;
        std::unique_ptr<LargeStep> largeStep = scene->options->sampleFromGlobalCache && scene->options->mala ? 
            std::unique_ptr<LargeStepCache>(new LargeStepCache(lengthDist, pathFuncLib->maxDepth)): // sample from global cache
            std::unique_ptr<LargeStep>(new LargeStep(lengthDist)); 
        std::unique_ptr<Mutation> smallStep =
            scene->options->h2mc // H2MC
                ? std::unique_ptr<Mutation>(new H2MCSmallStep(scene,
                                                              pathFuncLib->maxDepth,
                                                              scene->options->perturbStdDev))
                : 
                ( 
                scene->options->mala // LMC
                    ? std::unique_ptr<Mutation>(new MALASmallStep(scene, 
                                                                  pathFuncLib->maxDepth))
                : std::unique_ptr<Mutation>(new SmallStep()) // Isotropic   
                );
                
        Chain chain;
        chain.chainId = chainId;    
        chain.globalCache = &globalCache;
        chain.ss = scene->options->malaStepsize;
        for (int sampleIdx = 0; sampleIdx < numSamplesThisChain; sampleIdx++) {
            Float a = Float(1.0);
            bool isLargeStep = false;
            // In online exploration stage, use a smaller largestep prob to ensure MALA chain learns better pc. matrix
            // In H2MC case, this is disabled and lsScale will always be 1.0 
            Float lsScale = (sampleIdx > numSamplesThisChain * LS_RATIO) ? scene->options->largeStepProbScale : Float(1.0);
            if (!currentState.valid || uniDist(rng) < largeStepProb * lsScale) {
                isLargeStep = true;
                a = largeStep->Mutate(mltState, normalization, currentState, proposalState, rng, &chain);
            } else {
                a = smallStep->Mutate(mltState, normalization, currentState, proposalState, rng, &chain);
            }
            if (currentState.valid && a < Float(1.0)) {
                for (const auto splat : currentState.toSplat) {
                    Splat(indirectBuffer, splat.screenPos, (Float(1.0) - a) * splat.contrib);
                }
            }
            if (a > Float(0.0)) {
                for (const auto splat : proposalState.toSplat) {
                    Splat(indirectBuffer, splat.screenPos, a * splat.contrib);
                }
            }
            if (a > Float(0.0) && uniDist(rng) <= a) {
                ToSubpath(proposalState.spContrib.camDepth,
                          proposalState.spContrib.lightDepth,
                          proposalState.path);
                std::swap(currentState, proposalState);
                currentState.valid = true;
                adjacentReject = 0;
                if (isLargeStep) {
                    if (chain.buffered && chain.pathWeight > Float(1e-10)) {
                        int dim = GetDimension(proposalState.path); 
                        if (dim >= PSS_MIN_LENGTH && dim <= PSS_MAX_LENGTH && !globalCache.isReady(dim)) { // update global cache
                            std::lock_guard<std::mutex> global_cache_lock(globalCache.getMutex(dim));
                            globalCache.push(dim, chain.pss, chain.v1, chain.v2, 
                                chain.path, chain.spContrib, chain.pathWeight);
                        }
                    }
                    largeStep->lastScoreSum = currentState.scoreSum;
                    largeStep->lastScore = currentState.spContrib.lsScore;
                    currentState.gaussianInitialized = false;
                    chain.buffered = false;
                } else {
                    if (smallStep->lastMutationType == MutationType::MALASmall) {
                        chain.g = chain.prop_new_g;
                        chain.v1 = chain.prop_new_v1;
                        chain.v2 = chain.prop_new_v2;
                        chain.t += 1; 
                        chain.buffered = true; 
                        currentState.gaussianInitialized = true; 
                    }
                }
            } else {
                // Sometimes the derivatives are noisy so that the light paths
                // will "stuck" in some regions, we reset the Markov chain state
                // when a light path is "stuck" 
                #ifdef REMOVE_OUTLIERS // addresses outliers
                    adjacentReject += 1; 
                    bool strongReject = currentState.spContrib.lsScore > OUTLIER_RATIO_THRESHOLD * normalization;
                    if (adjacentReject > OUTLIER_WEAK_REJECT_CNT || 
                        (strongReject && adjacentReject > OUTLIER_STRONG_REJECT_CNT)) {
                        int _chainId = chainId, cnt = 0; 
                        while (true) { 
                            currentState = initStates[_chainId];
                            if (currentState.spContrib.lsScore < OUTLIER_RATIO_THRESHOLD * normalization)
                                break;
                            _chainId = (_chainId + sampleIdx + cnt++) % numChains;
                        }
                        currentState.valid = false;
                        currentState.gaussianInitialized = false; 
                        currentState.toSplat.clear();
                        proposalState.valid = false;
                        proposalState.gaussianInitialized = false; 
                        proposalState.toSplat.clear();
                        proposalState.pss.clear(); 
                        Clear(proposalState.path);
                        chain.buffered = false;
                    }
                #endif 
            }
            if (sampleIdx > 0 && (sampleIdx % reportInterval == 0)) {
                reporter.Update(reportInterval);
                const int reportIntervalSpp = scene->options->reportIntervalSpp;
                if (threadIndex == 0 && reportIntervalSpp > 0) {
                    if (reporter.GetWorkDone() >
                        uint64_t(numPixels * reportIntervalSpp * intervalImgId)) {

                        std::chrono::time_point<std::chrono::system_clock> _end_t = std::chrono::system_clock::now();
                        std::chrono::duration<double> _timeuse = _end_t - _start_t;
                        Float timeuse = _timeuse.count();

                        SampleBuffer buffer(pixelWidth, pixelHeight);
                        Float directWeight = scene->options->directSpp > 0 ? inverse(Float(scene->options->directSpp)) : Float(0.0);
                        Float indirectWeight = reportIntervalSpp * intervalImgId > 0 ? inverse(Float(reportIntervalSpp * intervalImgId)) : Float(0.0);
                        MergeBuffer(directBuffer, directWeight, indirectBuffer, indirectWeight, buffer);
                        BufferToFilm(buffer, film.get());
                        WriteImage("intermediate.exr", film.get());
                        std::string hdr2ldr = "hdrmanip --tonemap filmic -o intermediate.png intermediate.exr";
                        system(hdr2ldr.c_str());
                        intervalImgId++;
                    }
                }
            }
        }
        reporter.Update(numSamplesThisChain % reportInterval);
    }, numChains); 

    TerminateWorkerThreads();
    reporter.Done();
    Float elapsed = Tick(timer);
    std::cout << "Elapsed time:" << elapsed << std::endl;
   
    SampleBuffer buffer(pixelWidth, pixelHeight);
    Float directWeight = scene->options->directSpp > 0 ? inverse(Float(scene->options->directSpp)) : Float(0.0);
    Float indirectWeight = spp > 0 ? inverse(Float(spp)) : Float(0.0);
    MergeBuffer(directBuffer, directWeight, indirectBuffer, indirectWeight, buffer);
    BufferToFilm(buffer, film.get());
    std::string outputNameHDR = scene->outputName + "_timeuse_" + std::to_string(elapsed) + "s.exr";
    std::string outputNameLDR = scene->outputName + "_timeuse_" + std::to_string(elapsed) + "s.png";
    WriteImage(outputNameHDR, GetFilm(scene->camera.get()).get());
    std::string hdr2ldr = std::string("hdrmanip --tonemap filmic -o ") + outputNameLDR + " " + outputNameHDR;
    system(hdr2ldr.c_str());
    std::cout << "Done!" << std::endl;
}

