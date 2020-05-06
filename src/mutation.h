#pragma once 

#include "global_cache.h"

#define REMOVE_OUTLIERS
#define OUTLIER_WEAK_REJECT_CNT    10000
#define OUTLIER_STRONG_REJECT_CNT  1000
#define OUTLIER_RATIO_THRESHOLD    Float(30.0)
 

enum class MutationType { Large, Small, H2MCSmall, MALASmall };

struct Chain; 
struct LargeStep;

struct Mutation {

    virtual Float Mutate(const MLTState &mltState,
                         const Float normalization,
                         MarkovState &currentState,
                         MarkovState &proposalState,
                         RNG &rng,
                         Chain *chain = NULL) = 0;

    MutationType lastMutationType;
};

struct Chain {
    std::vector<Float> pss, last_pss, v1, v2, g, M; 
    std::vector<Float> curr_new_v1, curr_new_v2, curr_new_g;
    std::vector<Float> prop_new_v1, prop_new_v2, prop_new_g;
    
    Path path; 
    SubpathContrib spContrib;
    Float pathWeight;

    bool buffered = false;
    Float ss; 
    int chainId, t = 0;
    
    bool queried = false;
    GlobalCache *globalCache = NULL;
};