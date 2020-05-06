#pragma once

#include "commondef.h"
#include "utils.h"

#include <mutex>
#include <functional>
#include <atomic>

// From https://github.com/mmp/pbrt-v3/blob/master/src/core/parallel.h

class AtomicFloat {
    public:
    explicit AtomicFloat(Float v = 0) {
        bits = FloatToBits(v);
    }
    operator Float() const {
        return BitsToFloat(bits);
    }
    Float operator=(Float v) {
        bits = FloatToBits(v);
        return v;
    }
    void Add(Float v) {
#if defined(DOUBLE_PRECISION)
        uint64_t oldBits = bits, newBits;
#elif defined(SINGLE_PRECISION)
        uint32_t oldBits = bits, newBits;
#endif
        do {
            newBits = FloatToBits(BitsToFloat(oldBits) + v);
        } while (!bits.compare_exchange_weak(oldBits, newBits));
    }

    private:
#if defined(DOUBLE_PRECISION)
    std::atomic<uint64_t> bits;
#elif defined(SINGLE_PRECISION)
    std::atomic<uint32_t> bits;
#endif
};

void ParallelFor(const std::function<void(int64_t)> &func, int64_t count, int chunkSize = 1);
extern thread_local int threadIndex;
void ParallelFor(std::function<void(Vector2i)> func, const Vector2i count);
int MaxThreadIndex();
int NumSystemCores();
void TerminateWorkerThreads();
