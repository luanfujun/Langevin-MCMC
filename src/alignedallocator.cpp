#include "alignedallocator.h"

#include <cstdlib>

void *AllocAligned(size_t size, size_t alignment) {
    void *ptr;
    if (posix_memalign(&ptr, alignment, size) != 0)
        ptr = nullptr;
    return ptr;
}

void FreeAligned(void *ptr) {
    if (!ptr)
        return;
    free(ptr);
}
