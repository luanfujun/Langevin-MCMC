#pragma once

#include "commondef.h"

#include <chrono>
#include <ctime>

struct Timer {
    std::chrono::time_point<std::chrono::system_clock> last;
};

inline Float Tick(Timer &timer) {
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = now - timer.last;
    Float ret = elapsed.count();
    timer.last = now;
    return ret;
}
