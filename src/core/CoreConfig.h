#pragma once
#include <cstddef>

#ifndef CORE_MAX_MODULES
#define CORE_MAX_MODULES 15
#endif

#ifndef CORE_MAX_LISTENERS_PER_EVENT
#define CORE_MAX_LISTENERS_PER_EVENT 4
#endif

namespace CoreConfig {
    constexpr int MAX_MODULES = CORE_MAX_MODULES;
    constexpr int MAX_LISTENERS_PER_EVENT = CORE_MAX_LISTENERS_PER_EVENT;
}

