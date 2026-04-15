
#pragma once
#include <cstdint>
#include "rtos/Kernel.h"

// If you have a project-wide comm printf macro, define it before including this header.
// Otherwise we fall back to plain printf.
#ifndef COMM_PRINTF
  #include <cstdio>
  #define COMM_PRINTF(...) std::printf(__VA_ARGS__)
#endif

// Small utility to enforce minimum dwell time in a state to avoid chatter.
class StateDwellGuard {
public:
    StateDwellGuard() : _last_change_ms(0) {}

    // Call when the state _actually_ changes.
    void mark_changed(uint32_t now_ms = rtos_now_ms()) { _last_change_ms = now_ms; }

    // Return true if we may change the state now (i.e., min_dwell_ms elapsed).
    bool can_change(uint32_t min_dwell_ms, uint32_t now_ms = rtos_now_ms()) const {
        uint32_t dt = (now_ms >= _last_change_ms) ? (now_ms - _last_change_ms) : 0;
        return dt >= min_dwell_ms;
    }

    uint32_t last_change_ms() const { return _last_change_ms; }

    static inline uint32_t rtos_now_ms() {
        // Works on Mbed OS 6.x
        return Kernel::get_ms_count();
    }

private:
    uint32_t _last_change_ms;
};
