#pragma once
#include "mbed.h"

/**
 * ChargerPresenceFilter
 * ---------------------
 * Debounces "charger present" based on pack current.
 * Assumes negative current means charging by default (configurable).
 */
class ChargerPresenceFilter {
public:
    struct Params {
        // Enter/exit thresholds are absolute (in mA).
        int      enter_threshold_abs_mA;      // |I| must exceed this to ENTER "present"
        int      exit_threshold_abs_mA;       // |I| must go below this to EXIT  "present"
        uint32_t debounce_ms;                 // condition must hold for this long
        uint32_t hold_ms;                     // hold state for at least this long after a flip
        int      idle_deadband_mA;            // treat |I| <= deadband as zero (noise window)
        bool     negative_current_is_charging;// true: negative means charging (default)

        constexpr Params(int enter = 500,
                         int exit  = 100,
                         uint32_t debounce = 1000,
                         uint32_t hold     = 2500,
                         int deadband      = 50,
                         bool neg_is_chg   = true)
            : enter_threshold_abs_mA(enter),
              exit_threshold_abs_mA(exit),
              debounce_ms(debounce),
              hold_ms(hold),
              idle_deadband_mA(deadband),
              negative_current_is_charging(neg_is_chg) {}
    };

    explicit ChargerPresenceFilter(const Params& p = Params());

    // Force state and clear timers.
    void reset(bool present);

    // Feed latest pack current in mA. Returns current "present" state.
    // 'changed' becomes true if state flips in this call.
    bool update(int32_t pack_current_mA, bool& changed);

    bool present() const { return present_; }
    const Params& params() const { return params_; }

private:
    using clock      = Kernel::Clock;
    using time_point = clock::time_point;

    static uint64_t ms_since(const time_point& t) {
        const auto now = clock::now();
        return (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - t).count();
    }

    // Conditions to start enter/exit debounce windows
    bool enter_condition(int32_t cur_mA) const;
    bool exit_condition (int32_t cur_mA) const;

    Params     params_{};
    bool       present_{false};

    time_point enter_start_{};
    time_point exit_start_{};
    time_point last_flip_{};
};
