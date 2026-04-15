#include "charger_presence_filter.h"

ChargerPresenceFilter::ChargerPresenceFilter(const Params& p)
    : params_(p) {
    reset(false);
}

void ChargerPresenceFilter::reset(bool present) {
    present_     = present;
    enter_start_ = time_point{};
    exit_start_  = time_point{};
    last_flip_   = clock::now();   // start hold timer from "now"
}

bool ChargerPresenceFilter::enter_condition(int32_t cur_mA) const {
    // Treat tiny currents as zero (noise near 0)
    if (std::abs(cur_mA) <= params_.idle_deadband_mA) return false;

    if (params_.negative_current_is_charging) {
        // charging when current is negative
        return (cur_mA <= -params_.enter_threshold_abs_mA);
    } else {
        // charging when current is positive
        return (cur_mA >=  params_.enter_threshold_abs_mA);
    }
}

bool ChargerPresenceFilter::exit_condition(int32_t cur_mA) const {
    // Treat tiny currents as zero (noise near 0)
    if (std::abs(cur_mA) <= params_.idle_deadband_mA) return true;

    if (params_.negative_current_is_charging) {
        // exit when magnitude falls below exit threshold (toward zero or discharge)
        return (cur_mA > -params_.exit_threshold_abs_mA);
    } else {
        return (cur_mA <  params_.exit_threshold_abs_mA);
    }
}

bool ChargerPresenceFilter::update(int32_t pack_current_mA, bool& changed) {
    changed = false;

    const auto now       = clock::now();
    const bool in_hold   = (ms_since(last_flip_) < params_.hold_ms);

    if (!present_) {
        if (enter_condition(pack_current_mA)) {
            if (enter_start_.time_since_epoch().count() == 0) {
                enter_start_ = now;
            }
            if (!in_hold && ms_since(enter_start_) >= params_.debounce_ms) {
                present_   = true;
                changed    = true;
                last_flip_ = now;
                // clear the other edge timer
                exit_start_ = time_point{};
            }
        } else {
            enter_start_ = time_point{}; // condition not steady -> reset window
        }
    } else {
        if (exit_condition(pack_current_mA)) {
            if (exit_start_.time_since_epoch().count() == 0) {
                exit_start_ = now;
            }
            if (!in_hold && ms_since(exit_start_) >= params_.debounce_ms) {
                present_   = false;
                changed    = true;
                last_flip_ = now;
                // clear the other edge timer
                enter_start_ = time_point{};
            }
        } else {
            exit_start_ = time_point{};
        }
    }

    return present_;
}
