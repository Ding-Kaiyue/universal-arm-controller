#pragma once

#include "reactive_task_types.hpp"

namespace arm_controller::controller::reactive_task {

class ReactiveTaskLocalReferenceManager {
public:
    struct Config {
        double lookahead_min_sec{0.03};
        double lookahead_max_sec{0.15};
        double lookahead_release_error_m{0.18};
        double lookahead_full_error_band_m{0.12};
        double max_reference_lead_sec{0.25};
        double max_constrained_reference_lead_sec{0.10};
    };

    ReactiveTaskLocalReferenceManager();
    explicit ReactiveTaskLocalReferenceManager(Config cfg);

    bool compute(
        const LocalReferenceInput& input,
        LocalReferenceOutput* output) const;

private:
    double computeProgressScale(const LocalReferenceInput& input) const;
    static double smoothstep01(double x);

    Config cfg_;
};

}  // namespace arm_controller::controller::reactive_task
