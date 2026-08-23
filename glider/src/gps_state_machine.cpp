#include "glider/gkf/gps_state_machine.hpp"

namespace Glider
{

const char* to_string(GpsTrustState state)
{
    switch (state) {
        case GpsTrustState::kDistrust: return "DISTRUST";
        case GpsTrustState::kCandidate: return "CANDIDATE";
        case GpsTrustState::kTrust: return "TRUST";
    }
    return "UNKNOWN";
}

GpsStateMachine::GpsStateMachine(const GpsStateMachineConfig& config) : config_(config)
{
    if (config_.required_consecutive < 1) {
        config_.required_consecutive = 1;
    }
}

void GpsStateMachine::reset()
{
    state_ = GpsTrustState::kDistrust;
    consecutive_ = 0;
    last_timestamp_ = 0;
    has_sample_ = false;
}

GpsTrustState GpsStateMachine::update(int64_t timestamp, int8_t status)
{
    const int64_t dt = timestamp - last_timestamp_;

    // Backwards time or a gap in the stream breaks the streak.
    if (has_sample_ && (dt < 0 || (config_.timeout > 0 && dt > config_.timeout))) {
        consecutive_ = 0;
    }

    last_timestamp_ = timestamp;
    has_sample_ = true;

    if (status < config_.min_status) {
        consecutive_ = 0;
    } else if (consecutive_ < config_.required_consecutive) {
        ++consecutive_;
    }

    if (consecutive_ == 0) {
        state_ = GpsTrustState::kDistrust;
    } else if (consecutive_ >= config_.required_consecutive) {
        state_ = GpsTrustState::kTrust;
    } else {
        state_ = GpsTrustState::kCandidate;
    }

    return state_;
}

GpsTrustState GpsStateMachine::checkTimeout(int64_t now)
{
    if (!has_sample_ || config_.timeout <= 0) {
        return state_;
    }

    const int64_t dt = now - last_timestamp_;
    if (dt > config_.timeout) {
        consecutive_ = 0;
        state_ = GpsTrustState::kDistrust;
    }

    return state_;
}

}  // namespace Glider
