#pragma once

#include <cstdint>

namespace Glider
{

enum class GpsTrustState
{
  kDistrust = 0,
  kCandidate,
  kTrust
};

const char* to_string(GpsTrustState state);

struct GpsStateMachineConfig
{
    int8_t min_status = 2;
    int required_consecutive = 10;
    int32_t timeout = 0;
};

class GpsStateMachine
{
    public:
        GpsStateMachine() = default;
        explicit GpsStateMachine(const GpsStateMachineConfig& config);

        GpsTrustState update(int64_t timestamp, int8_t status);
        GpsTrustState checkTimeout(int64_t now);

        void reset();

        GpsTrustState state() const { return state_; }
        bool trusted() const { return state_ == GpsTrustState::kTrust; }

        int consecutiveCount() const { return consecutive_; }
        bool hasSample() const { return has_sample_; }

    private:
        GpsStateMachineConfig config_;
        GpsTrustState state_ = GpsTrustState::kDistrust;
        int consecutive_ = 0;
        int64_t last_timestamp_ = 0;
        bool has_sample_ = false;
};

}  // namespace Glider
