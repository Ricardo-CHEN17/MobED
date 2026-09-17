#include "robot_control/tasks/trajectory_generator.hpp"
#include <algorithm>

namespace robot_control {
namespace tasks {

// ============================================================================
// Single-segment quintic polynomial trajectory
// ============================================================================

void TrajectoryGenerator::plan(double q_start, double q_end, double duration) {
    // ================================================================
    // Quintic polynomial with boundary conditions:
    //   q(0) = q_start,  q'(0) = 0,  q''(0) = 0
    //   q(T) = q_end,    q'(T) = 0,  q''(T) = 0
    //
    // Solution (normalized time s = t/T):
    //   q(s) = q_start + (q_end - q_start) * (10*s^3 - 15*s^4 + 6*s^5)
    //
    // In polynomial form with actual time t:
    //   Let h = q_end - q_start, T = duration
    //   a0 = q_start
    //   a1 = 0
    //   a2 = 0
    //   a3 = 10*h / T^3
    //   a4 = -15*h / T^4
    //   a5 = 6*h / T^5
    // ================================================================

    duration_ = std::max(duration, 0.01);  // prevent division by zero
    double h = q_end - q_start;
    double T = duration_;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    a0_ = q_start;
    a1_ = 0.0;
    a2_ = 0.0;
    a3_ = 10.0 * h / T3;
    a4_ = -15.0 * h / T4;
    a5_ = 6.0 * h / T5;

    planned_ = true;
}

void TrajectoryGenerator::evaluate(double t, double& q, double& dq, double& ddq) const {
    if (!planned_) {
        q = 0.0; dq = 0.0; ddq = 0.0;
        return;
    }

    // Clamp time to [0, duration]
    double tc = std::clamp(t, 0.0, duration_);

    double t2 = tc * tc;
    double t3 = t2 * tc;
    double t4 = t3 * tc;
    double t5 = t4 * tc;

    // Position
    q = a0_ + a1_ * tc + a2_ * t2 + a3_ * t3 + a4_ * t4 + a5_ * t5;

    // Velocity (first derivative)
    dq = a1_ + 2.0 * a2_ * tc + 3.0 * a3_ * t2 + 4.0 * a4_ * t3 + 5.0 * a5_ * t4;

    // Acceleration (second derivative)
    ddq = 2.0 * a2_ + 6.0 * a3_ * tc + 12.0 * a4_ * t2 + 20.0 * a5_ * t3;

    // If past the end, hold final position with zero vel/accel
    if (t >= duration_) {
        dq = 0.0;
        ddq = 0.0;
    }
}

// ============================================================================
// Multi-segment trajectory
// ============================================================================

void MultiSegmentTrajectory::plan(
    double start_angle,
    const std::vector<TrajectoryWaypoint>& waypoints)
{
    segments_.clear();
    segment_start_times_.clear();
    total_duration_ = 0.0;

    if (waypoints.empty()) {
        planned_ = false;
        return;
    }

    double current_angle = start_angle;

    for (const auto& wp : waypoints) {
        TrajectoryGenerator seg;
        seg.plan(current_angle, wp.angle, wp.duration);

        segment_start_times_.push_back(total_duration_);
        segments_.push_back(seg);

        total_duration_ += wp.duration;
        current_angle = wp.angle;
    }

    planned_ = true;
}

void MultiSegmentTrajectory::evaluate(double t, double& q, double& dq, double& ddq) const {
    if (!planned_ || segments_.empty()) {
        q = 0.0; dq = 0.0; ddq = 0.0;
        return;
    }

    // Find the active segment
    int active_seg = static_cast<int>(segments_.size()) - 1;  // default to last
    for (int i = 0; i < static_cast<int>(segments_.size()); ++i) {
        double seg_end = segment_start_times_[i] + segments_[i].getDuration();
        if (t < seg_end) {
            active_seg = i;
            break;
        }
    }

    // Local time within the active segment
    double local_t = t - segment_start_times_[active_seg];

    segments_[active_seg].evaluate(local_t, q, dq, ddq);
}

}  // namespace tasks
}  // namespace robot_control
