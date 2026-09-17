#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace robot_control {
namespace tasks {

/**
 * @brief Quintic polynomial trajectory generator for eccentric joint stepping.
 *
 * Generates smooth position, velocity, and acceleration profiles for
 * large-angle eccentric arm rotations (e.g., 180° or 360° step-over
 * maneuvers during stair climbing).
 *
 * Uses a quintic (5th-order) polynomial:
 *   q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
 *
 * Boundary conditions (zero velocity & acceleration at start/end):
 *   q(0)  = q_start,   q'(0)  = 0,   q''(0)  = 0
 *   q(T)  = q_end,     q'(T)  = 0,   q''(T)  = 0
 *
 * This ensures jerk-limited, smooth motion that is safe for the
 * mechanical structure and doesn't excite vibrations.
 */
class TrajectoryGenerator {
public:
    /**
     * @brief Plan a quintic trajectory from start to end angle.
     *
     * @param q_start    Start angle (rad)
     * @param q_end      End angle (rad)
     * @param duration   Total trajectory duration (seconds)
     */
    void plan(double q_start, double q_end, double duration);

    /**
     * @brief Evaluate the trajectory at a given time.
     *
     * @param t  Time since trajectory start (seconds)
     * @param[out] q     Position (rad)
     * @param[out] dq    Velocity (rad/s)
     * @param[out] ddq   Acceleration (rad/s^2)
     */
    void evaluate(double t, double& q, double& dq, double& ddq) const;

    /**
     * @brief Check if the trajectory is complete at time t.
     * @param t  Time since trajectory start (seconds)
     * @return true if t >= duration
     */
    bool isComplete(double t) const { return t >= duration_; }

    /**
     * @brief Get the planned duration.
     */
    double getDuration() const { return duration_; }

    /**
     * @brief Check if a trajectory has been planned.
     */
    bool isPlanned() const { return planned_; }

private:
    // Quintic polynomial coefficients: q(t) = a0 + a1*t + ... + a5*t^5
    double a0_ = 0.0, a1_ = 0.0, a2_ = 0.0;
    double a3_ = 0.0, a4_ = 0.0, a5_ = 0.0;
    double duration_ = 0.0;
    bool planned_ = false;
};

/**
 * @brief Multi-segment trajectory for complex stepping maneuvers.
 *
 * Chains multiple quintic segments for motions that require intermediate
 * waypoints (e.g., lift → extend → place → retract).
 *
 * Each segment is defined by a target angle and duration.
 */
struct TrajectoryWaypoint {
    double angle;     // target angle (rad)
    double duration;  // time to reach this waypoint (seconds)
};

class MultiSegmentTrajectory {
public:
    /**
     * @brief Plan a multi-segment trajectory through a sequence of waypoints.
     *
     * @param start_angle    Initial angle (rad)
     * @param waypoints      Sequence of intermediate + final waypoints
     */
    void plan(double start_angle, const std::vector<TrajectoryWaypoint>& waypoints);

    /**
     * @brief Evaluate the trajectory at a given time.
     *
     * Automatically selects the correct segment based on elapsed time.
     *
     * @param t  Time since trajectory start (seconds)
     * @param[out] q     Position (rad)
     * @param[out] dq    Velocity (rad/s)
     * @param[out] ddq   Acceleration (rad/s^2)
     */
    void evaluate(double t, double& q, double& dq, double& ddq) const;

    /**
     * @brief Check if the entire multi-segment trajectory is complete.
     */
    bool isComplete(double t) const { return t >= total_duration_; }

    /**
     * @brief Get total duration of all segments.
     */
    double getTotalDuration() const { return total_duration_; }

    bool isPlanned() const { return planned_; }

private:
    std::vector<TrajectoryGenerator> segments_;
    std::vector<double> segment_start_times_;
    double total_duration_ = 0.0;
    bool planned_ = false;
};

}  // namespace tasks
}  // namespace robot_control
