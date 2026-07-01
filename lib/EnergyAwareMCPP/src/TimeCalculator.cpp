#include "EnergyAwareMCPP/TimeCalculator.hpp"
#include "EnergyAwareMCPP/utils.hpp"
#include "EnergyAwareMCPP/EnergyCalculator.h" // For access to static helper methods like angle_between_points
#include <cmath>
#include <algorithm>
#include <stdexcept>

TimeCalculator::TimeCalculator(const time_calculator_config_t& config) : m_config(config) {}

// This implementation of the interface method defaults to horizontal movement time.
double TimeCalculator::calculate_segment_cost(double v_in, double a_in, double v_out, double a_out, double s) const {
    return calculate_horizontal_segment_time(v_in, a_in, v_out, a_out, s);
}

// Calculates time for a horizontal segment using a trapezoidal velocity profile.
double TimeCalculator::calculate_horizontal_segment_time(double v_in, double a_in, double v_out, double a_out, double s) const {
    if (s <= 1e-5) return 0.0;
    
    double v_max = m_config.max_horizontal_speed;
    double t_acc = std::abs(v_max - v_in) / a_in;
    double s_acc = v_in * t_acc + 0.5 * a_in * std::pow(t_acc, 2);

    double t_dec = std::abs((v_max - v_out) / a_out);
    double s_dec = v_max * t_dec + 0.5 * a_out * std::pow(t_dec, 2);

    if (s_acc + s_dec <= s) {
        return t_acc + t_dec + (s - s_acc - s_dec) / v_max;
    } else {
        return calculate_short_segment_time(v_in, a_in, v_out, a_out, s);
    }
}

// Calculates time for a vertical segment using a trapezoidal velocity profile.
double TimeCalculator::calculate_vertical_segment_time(double v_in, double a_in, double v_out, double a_out, double s) const {
    if (s <= 1e-5) return 0.0;
    
    double v_max = m_config.max_vertical_speed;
    double t_acc = std::abs(v_max - v_in) / a_in;
    double s_acc = v_in * t_acc + 0.5 * a_in * std::pow(t_acc, 2);

    double t_dec = std::abs((v_max - v_out) / a_out);
    double s_dec = v_max * t_dec + 0.5 * a_out * std::pow(t_dec, 2);

    if (s_acc + s_dec <= s) {
        return t_acc + t_dec + (s - s_acc - s_dec) / v_max;
    } else {
        // Použijeme stejnou logiku pro krátký segment, ale se zřetelem na vertikální zrychlení
        double a_abs_out = std::abs(a_out);
        double v_sol = std::sqrt((v_in * v_in / a_in + v_out * v_out / a_abs_out + 2 * s) / (1.0 / a_in + 1.0 / a_abs_out));
        if (std::isnan(v_sol) || v_sol < v_in || v_sol < v_out || v_sol > v_max) {
            // Fallback for cases where the physics don't allow reaching the speed.
            double avg_speed = (v_in + v_out) / 2.0;
            if (avg_speed < 1e-3) return 0.0;
            return s / avg_speed;
        }
        return (v_sol - v_in) / a_in + (v_sol - v_out) / a_abs_out;
    }
}

// Handles cases where the segment is too short to reach max speed.
double TimeCalculator::calculate_short_segment_time(double v_in, double a_in, double v_out, double a_out, double s) const {
    double a_abs_out = std::abs(a_out);
    double v_sol = std::sqrt((v_in * v_in / a_in + v_out * v_out / a_abs_out + 2 * s) / (1.0 / a_in + 1.0 / a_abs_out));
    
    // If no real solution is found, fall back to a simple average speed calculation.
    if (std::isnan(v_sol) || v_sol < v_in || v_sol < v_out || v_sol > m_config.max_horizontal_speed) {
        double avg_speed = (v_in + v_out) / 2.0;
        if (avg_speed < 1e-3) return 0.0;
        return s / avg_speed;
    }
    return (v_sol - v_in) / a_in + (v_sol - v_out) / a_abs_out;
}

// Calculates turning properties based on physics from the original EnergyCalculator.
TimeCalculator::turning_properties_time_t TimeCalculator::calculate_turning_properties(double angle) const {
    angle = std::abs(angle);
    double v_r = m_config.max_horizontal_speed;
    double a_h = m_config.horizontal_acceleration;

    // For a 180-degree turn, assume the drone can maintain speed.
    if (std::abs(angle * 180.0 / M_PI - 180.0) < 1.0) {
        return {v_r, -a_h, v_r, a_h, 0.0, 0.0};
    }

    double phi = M_PI - angle;
    double phi_2 = phi / 2.0;
    double d_x = m_config.allowed_path_deviation;
    double a_x = a_h * std::cos(phi_2);
    double a_y = a_h * std::sin(phi_2);

    double dv_x = std::sqrt(2.0 * d_x * a_x);
    dv_x = std::min(dv_x, std::cos(M_PI_2 - phi) * v_r / 2.0);

    double dv_y = std::tan(phi_2) * dv_x;
    double vy_m = dv_x / std::tan(phi_2);

    double v_in = vy_m + dv_y;
    double t_turn = 2.0 * dv_y / a_y; // Time spent executing the turn maneuver.

    return {v_in, -a_y, v_in, a_y, t_turn, vy_m};
}

// Main cost calculation for a full 3D path.
double TimeCalculator::calculate_path_cost(const std::vector<point_heading_t<double>>& path) const {
    if (path.size() < 2) return 0.0;

    double total_time = 0.0;

    // Filter out adjacent duplicate points to avoid division by zero.
    std::vector<point_heading_t<double>> filtered_path;
    filtered_path.push_back(path[0]);
    for (size_t i = 1; i < path.size(); ++i) {
        double d = std::sqrt(std::pow(path[i].x - path[i-1].x, 2) + std::pow(path[i].y - path[i-1].y, 2) + std::pow(path[i].z - path[i-1].z, 2));
        if (d > 1e-5) {
            filtered_path.push_back(path[i]);
        }
    }

    if (filtered_path.size() < 2) return 0.0;

    // STEP A: Pre-calculate the properties of all turns in the 2D (X,Y) plane.
    std::vector<turning_properties_time_t> turns;
    turns.push_back({0.0, 0.0, 0.0, m_config.horizontal_acceleration, 0.0, 0.0}); // Start from rest
    for (size_t i = 1; i + 1 < filtered_path.size(); ++i) {
        double angle = EnergyCalculator::angle_between_points(
            {filtered_path[i-1].x, filtered_path[i-1].y},
            {filtered_path[i].x, filtered_path[i].y},
            {filtered_path[i+1].x, filtered_path[i+1].y}
        );
        turns.push_back(calculate_turning_properties(angle));
    }
    turns.push_back({0.0, -m_config.horizontal_acceleration, 0.0, 0.0, 0.0, 0.0}); // Stop at the end

    // STEP B: Calculate the time for each segment.
    for (size_t i = 0; i + 1 < filtered_path.size(); ++i) {
        const auto& p1 = filtered_path[i];
        const auto& p2 = filtered_path[i+1];

        double horiz_dist = std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
        double vert_dist = std::abs(p2.z - p1.z);

        double segment_horiz_time = 0.0;
        double segment_vert_time = 0.0;

        // 1. Calculate horizontal flight time, considering deceleration into and acceleration out of turns.
        if (horiz_dist > 1e-5) {
            const auto& turn1 = turns[i];
            const auto& turn2 = turns[i+1];

            double t_acc_slow = std::abs((turn1.v_after - turn1.d_vym) / turn1.a_after);
            double s_acc_slow = turn1.d_vym * t_acc_slow + 0.5 * turn1.a_after * std::pow(t_acc_slow, 2);

            double t_dec_slow = std::abs((turn2.v_before - turn2.d_vym) / turn2.a_before);
            double s_dec_slow = turn2.v_before * t_dec_slow + 0.5 * turn2.a_before * std::pow(t_dec_slow, 2);

            if (s_acc_slow + s_dec_slow < horiz_dist) {
                double slow_acc_time = t_acc_slow + t_dec_slow;
                double straight_time = calculate_horizontal_segment_time(
                    turn1.v_after, m_config.horizontal_acceleration,
                    turn2.v_before, -m_config.horizontal_acceleration,
                    horiz_dist - s_acc_slow - s_dec_slow
                );
                segment_horiz_time = slow_acc_time + straight_time;
            } else {
                segment_horiz_time = calculate_short_segment_time(
                    turn1.d_vym, turn1.a_after,
                    turn2.d_vym, turn2.a_before,
                    horiz_dist
                );
            }
            // Add the time spent maneuvering in the turn itself.
            segment_horiz_time += turn1.time;
        }

        // 2. Calculate vertical flight time (ascent/descent).
        if (vert_dist > 1e-5) {
            // Assume vertical movement starts and ends with zero vertical velocity for each segment.
            segment_vert_time = calculate_vertical_segment_time(
                0.0, m_config.vertical_acceleration,
                0.0, -m_config.vertical_acceleration,
                vert_dist
            );
        }

        // 3. Coordinated 3D movement:
        // Multirotors often perform horizontal and vertical movements simultaneously (decoupled axes).
        // The total time for a 3D segment is determined by the slower axis (the one that takes longer).
        total_time += std::max(segment_horiz_time, segment_vert_time);
    }

    return total_time;
}