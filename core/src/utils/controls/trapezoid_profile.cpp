#include "../core/include/utils/controls/trapezoid_profile.h"
#include "../core/include/utils/math_util.h"
#include <cmath>

TrapezoidProfile::TrapezoidProfile(double max_v, double accel, double decel)
    : start(0), end(0), max_v(max_v), accel(accel), decel(decel) {}

void TrapezoidProfile::set_max_v(double max_v) { this->max_v = max_v; }

void TrapezoidProfile::set_accel(double accel) { this->accel = accel; }

void TrapezoidProfile::set_endpts(double start, double end) {
  this->start = start;
  this->end = end;
}

// Kinematic equations as macros
#define CALC_POS(time_s, a, v, s) ((0.5 * (a) * (time_s) * (time_s)) + ((v) * (time_s)) + (s))
#define CALC_VEL(time_s, a, v) (((a) * (time_s)) + (v))

/**
 * @brief Run the trapezoidal profile based on the time that's ellapsed
 *
 * @param time_s Time since start of movement
 * @return motion_t Position, velocity and acceleration
 */
motion_t TrapezoidProfile::calculate(double time_s) {
  double delta_pos = end - start;

  // redefine accel and max_v in this scope for negative calcs
  double accel_local = this->accel;
  double decel_local = this->decel;
  double max_v_local = this->max_v;
  if (delta_pos < 0) {
    accel_local = -this->accel;
    decel_local = -this->decel;
    max_v_local = -this->max_v;
  }

  // Calculate the time spent during the acceleration / maximum velocity / deceleration stages
  double accel_time = max_v_local / accel_local;
  double decel_time = max_v_local / decel_local;
  double accelerating_dists =
    (.5 * accel_local * accel_time * accel_time) + (.5 * decel_local * decel_time * decel_time);
  double max_vel_time = (delta_pos - accelerating_dists) / max_v_local;
  this->time = accel_time + max_vel_time + decel_time;

  // If the time during the "max velocity" state is negative, use an S profile
  if (max_vel_time < 0) {

    double accel_dp = (decel / (accel + decel)) * delta_pos;
    double decel_dp = delta_pos - accel_dp;

    accel_time = sqrt(fabs(accel_dp / accel));
    decel_time = sqrt(fabs(decel_dp / decel));
    max_vel_time = 0;
    this->time = accel_time + decel_time;
  }

  motion_t out;

  // Handle if a bad time is put in
  if (time_s < 0) {
    out.pos = start;
    out.vel = 0;
    out.accel = 0;
    return out;
  }

  // Handle after the setpoint is reached
  if (time_s > accel_time + decel_time + max_vel_time) {
    out.pos = end;
    out.vel = 0;
    out.accel = 0;
    return out;
  }

  // ======== KINEMATIC EQUATIONS ========

  // Displacement from initial acceleration
  if (time_s < accel_time) {
    out.pos = CALC_POS(time_s, accel_local, 0, start);
    out.vel = CALC_VEL(time_s, accel_local, 0);
    out.accel = accel_local;
    return out;
  }

  double s_accel = CALC_POS(accel_time, accel_local, 0, 0);

  // Displacement during maximum velocity
  if (time_s < accel_time + max_vel_time) {
    out.pos = CALC_POS(time_s - accel_time, 0, max_v_local, (s_accel + start));
    out.vel = sign(delta_pos) * max_v;
    out.accel = 0;
    return out;
  }

  double s_max_vel = CALC_POS(max_vel_time, 0, max_v_local, s_accel);
  // Displacement during deceleration
  out.pos = CALC_POS(time_s - accel_time - max_vel_time, -decel_local, max_v_local, start + s_max_vel);
  out.vel = CALC_VEL(time_s - accel_time - max_vel_time, -decel_local, max_v_local);
  out.accel = -decel_local;
  return out;
}

double TrapezoidProfile::get_movement_time() { return time; }
