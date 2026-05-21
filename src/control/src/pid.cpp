#include "control/pid.hpp"
#include <algorithm>

namespace control
{

// -----------------------------------------------------------------------
// Constructor
// -----------------------------------------------------------------------
PID::PID(double kp, double ki, double kd, double min_output, double max_output)
: kp_(kp),
  ki_(ki),
  kd_(kd),
  min_output_(min_output),
  max_output_(max_output),
  integral_(0.0),
  prev_error_(0.0),
  first_call_(true)
{
}

// -----------------------------------------------------------------------
// Compute
// -----------------------------------------------------------------------
double PID::compute(double setpoint, double measurement, double dt)
{
  if (dt <= 0.0) {
    return 0.0;
  }

  // Error
  double error = setpoint - measurement;

  // Integral with anti-windup clamp
  integral_ += error * dt;
  integral_ = std::clamp(integral_, min_output_ / (ki_ + 1e-9),
                                     max_output_ / (ki_ + 1e-9));

  // Derivative (skip on first call to avoid spike)
  double derivative = 0.0;
  if (!first_call_) {
    derivative = (error - prev_error_) / dt;
  }
  first_call_ = false;

  // Store error for next iteration
  prev_error_ = error;

  // PID output
  double output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);

  // Clamp output
  return std::clamp(output, min_output_, max_output_);
}

// -----------------------------------------------------------------------
// Set Gains
// -----------------------------------------------------------------------
void PID::setGains(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

// -----------------------------------------------------------------------
// Reset
// -----------------------------------------------------------------------
void PID::reset()
{
  integral_   = 0.0;
  prev_error_ = 0.0;
  first_call_ = true;
}

}  // namespace control