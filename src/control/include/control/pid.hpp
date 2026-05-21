#ifndef CONTROL__PID_HPP_
#define CONTROL__PID_HPP_

namespace control
{

class PID
{
public:
  // -----------------------------------------------------------------------
  // Constructor
  // -----------------------------------------------------------------------
  PID(double kp, double ki, double kd, double min_output, double max_output);

  // -----------------------------------------------------------------------
  // Compute PID output
  // dt : time since last call in seconds
  // -----------------------------------------------------------------------
  double compute(double setpoint, double measurement, double dt);

  // -----------------------------------------------------------------------
  // Update gains at runtime
  // -----------------------------------------------------------------------
  void setGains(double kp, double ki, double kd);

  // -----------------------------------------------------------------------
  // Reset integrator (call when switching setpoints)
  // -----------------------------------------------------------------------
  void reset();

private:
  // Gains
  double kp_;
  double ki_;
  double kd_;

  // Output limits
  double min_output_;
  double max_output_;

  // Internal state
  double integral_;
  double prev_error_;
  bool   first_call_;
};

}  // namespace control

#endif  // CONTROL__PID_HPP_