#include "ShooterTrajopt.h"

#include <cmath>
#include <numbers>

#include <Eigen/Core>
#include <sleipnir/autodiff/Gradient.hpp>
#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "wpi/array.h"

namespace slp = sleipnir;

using Eigen::Vector3d;
using Vector6d = Eigen::Vector<double, 6>;

constexpr double field_width = 8.2296;    // 27 ft -> m
constexpr double field_length = 16.4592;  // 54 ft -> m
[[maybe_unused]]
constexpr double target_width = 1.05;       // m
constexpr double target_lower_edge = 1.98;  // m
constexpr double target_upper_edge = 2.11;  // m
constexpr double target_depth = 0.46;       // m
constexpr Vector6d target_wrt_field{{field_length - target_depth / 2.0},
                                    {field_width - 2.6575},
                                    {(target_upper_edge + target_lower_edge) / 2.0},
                                    {0.0},
                                    {0.0},
                                    {0.0}};
constexpr double g = 9.806;  // m/s²

slp::VariableMatrix f(const slp::VariableMatrix& x) {
  constexpr double rho = 1.204;  // kg/m³
  constexpr double C_D = 0.5;
  constexpr double A = std::numbers::pi * 0.3;
  constexpr double m = 2.0;  // kg
  auto a_D = [](auto v) { return 0.5 * rho * v * v * C_D * A / m; };

  auto v_x = x(3, 0);
  auto v_y = x(4, 0);
  auto v_z = x(5, 0);
  return slp::VariableMatrix{{v_x}, {v_y}, {v_z}, {-a_D(v_x)}, {-a_D(v_y)}, {-g - a_D(v_z)}};
}

/// @brief Calculates the optimal trajectory for the shooter to hit the target.
///
/// @param x_meter X-coordinate of the robot's position in meters.
/// @param y_meter Y-coordinate of the robot's position in meters.
/// @param vel_x Initial velocity in the X direction in m/s.
/// @param vel_y Initial velocity in the Y direction in m/s.
/// @param pitch The pitch angle of the arm in radians.
/// @param pivot_height The height of the pivot point (gear) above the ground in meters.
/// @return An array containing the angular velocity, yaw, and pitch angles.
wpi::array<double, 3> calculate_trajectory(const double x_meter, const double y_meter, const double vel_x,
                                           const double vel_y, const double pitch, const double pivot_height) {
  // Robot initial state
  Vector6d robot_wrt_field{{x_meter}, {y_meter}, {0.0}, {vel_x}, {vel_y}, {0.0}};

  constexpr double max_initial_velocity = 15.0;  // m/s

  // Length of the arm (constant)
  constexpr double arm_length = 1.0;  // Example length in meters

  slp::OptimizationProblem problem;

  // Set up decision variables
  auto arm_z_constraint = problem.DecisionVariable(1, 1);
  // We can't use SetValue on decision variables; instead, use subjectTo
  auto expected_arm_z = pivot_height + arm_length * std::sin(pitch);
  problem.SubjectTo(arm_z_constraint(0, 0) == expected_arm_z);

  // Shooter's position relative to the robot
  auto shooter_wrt_robot = problem.DecisionVariable(6, 1);
  problem.SubjectTo(shooter_wrt_robot(2, 0) == arm_z_constraint(0, 0));

  // Set initial guesses
  problem.SubjectTo(shooter_wrt_robot(0, 0) == 0.0);
  problem.SubjectTo(shooter_wrt_robot(1, 0) == 0.0);
  problem.SubjectTo(shooter_wrt_robot(3, 0) == 0.0);
  problem.SubjectTo(shooter_wrt_robot(4, 0) == 0.0);
  problem.SubjectTo(shooter_wrt_robot(5, 0) == 0.0);

  auto shooter_wrt_field = robot_wrt_field + shooter_wrt_robot.Value();

  // Set up duration decision variables
  constexpr int N = 10;
  auto T = problem.DecisionVariable();
  problem.SubjectTo(T >= 0);
  T = 1;  // Set initial guess value
  auto dt = T / N;

  // Disc state in the field frame
  auto x = problem.DecisionVariable(6, 1);

  // Position initial guess is start position
  x.Segment(0, 3) = shooter_wrt_field.segment(0, 3);

  // Velocity initial guess is max initial velocity toward target
  Vector3d uvec_shooter_to_target = (target_wrt_field.segment(0, 3) - shooter_wrt_field.segment(0, 3)).normalized();
  x.Segment(3, 3) = robot_wrt_field.segment(3, 3) + max_initial_velocity * uvec_shooter_to_target;

  // Constrain the z-offset of the shooter based on the arm pitch and pivot height
  problem.SubjectTo(x(2, 0) == arm_z_constraint(0, 0));

  // Shooter initial position
  problem.SubjectTo(x.Segment(0, 3) == shooter_wrt_field.segment(0, 3));

  // Require initial velocity is below max
  problem.SubjectTo(slp::pow(x(3, 0) - robot_wrt_field(3), 2) +
                        slp::pow(x(4, 0) - robot_wrt_field(4), 2) +
                        slp::pow(x(5, 0) - robot_wrt_field(5), 2) <=
                    max_initial_velocity * max_initial_velocity);

  // Dynamics constraints - RK4 integration
  auto h = dt;
  auto x_k = x;
  for (int k = 0; k < N - 1; ++k) {
    auto k1 = f(x_k);
    auto k2 = f(x_k + h / 2 * k1);
    auto k3 = f(x_k + h / 2 * k2);
    auto k4 = f(x_k + h * k3);
    x_k += h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
  }

  // Require final position is in center of target circle
  problem.SubjectTo(x_k.Segment(0, 3) == target_wrt_field.segment(0, 3));

  // Require the final velocity is up
  problem.SubjectTo(x_k(5, 0) > 0.0);

  // Minimize sensitivity of vertical position to velocity
  auto sensitivity = slp::Gradient(x_k(3, 0), x.Segment(3, 3)).Get();
  problem.Minimize(sensitivity.T() * sensitivity);

  problem.Solve({.diagnostics = true});

  // Initial velocity vector
  Eigen::Vector3d v0 = x.Segment(3, 3).Value() - robot_wrt_field.segment(3, 3);

  double velocity = v0.norm();
  double angular_velocity = velocity * 0.0254;

  double pitch_angle = std::atan2(v0(2), std::hypot(v0(0), v0(1)));

  double yaw = std::atan2(v0(1), v0(0));

  return {angular_velocity, yaw, pitch_angle};
}
