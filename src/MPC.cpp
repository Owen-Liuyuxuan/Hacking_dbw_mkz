#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 20;
double dt = 0.05;
const double PI = 3.14159;
int x_start = 0, y_start = x_start + N, theta_start = y_start + N, omega_start = theta_start + N;

CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x)
{
  CppAD::AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++)
  {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

CppAD::AD<double> derivative_eval(Eigen::VectorXd coeffs, CppAD::AD<double> x)
{
  CppAD::AD<double> result = 0.0;
  for (int i = 1; i < coeffs.size(); i++)
  {
    result += coeffs[i] * CppAD::pow(x, i - 1) * i;
  }
  return result;
}
CppAD::AD<double> transform_angle(CppAD::AD<double> x)
{
  CppAD::AD<double> result = x;

  while (result > PI)
  {
    result -= 2 * PI;
  }
  while (result <= -PI)
  {
    result += 2 * PI;
  }

  return result;
}

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.

class FG_eval
{
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector &fg, const ADvector &vars)
  {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + theta_start] = vars[theta_start];
    //fg[1 + cte_start] = vars[cte_start];
    //fg[1 + epsi_start] = vars[epsi_start];

    for (int i = 0; i < N - 1; i++)
    {
      fg[x_start + i + 2] = vars[x_start + i] + V * dt * CppAD::cos(vars[theta_start + i]) - vars[x_start + i + 1];
      fg[y_start + i + 2] = vars[y_start + i] + V * dt * CppAD::sin(vars[theta_start + i]) - vars[y_start + i + 1];
      fg[theta_start + i + 2] = vars[theta_start + i] + vars[omega_start + i] * dt - vars[theta_start + i + 1];
    }
    fg[0] = 0;
    for (int i = 0; i < N; i++)
    {
      fg[0] += 5 * CppAD::pow((polyeval(coeffs, vars[x_start + i]) - vars[y_start + i]), 2);
      fg[0] += 0.3 * CppAD::pow( (vars[theta_start + i] - derivative_eval(coeffs, vars[x_start + i])), 2);
    }
    for (int i = 0; i < N-1; i++)
    {
      fg[0] += 100 * CppAD::pow(vars[omega_start + i], 2);
    }
    for (int i = 0; i < N - 2; i++)
    {
      fg[0] += 1 * CppAD::pow(vars[omega_start + i] - vars[omega_start + i + 1], 2);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 15 + 2 * 14
  size_t n_vars = 3 * N + 1 * (N - 1); //3 * 15 + 1 * 14 = 45 + 14 = 59
  // TODO: Set the number of constraints
  size_t n_constraints = 3 * (N); //56

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
  {
    vars[i] = 0;
  }
  

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  for (int i = 0; i < omega_start; i++)
  {
    vars_lowerbound[i] = -1e9;
    vars_upperbound[i] = 1e9;
  }

  for (int i = omega_start; i < n_vars; i++)
  {
    vars_lowerbound[i] = -0.7;
    vars_upperbound[i] = +0.7;
  }


  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++)
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = state[0];
  constraints_lowerbound[y_start] = state[1];
  constraints_lowerbound[theta_start] = state[2];

  constraints_upperbound[x_start] = state[0];
  constraints_upperbound[y_start] = state[1];
  constraints_upperbound[theta_start] = state[2];

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;
  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;


  result.push_back(solution.x[omega_start]);
  return result;
}
