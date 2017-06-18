#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// DONE: Set the timestep length and duration
static size_t N = 25; // 25;
static double dt = 0.15;        /* for stability, 0.05, 0.1 are too small, 0.3 is too large, so far 0.15 works the best, 0.2 also works */

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
static const double Lf = 2.67;

// Both the reference cross track and orientation errors need to be 0.
// The reference velocity is set to 40 mph.
static double ref_v = 90;              // 40 would be more stable

struct MPC_OUTPUT {
  vector<double> predicted_ptsx;
  vector<double> predicted_ptsy;
  vector<double> next_vars;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state.
  // Return the next state and actuations as a
  // vector.
  // vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  MPC_OUTPUT Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
