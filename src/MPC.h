#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

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
