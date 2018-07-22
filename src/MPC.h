#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
  // Set the delay in seconds 
  void SetDelay(double delay_seconds);

 private:
  // Delay in seconds
  double delay_sec;
  // Previous steering angle actuation
  double pre_delta;
  // Previous throttle actuation
  double pre_a;
};

#endif /* MPC_H */
