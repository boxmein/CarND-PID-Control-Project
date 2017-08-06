#include "PID.h"
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

constexpr int n_sett = 150;
constexpr int n_eval = 2400;

PID::PID() : p_error(0.),
             i_error(0.),
             d_error(0.),
             enable_twiddle(false),
             trigger_reset(false),
             step(1),
             phase(ADD_NEXT),
             agg_error(0),
             param_idx(0),
             sum_cte(0.),
             last_cte(0.) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  p[0] = Kp;
  p[1] = Ki;
  p[2] = Kd;
  
  dp[0] = 0.5 * Kp;
  dp[1] = 0.5 * Ki;
  dp[2] = 0.5 * Kd;
  
  best_error = 1e36;
}

void PID::UpdateError(double cte) {
  last_cte = this->cte;
  diff_cte = cte - last_cte;
  sum_cte += cte;
  this->cte = cte;
  
  if (enable_twiddle) {
    if (step > n_sett) {
      agg_error += cte * cte;
    }
    
    if (step > n_sett+n_eval) {
      
      if (agg_error < best_error) {
        cout << "\x1b[31mtwiddle: found improvement (" << agg_error << " < " << best_error << ")\x1b[0m" << endl;
        best_error = agg_error;
        dp[param_idx] *= 1.1;
        param_idx = (param_idx + 1) % 3;
      }
      
      if (phase == ADD_NEXT) {
        cout << "\x1b[31mtwiddle: adding to param " << param_idx << "\x1b[0m" << endl;
        p[param_idx] += dp[param_idx];
        phase = SUBTRACT_NEXT;
      } else if (phase == SUBTRACT_NEXT) {
        cout << "\x1b[31mtwiddle: subtracting from param " << param_idx << "\x1b[0m" << endl;
        p[param_idx] -= 2 * dp[param_idx];
        phase = REDUCE_DP;
      } else if (phase == REDUCE_DP) {
        cout << "\x1b[31mtwiddle: reducing dp of " << param_idx << "\x1b[0m" << endl;
        p[param_idx] += dp[param_idx]; // add the double subtract back to start off again
        dp[param_idx] *= 0.9;
        param_idx = (param_idx + 1) % 3;
        phase = ADD_NEXT;
      }
      
      trigger_reset = true;
      step = 1;
      
      agg_error = 0;
      sum_cte = 0;
      diff_cte = 0;
      last_cte = 0;
      
      cout << "\x1b[32mtwiddle: adjusted: Kp " << p[0] << ", Ki " << p[1] << ", Kd " << p[2] << "\x1b[0m" << endl;
      cout << "\x1b[32mtwiddle: deltas:   Kp " << dp[0] << ", dKi " << dp[1] << ", dKd " << dp[2] << "\x1b[0m" << endl;
    }
    
    step += 1;
  }
}

double PID::TotalError() {
  return last_cte + diff_cte + sum_cte;
}

double PID::get() {
  return -p[0] * cte - p[2] * diff_cte - p[1] * sum_cte;
}

/*


# Make this tolerance bigger if you are timing out!
def twiddle(tol=0.2):
    # Don't forget to call `make_robot` before you call `run`!
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)
    
    while sum(dp) > tol:
        for i in range(3):
            p[i] += dp[i]
            robot = make_robot()
            x, y, error = run(robot, p)
            
            if error < best_err:
                best_err = error
                dp[i] *= 1.1
            
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x, y, error = run(robot, p)
                if error < best_err:
                    best_err = error
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
    
    return p, best_err
*/
