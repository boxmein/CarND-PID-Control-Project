#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  /*
  Twiddle parameters
  */
  bool enable_twiddle;
  bool trigger_reset;
  /** Parameter array - { Kp, Ki, Kd } */
  double p[3];
  /** delta-param array - { dKp, dKi, dKd } */
  double dp[3];
  /** Which step we're on */
  int step;
  /** what phase the twiddler is on */
  enum { START, ADD_NEXT, SUBTRACT_NEXT, REDUCE_DP } phase;
  
  double best_error;
  double agg_error;
  int param_idx;
  
  /** PID parameters - cross track error, sum(CTE), last CTE, CTE derivative */
  double cte;
  double sum_cte;
  double last_cte;
  double diff_cte;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);
  
  double get();

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
