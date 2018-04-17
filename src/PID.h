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
  * Coefficients
  */ 
  double tau[3];

  const int MIN_STEPS_TO_START_TWIDDLE = 100;

  int num_of_steps;

  double twiddle_error;
  double best_twiddle_error;

  double dp[3];

  int position_of_dp;

  enum class TwiddleType {
    INCREASE,
    DECREASE
  };

  TwiddleType lastTwiddleType;

  bool isTwiddleInitialized;
  
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

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Function to implement twiddle aglorithm
  */
  void twiddle();

  int getNumOfSteps();

};

#endif /* PID_H */
