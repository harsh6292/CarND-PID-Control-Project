#include "PID.h"
#include <iostream>
#include <math.h>

/*
* TODO: Complete the PID class.
*/

PID::PID() {

    // Initialize the errors
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    // Initilize variables for twiddle

    // Variable that keep tracks of number of steps twiddle/update error is called
    num_of_steps = 0;

    // variable that capture the cte error for twiddle after certain no of steps are passed
    twiddle_error = 0.0;

    // keeps track of the best twiddle error found
    best_twiddle_error = 0.0;

    // variable that keeps track which coefficient was changed in last twiddle
    position_of_dp = 0;


    dp[0] = 0.1;
    dp[1] = 0.00007;
    dp[2] = 0.3;

    // Variable that keeps track of last type of twiddle performed (increased values of error or decreased)
    lastTwiddleType = TwiddleType::INCREASE;

    // variable that keeps track if twiddle was initialized earlier
    isTwiddleInitialized = false;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    // Initialize tau co-efficients
    tau[0] = Kp;
    tau[1] = Ki;
    tau[2] = Kd;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;

    p_error = cte;

    i_error += cte;

    num_of_steps += 1;

    // Once algorithms has taken some time to converge, start tracking twiddle error for further optimization
    if (num_of_steps >= MIN_STEPS_TO_START_TWIDDLE) {
        twiddle_error += pow(cte, 2);
    }
}

double PID::TotalError() {
    return (-(tau[0] * p_error) -(tau[1] * i_error) -(tau[2] * d_error));
}


int PID::getNumOfSteps() {
    return num_of_steps;
}


void PID::twiddle() {

    // First time Twiddle
    if (!isTwiddleInitialized) {
        std::cout << "======================================================" << std::endl;
        std::cout << "Initializing Twiddle!" << std::endl;
        best_twiddle_error = twiddle_error;

        std::cout << "Best Error: " << best_twiddle_error << std::endl;
        tau[position_of_dp] += dp[position_of_dp];

        isTwiddleInitialized = true;
        return;
    }

    // Calculate the current error starting from the time when twiddle error state was initialized
    double current_error = (twiddle_error/(num_of_steps - MIN_STEPS_TO_START_TWIDDLE));
    std::cout << "Current twiddle error: " << current_error << std::endl;


    // Reset errors back to zero at each twiddle call
    twiddle_error = 0.0;
    num_of_steps = 0;
    i_error = 0.0;
    d_error = 0.0;
    p_error = 0.0;


    // If the current error is less than the best error, new best error found
    if (current_error < best_twiddle_error) {
        best_twiddle_error = current_error;

        // Update and increase the current tau value for which twiddle was running
        dp[position_of_dp] *= 1.1;
        position_of_dp = ((position_of_dp + 1) % 3);

        tau[position_of_dp] += dp[position_of_dp];
        lastTwiddleType = TwiddleType::INCREASE;

        std::cout << "***********************************************" << std::endl;
        std::cout << "Found the best current error" << std::endl;
        std::cout << "Kp: " << tau[0] << ", Ki: " << tau[1] << ", Kd: " << tau[2] << std::endl;

    } else if (lastTwiddleType == TwiddleType::INCREASE) {
        // Not the best error and last time we increased the value, now decrease such that value is less than last time value
        tau[position_of_dp] -= (2 * dp[position_of_dp]);
        lastTwiddleType = TwiddleType::DECREASE;

    } else {
        // Last time we decreased value and it didn't reduce the error, so now increase the value

        // This will reset the value to initial value before we decreased
        tau[position_of_dp] += dp[position_of_dp];

        // Decrease the factor by which to steer
        dp[position_of_dp] *= 0.9;
        position_of_dp = ((position_of_dp + 1) % 3);

        tau[position_of_dp] += dp[position_of_dp];
        lastTwiddleType = TwiddleType::INCREASE;
    }
}

