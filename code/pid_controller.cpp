/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   this->Kp = Kp;
   this->Ki = Ki;
   this->Kd = Kd;
   this->output_lim_max = output_lim_max;
   this->output_lim_min = output_lim_min;
   cte0 = 0;
   I = 0;
}

void PID::UpdateError(double cte) {
  /**
  * TODO: Update PID errors based on cte.
  **/
  if(abs(delta_time) < 0.000001) return;
  double P = Kp * cte;
  I += Ki * cte * delta_time;
  double D = Kd * (cte - cte0) / delta_time;
  action = P + I + D;
  cte0 = cte;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = action;
    if (control < output_lim_min) control = output_lim_min;
    if (control > output_lim_max) control = output_lim_max;
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   delta_time = new_delta_time;
   return delta_time;
}
