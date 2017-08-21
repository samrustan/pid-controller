#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

  p_error = i_error = d_error = 0.0;
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;


}

void PID::UpdateError(double cte) {
  p_error = cte;
  d_error = cte - p_error;
  i_error = cte + i_error;
}

double PID::TotalError() {
}

