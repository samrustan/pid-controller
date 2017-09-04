#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  prev_cte = 0.0;
}

void PID::UpdateError(double cte) {

  p_error = cte;
  d_error = cte - prev_cte;
  i_error = cte + i_error;

  prev_cte = cte;
}

double PID::TotalError() {
  return 0;
}

/*
double twiddle(double tol=0.2) {

    double p[] = {0, 0, 0};
    double dp[] = {1, 1, 1};

    int iter = 0;
    while (sum(dp) > tol) {
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p
*/
