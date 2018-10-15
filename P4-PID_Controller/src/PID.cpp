#include "PID.h"
#include <iostream>
#include <math.h>
#include <valarray>
#include <vector>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;
  cout << "Initialized" << endl;
}

void PID::UpdateError(double cte)
{
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::TotalError() { return Kp * p_error + Ki * i_error + Kd * d_error; }

void PID::twiddle(double cte)
{
  vector<double> p, dp;
  p[0], p[1], p[2] = 0;
  dp[0], dp[1], dp[2] = 0;
  double err = 0;
  err += pow(cte, 2);
  double best_err = 1;
  int j = 0;
  double dp_sum = 0;
  for (auto &num : dp)
  {
    dp_sum += num;
  }

  while (dp_sum > 1E-3)
  {
    for (int i = 0; i < 3; ++i)
    {
      p[i] += dp[i];
      err += pow(cte, 2);
      if (err < best_err)
      {
        best_err = err;
        dp[i] *= 1.1;
      }
      else
      {
        p[i] -= 2 * dp[i];
        err += pow(cte, 2);
        if (err < best_err)
        {
          best_err = err;
          dp[i] *= 1.1;
        }
        else
        {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
    for (auto &num : dp)
    {
      dp_sum += num;
    }
    j += 1;
  }

}
