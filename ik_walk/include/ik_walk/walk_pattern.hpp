#ifndef WALK_PATTERN_HPP
#define WALK_PATTERN_HPP

#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>

#define Linear 1
#define Fifth 5

using namespace std;

class foot_trajectory
{
public:
  struct interpolation
  {
    vector<double> coefs;
    double time_min;
    double time_max;
  };

  vector<interpolation> pattern;

  void put_point(double time, double pos, double vel, double acc);
  void compute_polynom();
  double result(double t);

private:
  struct data_point
  {
    double time;
    double pos;
    double vel;
    double acc;
  };

  vector<data_point> point;
  vector<double> Fifth_poly_interpolation(data_point f, data_point f1);
};

#endif