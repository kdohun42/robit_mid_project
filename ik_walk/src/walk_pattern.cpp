#include "walk_pattern.hpp"
using namespace std;

void foot_trajectory::put_point(double time, double pos, double vel, double acc)
{
  point.push_back({ time, pos, vel, acc });
  compute_polynom();
}

void foot_trajectory::compute_polynom()
{
  pattern.clear();

  if (point.size() >= 2)
  {
    for (int i = 0; i < point.size() - 1; i++)
    {
      pattern.push_back({ Fifth_poly_interpolation(point[i], point[i + 1]), point[i].time, point[i + 1].time });
    }
  }
}

std::vector<double> foot_trajectory::Fifth_poly_interpolation(data_point f, data_point f1)
{
  double p0 = f.pos;
  double v0 = f.vel;
  double a0 = f.acc;
  double t0 = f.time;

  double p1 = f1.pos;
  double v1 = f1.vel;
  double a1 = f1.acc;
  double t1 = f1.time;

  double T = t1 - t0;

  std::vector<double> get_coefs;
  get_coefs.push_back(p0);
  get_coefs.push_back(v0);
  get_coefs.push_back(a0 / 2);
  get_coefs.push_back((20 * (p1 - p0) - (8 * v1 + 12 * v0) * T - (3 * a1 - a0) * T * T) / (2 * T * T * T));
  get_coefs.push_back((30 * (p0 - p1) + (14 * v1 + 16 * v0) * T + (3 * a1 - 2 * a0) * T * T) / (2 * T * T * T * T));
  get_coefs.push_back((12 * (p1 - p0) - 6 * T * (v1 + v0) - (a1 - a0) * T * T) / (2 * T * T * T * T * T));

  return get_coefs;
}

double foot_trajectory::result(double t)
{
  double index = 0.0;
  double P5_x = 0.0;
  double t0, t1;

  for (int i = 0; i < pattern.size(); i++)
  {
    if (t >= pattern[i].time_min && t <= pattern[i].time_max)
      index = i;
    else if (t == pattern[i].time_max || t == pattern[i].time_min)
      index = i;
  }

  t0 = pattern[index].time_min;
  t1 = pattern[index].time_max;

  for (int i = 0; i < pattern[index].coefs.size(); i++)
  {
    P5_x += pattern[index].coefs[i] * pow(t - t0, i);
  }
  return P5_x;
}