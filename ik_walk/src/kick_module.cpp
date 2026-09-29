#include "kick_module.hpp"

void kick_trajectory::put_point(double time, double x, double z,
                                double vx, double vz,
                                double ax, double az)
{
  traj_x.put_point(time, x, vx, ax);
  traj_z.put_point(time, z, vz, az);
}

void kick_trajectory::result(double t, double &out_x, double &out_z)
{
  out_x = traj_x.result(t);
  out_z = traj_z.result(t);
}

void kick_trajectory::clear()
{
  traj_x = foot_trajectory();
  traj_z = foot_trajectory();
}

double kick_trajectory::get_duration() const
{
  return duration_;
}

void kick_trajectory::generate_kick_trajectory(double z_start)
{
  clear();
  put_point(0.0,  150.0, z_start + 25.0);
  //put_point(0.1,   90.0, z_start + 25.0);
  // put_point(0.2,   70.0, z_start + 25.0);
  // put_point(0.3,   50.0, z_start + 25.0);
  put_point(0.4,   50.0, z_start + 25.0);
  put_point(0.5,   10.0, z_start +  5.0);
  put_point(0.6,  -10.0, z_start       );
  duration_ = 0.6;
}


