#ifndef KICK_MODULE_HPP
#define KICK_MODULE_HPP

#include "walk_pattern.hpp"

class kick_trajectory
{
public:
  // (x, z) 경유점 추가. vel/acc 기본값 0 (시작/끝점 정지)
  void put_point(double time, double x, double z,
                 double vx = 0.0, double vz = 0.0,
                 double ax = 0.0, double az = 0.0);

  // 시각 t에서의 EE 위치 (x, z) 반환
  void result(double t, double &out_x, double &out_z);

  // 경유점 전체 초기화
  void clear();

  // z_start = Default_Z_Right 를 외부에서 받아 경유점 생성
  void generate_kick_trajectory(double z_start);

  double get_duration() const;

private:
  foot_trajectory traj_x;
  foot_trajectory traj_z;
  double duration_ = 0.0;
};

#endif
