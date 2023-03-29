#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

int main() {
  // Robot arm design specifications
  const double d1 = 1.0;
  const double a1 = 0.5;
  const double alpha1 = M_PI_2;
  const double d2 = 0.5;
  const double a2 = 1.0;
  const double alpha2 = 0.0;
  const double d3 = 0.5;
  const double a3 = 0.5;
  const double alpha3 = M_PI_2;
  const double d4 = 0.25;
  const double a4 = 0.0;
  const double alpha4 = 0.0;

  // Joint angles
  const double q1 = M_PI_2;
  const double q2 = 0.0;
  const double q3 = M_PI_2;
  const double q4 = M_PI_4;

  // Compute homogeneous transformation matrices
  Matrix4d T_01, T_12, T_23, T_34;
  T_01 << cos(q1), -sin(q1)*cos(alpha1),  sin(q1)*sin(alpha1), a1*cos(q1),
          sin(q1),  cos(q1)*cos(alpha1), -cos(q1)*sin(alpha1), a1*sin(q1),
                0,            sin(alpha1),            cos(alpha1),         d1,
                0,                      0,                      0,          1;
  T_12 << cos(q2), -sin(q2)*cos(alpha2),  sin(q2)*sin(alpha2), a2*cos(q2),
          sin(q2),  cos(q2)*cos(alpha2), -cos(q2)*sin(alpha2), a2*sin(q2),
                0,            sin(alpha2),            cos(alpha2),         d2,
                0,                      0,                      0,          1;
  T_23 << cos(q3), -sin(q3)*cos(alpha3),  sin(q3)*sin(alpha3), a3*cos(q3),
          sin(q3),  cos(q3)*cos(alpha3), -cos(q3)*sin(alpha3), a3*sin(q3),
                0,            sin(alpha3),            cos(alpha3),         d3,
                0,                      0,                      0,          1;
  T_34 << cos(q4), -sin(q4)*cos(alpha4),  sin(q4)*sin(alpha4), a4*cos(q4),
          sin(q4),  cos(q4)*cos(alpha4), -cos(q4)*sin(alpha4), a4*sin(q4),
                0,            sin(alpha4),            cos(alpha4),         d4,
                0,                      0,                      0,          1;

  // Compute end effector position and orientation
  Matrix4d T_0e = T_01 * T_12 * T_23 * T_34;
  Vector3d position = T_0e.block<3,1>(0,3);
  Matrix3d orientation = T_0e.block<3,3>(0,0);

  // Print results
  std::cout << "End effector position:\n" << position << "\n";
  std::cout << "End effector orientation:\n" << orientation <<"\n";
}