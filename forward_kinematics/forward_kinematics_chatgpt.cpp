#include <Eigen/Dense>
#include <iostream>

int main()
{
    // Define the homogeneous transformation matrices for each joint
    Eigen::Matrix4d T1, T2, T3, T4;
    
    // Define the robot's joint angles
    double q1 = 0.1;
    double q2 = 0.2;
    double q3 = 0.3;
    double q4 = 0.4;
    
    // Define the robot's link lengths and offsets
    double l1 = 0.5;
    double l2 = 0.4;
    double l3 = 0.3;
    double l4 = 0.2;
    
    // Calculate the homogeneous transformation matrix for each joint
    T1 << cos(q1), -sin(q1), 0, 0,
          sin(q1), cos(q1), 0, 0,
          0, 0, 1, l1,
          0, 0, 0, 1;
    
    T2 << cos(q2), -sin(q2), 0, l2*cos(q2),
          sin(q2), cos(q2), 0, l2*sin(q2),
          0, 0, 1, 0,
          0, 0, 0, 1;
          
    T3 << cos(q3), -sin(q3), 0, l3*cos(q3),
          sin(q3), cos(q3), 0, l3*sin(q3),
          0, 0, 1, 0,
          0, 0, 0, 1;
          
    T4 << cos(q4), -sin(q4), 0, l4*cos(q4),
          sin(q4), cos(q4), 0, l4*sin(q4),
          0, 0, 1, 0,
          0, 0, 0, 1;
    
    // Calculate the overall transformation matrix from the base to the end effector
    Eigen::Matrix4d T = T1 * T2 * T3 * T4;
    
    // Extract the position and orientation of the end effector from the transformation matrix
    Eigen::Vector3d position = T.block<3,1>(0,3);
    Eigen::Matrix3d orientation = T.block<3,3>(0,0);
    
    // Print the results
    std::cout << "End effector position: " << position.transpose() << std::endl;
    std::cout << "End effector orientation:\n" << orientation << std::endl;
    
    return 0;
}
