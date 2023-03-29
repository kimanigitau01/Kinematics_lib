/* this is for a robot with a DoF of 3 */
#include <iostream>
#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;
using std::cout;
using std::endl;

int main()
{
    // initialize the end effector position and orientation
    double x  {3};
    double y  {4};
    double phi {150};

    // initialize the constant link lengths
    const int L1 {2};
    const int L2 {5};

    //compute and compare the general transformation matrix and desired matrix
    // to get the joint angles

    // the variables are the the cos and sine (sine has two values due to the sqrt())
    double c2 {};
    double s2_1 {};
    double s2_2 {};
    
    c2 = (pow(x, 2) + pow(y, 2) - pow(L1, 2) - pow(L2, 2)) / (2 * L1 *L2);  

    if (c2 > -1 && c2 < 1){
        s2_1 = +sqrt(1 - pow(c2, 2));
        s2_2 = -sqrt(1 - pow(c2, 2));

    }else{
        return 1;
    }

    // the variables for the joint angles
    double theta1_1 {};
    double theta1_2 {};
    double theta2_1 {};
    double theta2_2 {};
    double theta3_1 {};
    double theta3_2 {};

    //the two values for theta2
    theta2_1 = atan2(s2_1, c2);
    theta2_2 = atan2(s2_2, c2);

    //the two values for theta1
    theta1_1 = (atan2(y, x) - atan2(L2 * s2_1, L1 + L2 * c2));
    theta1_2 = (atan2(y, x) - atan2(L2 * s2_2, L1 + L2 * c2));

    //the two values for theta 3
    theta3_1 = (atan2(sin(phi), cos(phi)) - theta1_1 - theta2_1);
    theta3_2 = (atan2(sin(phi), cos(phi)) - theta1_2 - theta2_2);

    cout<<"the joint values 1 are: "<<theta1_1<<", "<<theta2_1<<", "<<theta3_1<<" radians."<<endl;
    cout<<"the joint values 2 are: "<<theta1_2<<", "<<theta2_2<<", "<<theta3_2<<" radians."<<endl;
} 