#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using std::cout;
using std::endl;
using namespace Eigen;

int main()
{
    //end effector position
    double x {-0.0833};
    double y {0.361};
    double z {0.1744};

    // link lengths
    double l1 {0};
    double l2 {0.130};
    double l3 {0.135};
    double l4 {0.126};
    
    //joint offsets
    double d1 {0.077};
    double d2 {0};
    double d3 {0};
    double d4 {0};

    //twist angles
    double a1 {90};
    double a2 {0};
    double a3 {0};
    double a4 {0};

    //joint angles
    double theta1 {};
    double theta2_1 {};
    double theta2_2 {};
    double theta3_1 {};
    double theta3_2 {};
    double theta4_1 {};
    double theta4_2 {};

    //end effector angle relative to base frame
    double phi{15-5+21};

    theta1 = atan2(y, x);

    double pr {sqrt(pow(x, 2)+ pow(y,2))};
    
    double r3 {pr};

    double z3 {z -d1};

    double r2 {r3 - l4 * cos(phi)};
    double z2 {z3 - l4 * sin(phi)};

    theta3_1 = +acos((pow(r2, 2) + pow(z2, 2) - (pow(l2, 2) + pow(l3, 2)))/(2 * l2 * a3));
    theta3_2 = -acos((pow(r2, 2) + pow(z2, 2) - (pow(l2, 2) + pow(l3, 2)))/(2 * l2 * a3));
    
    double x_1 {}; //cos(theta_2_1)
    x_1 = ((l2 + l3 * cos(theta3_1)) * r2 + (l3 * sin(theta3_1)) * z2) / (pow(r2, 2) + pow(z2, 2));

    double x_2 {}; //cos(theta_2_2)
    x_2 = ((l2 + l3 * cos(theta3_2)) * r2 + (l3 * sin(theta3_2)) * z2) / (pow(r2, 2) + pow(z2, 2));

    double y_1 {}; //sin(theta_2_1)
    y_1 = ((l2 + l3 * cos(theta3_1)) * z2 + (l3 * sin(theta3_1)) * r2) / (pow(r2, 2) + pow(z2, 2));

    double y_2 {}; //sin(theta _2_1)
    y_2 = ((l2 + l3 * cos(theta3_2)) * z2 + (l3 * sin(theta3_2)) * r2) / (pow(r2, 2) + pow(z2, 2));

    theta2_1 = atan(sin(theta2_1)/ cos(theta2_1));
    theta2_2 = atan(sin(theta2_2)/ cos(theta2_2));

    theta4_1  = phi - (theta2_1 + theta3_1);
    theta4_2  = phi - (theta2_2 + theta3_2);

    const double pi{3.142};
    cout<<"Theta 1: "<<theta1 * (180/pi)<<endl;
    cout<<"Theta 2_1: "<<theta2_1 * (180/pi)<<endl;
    cout<<"Theta 2_2: "<<theta2_2 * (180/pi)<<endl;
    cout<<"Theta 3_1: "<<theta3_1 * (180/pi)<<endl;
    cout<<"Theta 3_2: "<<theta3_2 * (180/pi)<<endl;
    cout<<"Theta 4_1: "<<theta4_1 * (180/pi)<<endl;
    cout<<"Theta 4_2: "<<theta4_2 * (180/pi)<<endl;

    return 0;
}
