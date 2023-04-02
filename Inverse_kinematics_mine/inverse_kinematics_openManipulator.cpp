#include <iostream>
#include <cmath>
#include <Eigen/Dense>

using std::cout;
using std::endl;
using namespace Eigen;

int main()
{
    const double  degree_conversion {(180/3.1415)};
    const double radians_conversion {(3.1415/180)};

    //end effector position
   const double x {-0.0833};
   const double y {0.361};
   const double z {0.1744};

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
    double phi{(15-5+21) * radians_conversion};
    double theta_prep {atan(y/x) * degree_conversion}; // find theta 1
    

    if (x > 0 && y > 0){
      theta1 = theta_prep;  
    }else if (x < 0 && y > 0){
        theta1 = theta_prep + 180;
    }else if(x < 0 && y <0){
        theta1 = theta_prep - 180;
    }else if(x > 0 && y == 1){
        theta1 = theta_prep;
    }

    double pr {sqrt(pow(x, 2)+ pow(y,2))};
    

    double r3 {pr};

    double z3 {z -d1};

    double r2 {(r3 - (l4 * cos(phi)))};
    double z2 {(z3 - (l4 * sin(phi)))};
    // cout<<r2<<endl;
    // cout<<z2<<endl;
    // cout<<cos(phi)<<endl;

    double conversion_acos {(((r2 * r2) + (z2 * z2)) - ((l2 * l2) + (l3 * l3)))/(2 * l2 * l3)};

    theta3_1 = +acos(conversion_acos) * degree_conversion;
    theta3_2 = -acos(conversion_acos) * degree_conversion;
    
    double x_1 {}; //cos(theta_2_1)
    x_1 = ((l2 + l3 * cos((theta3_1 * radians_conversion))) * r2 + (l3 * sin((theta3_1 * radians_conversion))) * z2) / ((r2, r2) + (z2, r2));
    cout<<x_1<<endl;

    double x_2 {}; //cos(theta_2_2)
    x_2 = ((l2 + l3 * cos((theta3_2 * radians_conversion))) * r2 + (l3 * sin((theta3_2 * radians_conversion))) * z2) / ((r2, r2) + (z2, r2));
    cout<<x_2<<endl;

    double y_1 {}; //sin(theta_2_1)
    y_1 = ((l2 + l3 * cos((theta3_1 * radians_conversion))) * z2 + (l3 * sin((theta3_1 * radians_conversion))) * r2) / ((r2 * r2) + (z2 * z2));

    double y_2 {}; //sin(theta _2_1)
    y_2 = ((l2 + l3 * cos((theta3_2 * radians_conversion))) * z2 + (l3 * sin((theta3_2 * radians_conversion))) * r2) / ((r2 * r2) + (z2 * z2));

    theta2_1 = (atan(sin(y_1)/ cos(x_1)) * degree_conversion);
    theta2_2 = (atan(sin(y_2)/ cos(x_2)) * degree_conversion);

    theta4_1  = phi - (theta2_1 + theta3_1);
    theta4_2  = phi - (theta2_2 + theta3_2);

    cout<<"Theta 1: "<<theta1<<endl;
    cout<<"Theta 2_1: "<<theta2_1<<endl;
    cout<<"Theta 2_2: "<<theta2_2<<endl;
    cout<<"Theta 3_1: "<<theta3_1<<endl;
    cout<<"Theta 3_2: "<<theta3_2<<endl;
    cout<<"Theta 4_1: "<<theta4_1<<endl;
    cout<<"Theta 4_2: "<<theta4_2<<endl;

    return 0;
}
