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
   const double x {0.1716}, y {0.2544}, z {0.1531};

    // link lengths
    double l1 {0}, l2 {0.130}, l3 {0.135},l4 {0.126};
  
    //joint offsets
    double d1 {0.077}, d2 {0}, d3 {0}, d4 {0};

    //twist angles
    double a1 {90}, a2 {0}, a3 {0}, a4 {0};
  
    //joint angles
    double theta1 {}, theta2_1 {}, theta2_2 {}, theta3_1 {}, theta3_2 {}, theta4_1 {}, theta4_2 {};
  
    //end effector angle relative to base frame
    double phi{(3-13+79)}; // variable φ represents the angle between the y-axis of the base frame and the projection of the end effector position on the xy-plane. 
    double theta_prep {atan(y/x) * degree_conversion}; //  used to find theta 1 in degrees    

    if (x > 0 && y > 0){ // finding theta i in its appropriate quadrant
      theta1 = theta_prep;  
    }else if (x < 0 && y > 0){
        theta1 = theta_prep + 180;
    }else if(x < 0 && y <0){
        theta1 = theta_prep - 180;
    }else if(x > 0 && y == 1){
        theta1 = theta_prep;
    }else{
        return 1;
    }

    double pr {sqrt((x * x)+ (y * y))};
    
    //r-z surface
    double r3 {pr};
    double z3 {z - d1};

    double r2 {(r3 - (l4 * cos(phi * radians_conversion)))};
    double z2 {(z3 - (l4 * sin(phi* radians_conversion)))};

    double conversion_acos {((r2 * r2) + (z2 * z2) - ((l2 * l2) + (l3 * l3)))/(2 * l2 * l3)};

    double joint_offset {(atan(0.024/0.128) * degree_conversion)};

    theta3_1 = +((acos(conversion_acos) * degree_conversion) + joint_offset);
    theta3_2 = -((acos(conversion_acos) * degree_conversion) + joint_offset);
    
    double x_1 {}; //cos(theta_2_1)
    x_1 = ((l2 + l3 * cos((theta3_1 * radians_conversion))) * r2 + (l3 * sin((theta3_1 * radians_conversion))) * z2) / ((r2 * r2) + (z2 * r2));

    double x_2 {}; //cos(theta_2_2)
    x_2 = ((l2 + l3 * cos((theta3_2 * radians_conversion))) * r2 + (l3 * sin((theta3_2 * radians_conversion))) * z2) / ((r2 *  r2) + (z2 * r2));

    double y_1 {}; //sin(theta_2_1)
    y_1 = ((l2 + l3 * cos((theta3_1 * radians_conversion))) * z2 + (l3 * sin((theta3_1 * radians_conversion))) * r2) / ((r2 * r2) + (z2 * z2));

   double y_2 {}; //sin(theta _2_1)
   y_2 = ((l2 + l3 * cos((theta3_2 * radians_conversion))) * z2 + (l3 * sin((theta3_2 * radians_conversion))) * r2) / ((r2 * r2) + (z2 * z2));

    theta2_1 = (atan(sin(y_1)/ cos(x_1)) * degree_conversion) - joint_offset;
    theta2_2 = (atan(sin(y_2)/ cos(x_2)) * degree_conversion) - joint_offset;

    theta4_1  = phi - (theta2_1 + theta3_1);
    theta4_2  = phi - (theta2_2 + theta3_2);

    cout<<"Theta 1: "<<std::round(theta1)<<endl;
    cout<<"Theta 2_1: "<<std::round(theta2_1)<<endl;
    cout<<"Theta 2_2: "<<std::round(theta2_2)<<endl;
    cout<<"Theta 3_1: "<<std::round(theta3_1)<<endl;
    cout<<"Theta 3_2: "<<std::round(theta3_2)<<endl;
    cout<<"Theta 4_1: "<<std::round(theta4_1)<<endl;
    cout<<"Theta 4_2: "<<std::round(theta4_2)<<endl;

    return 0;
}
