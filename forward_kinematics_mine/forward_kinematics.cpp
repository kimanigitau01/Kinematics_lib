#include <iostream>
#include <Eigen/Dense>
#include <math.h>


using std::cout;
using std::endl;
using namespace Eigen;

int main()
{

    const double  degree_conversion {(180/M_PI)};
    const double radians_conversion {(M_PI/180)};

    MatrixXd T (4,4), T1 (4,4), T2 (4,4), T3(4,4), T4(4,4);

    // link length for t1...t4
    double linkLengthT1 {0}, linkLengthT2 {0.130}, linkLengthT3 {0.135}, linkLengthT4 {0.126};

    // twist angle for t1...t4
    double twistAngleT1 {90 * radians_conversion}, twistAngleT2 {0 * radians_conversion}, twistAngleT3 {0 * radians_conversion}, twistAngleT4 {0 * radians_conversion};
  
    //joint offset for t1...t4
    double jointOffsetT1 {0.077}, jointOffsetT2 {0}, jointOffsetT3 {0}, jointOffsetT4 {0};
    

    //joint angle for t1...t4
    double jointAngleT1 {56 * radians_conversion}, jointAngleT2 {3 * radians_conversion}, jointAngleT3 {-13 * radians_conversion}, jointAngleT4 {79 * radians_conversion};
 
    T1<<cos(jointAngleT1), (-sin(jointAngleT1) * cos(twistAngleT1)), (sin(jointAngleT1) * sin(twistAngleT1)) , (linkLengthT1 * cos(jointAngleT1)),
        sin(jointAngleT1), cos(jointAngleT1) * cos(twistAngleT1), -cos(jointAngleT1) * sin(twistAngleT1), linkLengthT1 * sin(jointAngleT1),
        0, sin(twistAngleT1), cos(twistAngleT1), jointOffsetT1,
        0,0,0,1;

    cout<<T1<<endl;

    T2<<cos(jointAngleT2), -sin(jointAngleT2) * cos(twistAngleT2), sin(jointAngleT2) * sin(twistAngleT2) , linkLengthT2 * cos(jointAngleT2),
        sin(jointAngleT2), cos(jointAngleT2) * cos(twistAngleT2), -cos(jointAngleT2) * sin(twistAngleT2), linkLengthT2 * sin(jointAngleT2),
        0, sin(twistAngleT2), cos(twistAngleT2), jointOffsetT2,
        0,0,0,1;

    cout<<T2<<endl;

    T3<<cos(jointAngleT3), -sin(jointAngleT3) * cos(twistAngleT3), sin(jointAngleT3) * sin(twistAngleT3) , linkLengthT3 * cos(jointAngleT3),
        sin(jointAngleT3), cos(jointAngleT3) * cos(twistAngleT3), -cos(jointAngleT3) * sin(twistAngleT3), linkLengthT3 * sin(jointAngleT3),
        0, sin(twistAngleT3), cos(twistAngleT3), jointOffsetT3,
        0,0,0,1;        

    cout<<T3<<endl;

    T4<<cos(jointAngleT4), -sin(jointAngleT4) * cos(twistAngleT4), sin(jointAngleT4) * sin(twistAngleT4) , linkLengthT4 * cos(jointAngleT4),
        sin(jointAngleT4), cos(jointAngleT4) * cos(twistAngleT4), -cos(jointAngleT4) * sin(twistAngleT4), linkLengthT4 * sin(jointAngleT4),
        0, sin(twistAngleT4), cos(twistAngleT4), jointOffsetT4,
        0,0,0,1;

    cout<<T4<<endl;

    T = T1 * T2 * T3 * T4;

    cout<<"End effector orientation: \n"<<T.block(0,0,3,3)<<"\n";
    cout<<"\nEnd Effector Position: \n"<<T.block(0,3,3,1)<<"\n";
    
    return 0;
}