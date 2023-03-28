#include <iostream>
#include <Eigen/Dense>
#include <math.h>


using std::cout;
using std::endl;
using namespace Eigen;

int main()
{
    MatrixXd T (4,4), T1 (4,4), T2 (4,4), T3(4,4), T4(4,4); //the overall transformation matrix and t1...t4 matrices

    // link length for t1...t4
    double linkLengthT1 {0};
    double linkLengthT2 {0};
    double linkLengthT3 {0};
    double linkLengthT4 {0};

    // twist angle for t1...t4
    double twistAngleT1 {0};
    double twistAngleT2 {90};
    double twistAngleT3 {0};
    double twistAngleT4 {0};

    //joint offset for t1...t4
    double jointOffsetT1 {10};
    double jointOffsetT2 {20};
    double jointOffsetT3 {30};
    double jointOffsetT4 {40};

    //joint angle for t1...t4
    double jointAngleT1 {10};
    double jointAngleT2 {0};
    double jointAngleT3 {30};
    double jointAngleT4 {0};

    T1<<cos(jointAngleT1), -sin(jointAngleT1), 0, twistAngleT1,
    sin(jointAngleT1) * cos(twistAngleT1), cos(jointAngleT1) * cos(twistAngleT1), -sin(twistAngleT1), -sin(twistAngleT1) * jointOffsetT1,
    sin(jointAngleT1) * sin(twistAngleT1), cos(jointAngleT1) * sin(twistAngleT1), cos(twistAngleT1), cos(twistAngleT1) * jointOffsetT1,
    0,0,0,1;


    T2<<cos(jointAngleT2), -sin(jointAngleT2), 0, twistAngleT2,
    sin(jointAngleT2) * cos(twistAngleT2), cos(jointAngleT2) * cos(twistAngleT2), -sin(twistAngleT2), -sin(twistAngleT2) * jointOffsetT2,
    sin(jointAngleT2) * sin(twistAngleT2), cos(jointAngleT2) * sin(twistAngleT2), cos(twistAngleT2), cos(twistAngleT2) * jointOffsetT2,
    0,0,0,1;

    T3<<cos(jointAngleT3), -sin(jointAngleT3), 0, twistAngleT3,
    sin(jointAngleT3) * cos(twistAngleT3), cos(jointAngleT3) * cos(twistAngleT3), -sin(twistAngleT3), -sin(twistAngleT3) * jointOffsetT3,
    sin(jointAngleT3) * sin(twistAngleT3), cos(jointAngleT3) * sin(twistAngleT3), cos(twistAngleT3), cos(twistAngleT3) * jointOffsetT3,
    0,0,0,1;

    T4<<cos(jointAngleT4), -sin(jointAngleT4), 0, twistAngleT4,
    sin(jointAngleT4) * cos(twistAngleT4), cos(jointAngleT4) * cos(twistAngleT4), -sin(twistAngleT4), -sin(twistAngleT4) * jointOffsetT4,
    sin(jointAngleT4) * sin(twistAngleT4), cos(jointAngleT4) * sin(twistAngleT4), cos(twistAngleT4), cos(twistAngleT4) * jointOffsetT4,
    0,0,0,1;

    T = T1 * T2 * T3 * T4;

    cout<<"End effector orientation: \n"<<T.block(0,0,3,3)<<"\n";
    cout<<"End Effector Position: \n"<<T.col(3)<<"\n";
    cout<<"End Effector Position: \n"<<T.block(0,3,3,1)<<"\n";
    
    return 0;
}