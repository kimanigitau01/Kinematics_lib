#include <iostream>
#include <Eigen/Dense>

using std::cout;
using std::endl;
using namespace Eigen;

int main()
{
    MatrixXd T (4,4), T1 (4,4), T2 (4,4), T3(4,4), T4(4,4); //the overall transformation matrix and t1...t4 matrices

    T.setIdentity();
    T1.setIdentity();
    T2.setIdentity();
    T3.setIdentity();
    T4.setIdentity();

    cout<<T<<endl;

    return 0;
}