#include <iostream>
#include <cmath>

using namespace std;

const double pi = acos(-1.0);

double deg2rad(double deg) {
    return deg * pi / 180.0;
}

double rad2deg(double rad) {
    return rad * 180.0 / pi;
}

int main() {
    // Define robot parameters
    double d1 = 0.77, d2 = 0, d3 = 0, d4 = 0; // Joint offset
    double a1 = 90.0, a2 = 0.0, a3 = 0.0, a4 = 0.0; // Twist angle
    double l1 = 0, l2 = 0.130, l3 = 0.135, l4 = 0.126; // Link length
    double x = -0.0833, y = 0.361, z = 0.1744; // End effector position

    // Calculate inverse kinematics
    double r = sqrt(x * x + y * y);
    double s = z - d1;
    double theta1 = atan2(y, x);
    double D = (r * r + s * s - l2 * l2 - l3 * l3) / (2 * l2 * l3);
    double theta3 = acos(D);
    double theta2 = atan2(s, r) - atan2(l3 * sin(theta3), l2 + l3 * cos(theta3));
    double theta4 = a4 - a2 - theta2 - theta3;

    // Print results in degrees
    cout << "Joint angles:\n";
    cout << "theta1 = " << rad2deg(theta1) << " degrees\n";
    cout << "theta2 = " << rad2deg(theta2) << " degrees\n";
    cout << "theta3 = " << rad2deg(theta3) << " degrees\n";
    cout << "theta4 = " << rad2deg(theta4) << " degrees\n";

    return 0;
}
