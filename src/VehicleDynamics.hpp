#ifndef VEHICLEDYNAMICS_H
#define VEHICLEDYNAMICS_H

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>

// const double m = 1450;              // Mass of the vehicle (kg)
// const double r = 0.3;               // Wheel radius (m)
// const double rho = 1.225;           // Air density (kg/m^3)
// const double Cd = 0.32;             // Drag coefficient
// const double A = 2.2;               // Frontal area (m^2)
// const double Cr = 0.015;            // Rolling resistance coefficient
// const double g = 9.81;              // Gravity (m/s^2)
// const double tau_max = 400;         // Max engine torque (Nm)
// const double max_torque_rate = 50;  // Maximum change in torque per time step (Nm/s)

class VehicleDynamics {
public:
    VehicleDynamics(double r, double rho, double Cd, double A, double Cr, double m, double g);

    double update(double torque, double v_actual, double dt);

private:
    double r, rho, Cd, A, Cr, m, g;
};

class VehicleModel {
public:
    VehicleModel(double mass, double wheelbase, double dt);

    void update(double force, double delta);
    void getState(double& x_out, double& y_out, double& theta_out, double& v_out) const;
    void resetState();
    void setState(double x, double y, double theta, double v);

private:
    double mass;
    double wheelbase;
    double dt;
    double x, y, theta, v, w;
};

#endif // VEHICLEDYNAMICS_H