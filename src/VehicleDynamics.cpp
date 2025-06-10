#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>
#include "VehicleDynamics.hpp"
using namespace std;

// Implementation of VehicleDynamics methods
VehicleDynamics::VehicleDynamics(double r, double rho, double Cd, double A, double Cr, double m, double g)
    : r(r), rho(rho), Cd(Cd), A(A), Cr(Cr), m(m), g(g) {}

double VehicleDynamics::update(double torque, double v_actual, double dt) {
    double F_drive = torque / r;
    double F_drag = 0.5 * rho * Cd * A * v_actual * v_actual;
    double F_rolling = Cr * m * g;
    // add mgsing(theta) as gravitational force
    double a = (F_drive - F_drag - F_rolling) / m;
    return v_actual + a * dt;
}

// Implementation of VehicleModel methods
VehicleModel::VehicleModel(double mass, double wheelbase, double dt)
    : mass(mass), wheelbase(wheelbase), dt(dt), x(0.0), y(0.0), theta(0.0), v(0.0), w(0.0) {}

void VehicleModel::update(double force, double delta) {
    double a = force / mass;
    v += a * dt;
    theta += (v / wheelbase) * std::tan(delta) * dt;
    x += v * std::cos(theta) * dt;
    y += v * std::sin(theta) * dt;
    w = (v / wheelbase) * std::tan(delta);
}

void VehicleModel::getState(double& x_out, double& y_out, double& theta_out, double& v_out) const {
    x_out = x;
    y_out = y;
    theta_out = theta;
    v_out = v;
}

void VehicleModel::resetState() {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    v = 0.0;
}

void VehicleModel::setState(double x, double y, double theta, double v) {
    this->x = x;
    this->y = y;
    this->theta = theta;
    this->v = v;
}
