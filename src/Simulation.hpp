#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "VehiclePIDController.hpp"
#include <vector>

class Simulation {
public:
    // Constructor
    Simulation(double dt, double T);

    // Methods
    std::vector<double> runSimulation(double v_target, double v_initial, double dt, double T);
    std::vector<double> runSimulationWithTwoVehicles(double v_target, double v_initial, double dt, double T, 
    double v2_initial, double distance, double sin_amplitude, double sin_freq);
    std::vector<double> sinTarget(double initial_value, double amplitude, double frequency, double dt, double T);
    // void loadDynamics(std::string filename);
    // void loadControllerParameters(std::string filename);

    std::vector<double> runSimulationWithChecks(double v_target, double v_initial, double dt, 
                                        double T, double v2_initial, double distance, double sin_amplitude, double sin_freq);
    void displayResults(const std::vector<double>& actual_speed, const std::vector<double>& torque_output,
                    const std::vector<double>& error_output, double v_target, double dt);

private:
    double dt;
    double T;
};

#endif // SIMULATION_HPP