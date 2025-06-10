#include "VehiclePIDController.hpp"
#include "Simulation.hpp"
#include "Utilities.hpp"
#include <iostream>

// load the simulation parameters using load parameters function from Utilities.hpp
void loadSimulationParameters(double &v_target, double &v_initial, double &dt, double &T, double &v2_initial, double &distance
    , double &sin_amplitude, double &sin_freq) {
    std::string path = "sim_parameters.csv";
    std::map<std::string, double> parameters = loadParameters(path);

    if (parameters.find("v_target") != parameters.end()) {
        v_target = parameters["v_target"];
    }
    if (parameters.find("v_initial") != parameters.end()) {
        v_initial = parameters["v_initial"];
    }
    if (parameters.find("sin_amplitude") != parameters.end()) {
        sin_amplitude = parameters["sin_amplitude"];
    }
    if (parameters.find("sin_freq") != parameters.end()) {
        sin_freq = parameters["sin_freq"];
    }
    if (parameters.find("dt") != parameters.end()) {
        dt = parameters["dt"];
    }
    if (parameters.find("T") != parameters.end()) {
        T = parameters["T"];
    }
    if (parameters.find("v2_initial") != parameters.end()) {
        v2_initial = parameters["v2_initial"];
    }
    if (parameters.find("distance") != parameters.end()) {
        distance = parameters["distance"];
    }
}

int main(int argc, char *argv[]) {
    // Simulation parameters
    double v_target = 35 ;  // Desired speed in km/h
    double v_initial = 15 ; // Initial speed in km/h
    double dt = 0.1;                // Time step s
    double T = 500;                  // Total simulation time s
    double v2_initial = 10 ; // Initial speed of the second vehicle in km/h
    double distance = 100;            // Initial distance between vehicles in m
    double sin_amplitude = 0;        // Amplitude of the sine wave in km/h
    double sin_freq = 0;            // Frequency of the sine wave in Hz
    int choice = 2;                // Choice of simulation
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <choice>\n";
        return 1;
    }
    choice = std::stoi(argv[1]);

    // Call the function to load parameters
    loadSimulationParameters(v_target, v_initial, dt, T, v2_initial, distance, sin_amplitude, sin_freq);

    // convert the parameters to the correct units (m/s)
    v_target *= 0.27778;
    v_initial *= 0.27778;
    v2_initial *= 0.27778;


    Simulation sim(dt, T);
    std::vector<double> actual_speed;
    if (choice == 1) {
        std::cout << "Running simulation with one vehicle...\n";
        actual_speed = sim.runSimulation(v_target, v_initial, dt, T);
    } else if (choice == 2) {
        std::cout << "Running simulation with two vehicles...\n";
        actual_speed = sim.runSimulationWithTwoVehicles(v_target, v_initial, dt, T, v2_initial, distance, sin_amplitude, sin_freq);
    } else if (choice == 3){
        std::cout << "Running simulation with checks...\n";
        actual_speed = sim.runSimulationWithChecks(v_target, v_initial, dt, T, v2_initial, distance, sin_amplitude, sin_freq);
    } else {
        std::cerr << "Invalid choice. Exiting...\n";
        return 1;
    }
    
    // Display performance metrics
    std::cout << "Simulation completed!\n";
    return 0;
}
