#include "VehiclePIDController.hpp"
#include "VehicleDynamics.hpp"
#include "Simulation.hpp"
#include "Utilities.hpp"
#include <iostream>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <fstream>

// Vehicle parameters
double m = 1450;              // Mass of the vehicle (kg)
double r = 0.3;               // Wheel radius (m)
double rho = 1.225;           // Air density (kg/m^3)
double Cd = 0.32;             // Drag coefficient
double A = 2.2;               // Frontal area (m^2)
double Cr = 0.015;            // Rolling resistance coefficient
double g = 9.81;              // Gravity (m/s^2)
double tau_max = 400;         // Max engine torque (Nm)
double max_torque_rate = 50;  // Maximum change in torque per time step (Nm/s)

// PID Gains
double Kp = 100;
double Ki = 0.82;
double Kd = 0;
double d_Kp = 0.3;
double d_Ki = 0.01;
double d_Kd = 0.0;

// ACC Gains
double time_gap = 1.5;
double time_gap_tolerance = 0.1;
double pid_speed_control = 0;
double minimum_distance = 10;
// Sin Target
// double sin_amplitude = 0;
// double sin_freq = 0;


Simulation::Simulation(double dt, double T):dt(dt), T(T) {
        // Load the controller parameters
        std::string path = "gains.csv";
        std::map<std::string, double> parameters = loadParameters(path);
        
        if (parameters.find("Kp") != parameters.end()) {
            Kp = parameters["Kp"];
        }
        if (parameters.find("Ki") != parameters.end()) {
            Ki = parameters["Ki"];
        }
        if (parameters.find("Kd") != parameters.end()) {
            Kd = parameters["Kd"];
        }
        if (parameters.find("d_Kp") != parameters.end()) {
            d_Kp = parameters["d_Kp"];
        }
        if (parameters.find("d_Ki") != parameters.end()) {
            d_Ki = parameters["d_Ki"];
        }
        if (parameters.find("d_Kd") != parameters.end()) {
            d_Kd = parameters["d_Kd"];
        }
        if (parameters.find("time_gap") != parameters.end()) {
            time_gap = parameters["time_gap"];
        }
        if (parameters.find("time_gap_tolerance") != parameters.end()) {
            time_gap_tolerance = parameters["time_gap_tolerance"];
        }
        if (parameters.find("pid_speed_control") != parameters.end()) {
            pid_speed_control = parameters["pid_speed_control"];
        }
        if (parameters.find("minimum_distance") != parameters.end()) {
            minimum_distance = parameters["minimum_distance"];
        }
        
        // load the vehicle parameters
        path = "dynamics_parameters.csv";
        parameters = loadParameters(path);
        if (parameters.find("m") != parameters.end()) {
            m = parameters["m"];
        }
        if (parameters.find("r") != parameters.end()) {
            r = parameters["r"];
        }
        if (parameters.find("rho") != parameters.end()) {
            rho = parameters["rho"];
        }
        if (parameters.find("Cd") != parameters.end()) {
            Cd = parameters["Cd"];
        }
        if (parameters.find("A") != parameters.end()) {
            A = parameters["A"];
        }
        if (parameters.find("Cr") != parameters.end()) {
            Cr = parameters["Cr"];
        }
        if (parameters.find("g") != parameters.end()) {
            g = parameters["g"];
        }
        if (parameters.find("tau_max") != parameters.end()) {
            tau_max = parameters["tau_max"];
        }
        if (parameters.find("max_torque_rate") != parameters.end()) {
            max_torque_rate = parameters["max_torque_rate"];
        }
    }

std::vector<double> Simulation::runSimulation(double v_target, double v_initial, double dt, double T) {
    int steps = static_cast<int>(T / dt) + 1;
    std::vector<double> time(steps);
    std::vector<double> actual_speed(steps, 0.0);
    std::vector<double> torque_output(steps, 0.0);
    std::vector<double> error_output(steps, 0.0);

    ACCController acc(time_gap, time_gap_tolerance, pid_speed_control,Kp, Ki, Kd, d_Kp, d_Ki, d_Kd);
    VehicleDynamics vehicleDynamics(r, rho, Cd, A, Cr, m, g);
    
    double v_actual = v_initial;
    double prev_torque = 0;

    // Open a CSV file to log the data
    std::ofstream logFile("simulation_log.csv");
    logFile << "Time,Error,Actual Speed,Torque,Target Speed\n";

    for (int i = 0; i < steps; ++i) {
        // Time calculation
        time[i] = i * dt;
        // PID logic
        double torque = acc.calculate_torque(v_target, v_actual, tau_max, dt);
        // torque = std::max(0.0, std::min(tau_max, torque));
        double torque_rate = (torque - prev_torque) / dt;
        if (std::abs(torque_rate) > max_torque_rate) {
            torque = prev_torque + std::copysign(max_torque_rate * dt, torque_rate);
        }
        prev_torque = torque;
        v_actual = vehicleDynamics.update(torque, v_actual, dt);
        
        // Store results
        actual_speed[i] = v_actual * 3.6;
        torque_output[i] = torque;
        error_output[i] = v_target*3.6 - v_actual*3.6;

        // Log the data to the CSV file
        logFile << time[i] << "," << error_output[i] << "," << actual_speed[i] << "," << torque_output[i] << "," << v_target*3.6 << "\n";
    }

    // Close the CSV file
    logFile.close();

    displayResults(actual_speed, torque_output, error_output, v_target, dt);
    // Return results (for example, speed for now)
    return actual_speed;
}

std::vector<double> Simulation::runSimulationWithTwoVehicles(double v_target, double v_initial, double dt, 
double T, double v2_initial, double distance, double sin_amplitude, double sin_freq) {
    int steps = static_cast<int>(T / dt) + 1;
    std::vector<double> time(steps);
    std::vector<double> actual_speed(steps, 0.0);
    std::vector<double> torque_output(steps, 0.0);
    std::vector<double> error_output(steps, 0.0);
    std::vector<double> target_vehicle_speed(steps, v2_initial);
    std::vector<double> distance_between(steps, distance);
    std::vector<double> true_time_gap(steps, 0.0);
    std::vector<double> target_time_gap(steps, time_gap);

    VehicleDynamics vehicleDynamics(r, rho, Cd, A, Cr, m, g);
    ACCController acc(time_gap, time_gap_tolerance, pid_speed_control, Kp, Ki, Kd, d_Kp, d_Ki, d_Kd);

    // Generate the target vehicle speed profile
    if (sin_amplitude > 0 && sin_freq > 0) {
        target_vehicle_speed = sinTarget(v2_initial, sin_amplitude, sin_freq, dt, T);
    }
    
    double v_actual = v_initial;
    double prev_torque = 0;
    double driver_set_speed = v_target;

    // Open a CSV file to log the data
    std::ofstream logFile("simulation_log_ACC.csv");
    logFile << "Time,Error,EgoSpeed,Torque,ACCTargetSpeed,LeadSpeed,DriverSetSpeed,Gap(m),TrueTimeGap,TargetTimeGap,SafeDistance\n";

    for (int i = 0; i < steps; ++i) {
        // Time calculation
        time[i] = i * dt;
        // ACC logic
        double v_lead = target_vehicle_speed[i];
        double distance_to_lead = distance;
        if (i > 0)
            distance_to_lead = distance_between[i - 1];

        bool lead_vehicle_present = distance_to_lead >= 0;
        // ACC logic
        double v_target = acc.calculateTargetSpeed(distance_to_lead, minimum_distance,v_actual, v_lead, driver_set_speed, lead_vehicle_present,dt);
        double torque = acc.calculate_torque(v_target, v_actual, tau_max, dt);
        
        // Dynamic constraints
        torque = std::max(-tau_max, std::min(tau_max, torque));
        double torque_rate = (torque - prev_torque) / dt;
        if (std::abs(torque_rate) > max_torque_rate) {
            torque = prev_torque + std::copysign(max_torque_rate * dt, torque_rate);
        }
        prev_torque = torque;
        v_actual = vehicleDynamics.update(torque, v_actual, dt);

        // Update distance between vehicles
        if (i > 0)    
            distance_between[i] = distance_between[i - 1] + (v_lead - v_actual) * dt;

        // Calculate true time gap
        if (v_actual > 0) {
            // if v_actual is near 0 then set time gap to -1
            if (v_actual < 1){
                true_time_gap[i] = -1;}
            else{
                true_time_gap[i] = (distance_between[i]-10) / (v_actual);}
        } else {
            true_time_gap[i] = 0.0;
        }

        // Store results
        actual_speed[i] = v_actual * 3.6;
        torque_output[i] = torque;
        error_output[i] = v_target * 3.6 - v_actual * 3.6;

        double safe_distance = acc.safe_distance;
        // Log the data to the CSV file
        logFile << time[i] << "," << error_output[i] << "," << actual_speed[i] << "," << torque_output[i] << "," 
        << v_target * 3.6 << "," << target_vehicle_speed[i] * 3.6 << "," << driver_set_speed * 3.6 << "," 
        << distance_between[i] << "," << true_time_gap[i] << "," << target_time_gap[i] << "," << safe_distance << "\n";
    }
    // Close the CSV file
    logFile.close();
    displayResults(actual_speed, torque_output, error_output, v_target, dt);
    // Return results (for example, speed for now)
    return actual_speed;
}


std::vector<double> Simulation::runSimulationWithChecks(double v_target, double v_initial, double dt, 
double T, double v2_initial, double distance, double sin_amplitude, double sin_freq) {
        int steps = static_cast<int>(T / dt) + 1;
    std::vector<double> time(steps);
    std::vector<double> actual_speed(steps, 0.0);
    std::vector<double> torque_output(steps, 0.0);
    std::vector<double> error_output(steps, 0.0);
    std::vector<double> target_vehicle_speed(steps, v2_initial);
    std::vector<double> distance_between(steps, distance);
    std::vector<double> true_time_gap(steps, 0.0);
    std::vector<double> target_time_gap(steps, time_gap);

    VehicleDynamics vehicleDynamics(r, rho, Cd, A, Cr, m, g);
    ACCController acc = ACCController();

    // Generate the target vehicle speed profile
    if (sin_amplitude > 0 && sin_freq > 0) {
        target_vehicle_speed = sinTarget(v2_initial, sin_amplitude, sin_freq, dt, T);
    }
    
    double v_actual = v_initial;
    double prev_torque = 0;
    double driver_set_speed = v_target;

    // Open a CSV file to log the data
    std::ofstream logFile("simulation_log_ACC.csv");
    logFile << "Time,Error,EgoSpeed,Torque,ACCTargetSpeed,LeadSpeed,DriverSetSpeed,Gap(m),TrueTimeGap,TargetTimeGap,SafeDistance\n";

    for (int i = 0; i < steps; ++i) {
        // Time calculation
        time[i] = i * dt;
        // ACC logic
        double v_lead = target_vehicle_speed[i];
        double distance_to_lead = distance;
        if (i > 0)
            distance_to_lead = distance_between[i - 1];

        bool lead_vehicle_present = distance_to_lead >= 0;
        // ACC logic
        // double v_target = acc.calculateTargetSpeed(distance_to_lead, minimum_distance,v_actual, v_lead, driver_set_speed, lead_vehicle_present,dt);
        // double torque = acc.calculate_torque(v_target, v_actual, tau_max, dt);
        double torque = acc.run_acc(v_actual, driver_set_speed, 0, true, 0, 0, 0, dt, v_lead, distance_to_lead, tau_max, 0);
        double v_target = acc.target_speed;

        // Dynamic constraints
        torque = std::max(-tau_max, std::min(tau_max, torque));
        double torque_rate = (torque - prev_torque) / dt;
        if (std::abs(torque_rate) > max_torque_rate) {
            torque = prev_torque + std::copysign(max_torque_rate * dt, torque_rate);
        }
        prev_torque = torque;
        v_actual = vehicleDynamics.update(torque, v_actual, dt);

        // Update distance between vehicles
        if (i > 0)    
            distance_between[i] = distance_between[i - 1] + (v_lead - v_actual) * dt;

        // Calculate true time gap
        if (v_actual > 0) {
            // if v_actual is near 0 then set time gap to -1
            if (v_actual < 1){
                true_time_gap[i] = -1;}
            else{
                true_time_gap[i] = (distance_between[i]-10) / (v_actual);}
        } else {
            true_time_gap[i] = 0.0;
        }

        // Store results
        actual_speed[i] = v_actual * 3.6;
        torque_output[i] = torque;
        error_output[i] = v_target * 3.6 - v_actual * 3.6;

        double safe_distance = acc.safe_distance;
        // Log the data to the CSV file
        logFile << time[i] << "," << error_output[i] << "," << actual_speed[i] << "," << torque_output[i] << "," 
        << v_target * 3.6 << "," << target_vehicle_speed[i] * 3.6 << "," << driver_set_speed * 3.6 << "," 
        << distance_between[i] << "," << true_time_gap[i] << "," << target_time_gap[i] << "," << safe_distance << "\n";
    }
    // Close the CSV file
    logFile.close();
    displayResults(actual_speed, torque_output, error_output, v_target, dt);
    // just return speed for no reason, maybe send flags for failure success
    return actual_speed;
}

std::vector<double> Simulation::sinTarget(double initial_value, double amplitude, double frequency, double dt, double T) {
    int steps = static_cast<int>(T / dt) + 1;
    std::vector<double> time(steps);
    std::vector<double> target_speed(steps, 0.0);

    for (int i = 0; i < steps; ++i) {
        time[i] = i * dt;
        target_speed[i] = initial_value + amplitude * std::sin(2 * frequency * time[i]);
    }

    return target_speed;
}
void Simulation::displayResults(const std::vector<double>& actual_speed, const std::vector<double>& torque_output,
                    const std::vector<double>& error_output, double v_target, double dt) {
    // convert actual speed to m/s
    double overshoot = (*std::max_element(actual_speed.begin(), actual_speed.end())*0.2778 - v_target) / v_target * (100.0);
    std::cout << "Overshoot: " << overshoot << "%\n";
}
