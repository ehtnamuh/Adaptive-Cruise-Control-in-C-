#include "VehiclePIDController.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>
#include "Utilities.hpp"
using namespace std;

// Add a standby mode for the ACC System
// ACC button on sends car to standby mode, ACC button off sends car to off mode
// When the driver overrides the ACC system, the ACC system should go to standby mode
// Driver can resume the ACC system from the standby mode using the Set Speed button
// Only retain the set speed when the ACC system is in standby mode
// if ACC button is turned off, clear the set speed
// Fill out the default Axle Torque message so that not ECU gets upset

// Instead of create S-Function Wrappers, Create a C-Code caller and Function Import Wizard
// Create a C-Code caller and Function Wizard

// Use a Gain Scheduler for the ACC Controller

// Have all the ACC indicators on the team added display
// use stock ACC light for the ACC system
// add dyno mode switch. enable disable decrement
ACCController::ACCController(double time_gap, double time_gap_tolerance, double pid_speed_control,
double kp, double ki, double kd, double d_kp, double d_ki, double d_kd)
        : time_gap(time_gap), time_gap_tolerance(time_gap_tolerance), pid_speed_control(pid_speed_control),Kp(kp), Ki(ki), Kd(kd), d_Kd(d_kd), d_Kp(d_kp), d_Ki(d_ki) ,integral_error(0), prev_error(0) {
            // print the loaded parameters
            std::cout << "Loaded ACC Controller Parameters: " << std::endl;
            std::cout << "Time Gap: " << time_gap << std::endl;
            std::cout << "Time Gap Tolerance: " << time_gap_tolerance << std::endl;
            std::cout << "Kp: " << Kp << std::endl;
            std::cout << "Ki: " << Ki << std::endl;
            std::cout << "Kd: " << Kd << std::endl;
            std::cout << "d_Kp: " << d_Kp << std::endl;
            std::cout << "d_Ki: " << d_Ki << std::endl;
            std::cout << "d_Kd: " << d_Kd << std::endl;
            std::cout << "PID Speed Control: " << pid_speed_control << std::endl;
            state = OFF;
        }

ACCController::ACCController() {
    time_gap = 1.5;
    time_gap_tolerance = 0.1;
    pid_speed_control = 0;
    Kp = 100;
    Ki = 0.82;
    Kd = 0;
    d_Kp = 0.3;
    d_Ki = 0.01;
    d_Kd = 0.0;
    integral_error = 0;
    prev_error = 0;
    state = CRUISE;
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
        
}

double ACCController::run_acc(double v_ego, double driver_set_speed, double driver_intented_axle_torque, bool acc_enable,
double brake_pedal_percent, double throttle_pedal_percent, int ress_position, double dt, double v_lead, double distance_to_lead, double tau_max, double max_deceleration) {
    if (!feature_enable_precheck(v_ego, driver_set_speed, driver_intented_axle_torque, acc_enable, brake_pedal_percent, throttle_pedal_percent)) {
        return -1;
    }
    double torque = 0.0;
    if (ress_position != 0) {
        driver_set_speed = process_ress_button(ress_position);
    }
    bool lead_vehicle_present = distance_to_lead >= 0; 
    // Skip target speed calculation if the Sensor fusion is not available
    target_speed = calculateTargetSpeed( distance_to_lead,  minimum_distance, v_ego, v_lead,  driver_set_speed, lead_vehicle_present,  dt);
    target_torque = calculate_torque(target_speed, v_ego, tau_max, dt);
    return target_torque;
}

double ACCController::process_ress_button(int ress_position) {
    if (ress_position == 0) {
        return driver_set_speed;
    } else if (ress_position == 1) {
        return (++driver_set_speed);
    } else if (ress_position == 2) {
        return (driver_set_speed + 5);
    } else if (ress_position == -1) {
        return (--driver_set_speed);
    } else if (ress_position == -2) {
        return (driver_set_speed - 5);
    } else{
        return driver_set_speed;
    }
}

bool ACCController::feature_enable_precheck(double v_ego, double driver_set_speed, double driver_intented_axle_torque, bool acc_enable, 
double brake_pedal_percent, double throttle_pedal_percent) {
    // Check for subsystem communication failure (checking if last received message is older than 10 s) or Team added / GM invalid flags for ACC 02.3
    // Check for freshness of critical input, if input is older than 2 seconds, return false
    if (!acc_enable) {
        return false;
    }
    if (brake_pedal_percent > 5) {
        return false;
    }
    if (throttle_pedal_percent > 5 && driver_intented_axle_torque > target_torque*gear_ratio) {
        return false; // pause ACC
    }
    // < 20 mph // This requirement has been removed, egage ACC at any speed Enable ACC at 0 for CARB
    if (v_ego < 8.8 && state == OFF) {
        return false;
    }
    return true;
}

double ACCController::calculateTargetSpeed(double distance, double minimum_distance, double v_ego, double lead_vehicle_speed, double driver_set_speed, bool lead_vehicle_present, double dt) {
    if (!lead_vehicle_present) {
        return driver_set_speed;
    }
    double target_speed = driver_set_speed;
    double relative_speed = lead_vehicle_speed - v_ego;

    if (distance < minimum_distance) {
        return 0.0; // If the distance is less than the minimum distance, stop the vehicle
    }
    // make the max deceleration a parameter to tune
    double stop_time = (0.5 * v_ego * v_ego / (1.1))/v_ego; // Time to stop the vehicle
    double time_gap = this->time_gap;
    if (time_gap < stop_time) {
        cout << "Stop Time: " << stop_time << endl;
        time_gap = stop_time;
    }

    double safe_distance = minimum_distance + v_ego * time_gap; // Safe distance based on current speed and time gap
    this->safe_distance = safe_distance;
    double tolerance_distance = v_ego * time_gap_tolerance; // Tolerance distance based on current speed and time gap
    if (distance > safe_distance && state != FOLLOW) {
        state = CRUISE;
    } else if (distance > safe_distance + tolerance_distance ) {
        state = CRUISE;
    } 
    else if (distance <= (safe_distance + tolerance_distance) || distance >= safe_distance - tolerance_distance) {
        state = FOLLOW;
    } else {
        state = STOP;
    }

    if (state == CRUISE) {
        target_speed = driver_set_speed;
    } else if (state == FOLLOW) {
        target_speed = lead_vehicle_speed ;
        if (pid_speed_control >= 1) {
            double distance_error =  distance - safe_distance;
            double derivative_error = relative_speed * dt;
            distance_integral_error += distance_error * dt;
            double gain = distance_error * d_Kp + distance_integral_error * d_Ki + derivative_error * d_Kd;
            target_speed = lead_vehicle_speed  + gain;
            if (target_speed < lead_vehicle_speed) {
                target_speed = max(lead_vehicle_speed*0.9, target_speed);
            }
        }
        if (target_speed > driver_set_speed) {
            target_speed = driver_set_speed;
        }
        // prev_distance_error = distance_error;
        // cout << "Distance Error: " << distance_error << endl;
        // cout << "Integral Error: " << distance_integral_error << endl;
        // cout << "Derivative Error: " << derivative_error << endl;
        // cout << "Gain: " << gain << endl;
        // cout << "Safe Distance: " << safe_distance << endl;
    } else if (state == STOP) {
        target_speed = 0.0;
    }
    return target_speed;
}

double ACCController::calculate_torque(double target, double actual, double tau_max, double dt, double v_lead, double distance_to_lead,
double minimum_distance, double driver_set_speed, bool lead_vehicle_present) {
    double v_ego = actual;
    double v_target = calculateTargetSpeed(distance_to_lead, minimum_distance, v_ego, v_lead, driver_set_speed, lead_vehicle_present, dt);
    return calculate_torque(v_target, actual, tau_max, dt);
}

double ACCController::calculate_torque(double target, double actual, double tau_max, double dt) {
    double error = target - actual;
    integral_error += error * dt;
    double derivative_error = (error - prev_error) / dt;
    double output = Kp * error + Ki * integral_error + Kd * derivative_error;
    // Clip the output using tau_max and max_torque_rate
    // Keep in mind having multiple filters, sth. , Let PCM check the EngineTrq output
    // IF our request is out of bounds, CAV can choose to not care 
    // Only case we care is when ACC is trying to brake to avoid collision, but PCM is not braking, So we flash user 
    // to brake manually, 5 to seconds to react
    output =  std::max(-tau_max, output);
    output =  std::min(tau_max, output);
    prev_error = error;
    // stability analysis
    return output;
}


