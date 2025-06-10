#ifndef VEHICLE_PID_CONTROLLER_H
#define VEHICLE_PID_CONTROLLER_H

#include <cmath>

class ACCController {
public:
    ACCController(double time_gap, double time_gap_tolerance, double pid_speed_control,
    double kp, double ki, double kd, double d_kp, double d_ki, double d_kd);
    ACCController();

    double calculateTargetSpeed(double distance, double minimum_distance, double v_ego, 
    double lead_vehicle_speed, double driver_set_speed, bool lead_vehicle_present, double dt);
    double calculate_torque(double target, double actual, double tau_max, double dt);
    double calculate_torque(double target, double actual, double tau_max, double dt, 
    double v_lead, double distance_to_lead, double minimum_distance, double driver_set_speed, bool lead_vehicle_present);
    bool feature_enable_precheck(double v_ego, double driver_set_speed, double driver_intented_axle_torque, bool acc_enable,
    double brake_pedal_percent, double throttle_pedal_percent);
    double process_ress_button(int ress_position);
    double run_acc(double v_ego, double driver_set_speed, double driver_intented_axle_torque, bool acc_enable,
                    double brake_pedal_percent, double throttle_pedal_percent, int ress_position, 
                    double dt, double v_lead, double distance_to_lead, double tau_max, double max_deceleration);


private:
    double time_gap;
    double time_gap_tolerance;
    double pid_speed_control;
    double Kp, Ki, Kd;
    double d_Kp, d_Ki, d_Kd;
    double integral_error;
    double prev_error;
    double prev_distance_error;
    double distance_integral_error;
    double driver_set_speed;
    double minimum_distance;
    enum VehicleState {
        CRUISE,
        FOLLOW,
        STOP,
        STANDBY,
        OFF
    };
    VehicleState state;
public:
    double safe_distance;
    double target_speed;
    double target_torque;
    double gear_ratio = 11.8;
};


#endif // VEHICLE_PID_CONTROLLER_H
