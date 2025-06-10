
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#define SIMPLIFIED_RTWTYPES_COMPATIBILITY
#include "rtwtypes.h"
#undef SIMPLIFIED_RTWTYPES_COMPATIBILITY
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include "VehiclePIDController.hpp"
#include "VehiclePIDController.cpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>
#include "Utilities.hpp"
#include "Utilities.cpp"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define u_1_width 1
#define u_2_width 1
#define u_3_width 1
#define u_4_width 1
#define u_5_width 1
#define u_6_width 1
#define u_7_width 1
#define u_8_width 1
#define u_9_width 1
#define u_10_width 1
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
void ACCWrapper_Start_wrapper(void **pW,
			const real_T *Kp, const int_T p_width0,
			const real_T *Kd, const int_T p_width1,
			const real_T *Ki, const int_T p_width2,
			const real_T *pid_speed_control, const int_T p_width3,
			const real_T *d_Kp, const int_T p_width4,
			const real_T *d_Ki, const int_T p_width5,
			const real_T *d_Kd, const int_T p_width6,
			const real_T *time_gap, const int_T p_width7,
			const real_T *time_gap_tolerance, const int_T p_width8,
			const real_T *minimum_distance, const int_T p_width9,
			const real_T *dt, const int_T p_width10)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
/*
 * Custom Start code goes here.
 */
//    ACCController acc(time_gap[0], time_gap_tolerance[0], pid_speed_control[0], Kp[0], Ki[0], Kd[0], d_Kp[0], d_Ki[0], d_Kd[0]);
ACCController* acc = new ACCController(time_gap[0], time_gap_tolerance[0], pid_speed_control[0], Kp[0], Ki[0], Kd[0], d_Kp[0], d_Ki[0], d_Kd[0]);
pW[0] = acc;
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
void ACCWrapper_Outputs_wrapper(const real_T *v_ego,
			const real_T *driver_set_speed,
			const real_T *driver_intended_axl_trq,
			const real_T *acc_enable,
			const real_T *brake_pedal,
			const real_T *throttle_pedal,
			const real_T *ress,
			const real_T *v_lead,
			const real_T *distance_to_lead,
			const real_T *max_deceleration,
			const real_T *max_torque,
			real_T *engine_torque_request,
			void **pW,
			const real_T *Kp, const int_T p_width0,
			const real_T *Kd, const int_T p_width1,
			const real_T *Ki, const int_T p_width2,
			const real_T *pid_speed_control, const int_T p_width3,
			const real_T *d_Kp, const int_T p_width4,
			const real_T *d_Ki, const int_T p_width5,
			const real_T *d_Kd, const int_T p_width6,
			const real_T *time_gap, const int_T p_width7,
			const real_T *time_gap_tolerance, const int_T p_width8,
			const real_T *minimum_distance, const int_T p_width9,
			const real_T *dt, const int_T p_width10)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
ACCController* acc = static_cast<ACCController*>(pW[0]);
bool lead_vehicle_present = distance_to_lead[0] >= 0;
double v_target = acc->calculateTargetSpeed(distance_to_lead[0], minimum_distance[0], v_ego[0], v_lead[0], driver_set_speed[0], lead_vehicle_present, dt[0]);
double torque = acc->calculate_torque(v_target, v_ego[0], max_torque[0], dt[0]);
engine_torque_request[0] = torque;
// bool lead_vehicle_present = distance_to_lead[0] >= 0;
// double v_target = acc.calculateTargetSpeed(distance_to_lead[0], minimum_distance[0],v_ego[0], v_lead[0], driver_set_speed[0], lead_vehicle_present,dt[0]);
// double torque = acc.calculate_torque(v_target, v_ego[0], max_torque[0], dt[0]);
// engine_torque_request[0] = torque;


        
// y0[0] = u0[0] * p0[0] + u1[0] * p1[0];
/* This sample sets the output equal to the input
      y0[0] = u0[0]; 
 For complex signals use: y0[0].re = u0[0].re; 
      y0[0].im = u0[0].im;
      y1[0].re = u1[0].re;
      y1[0].im = u1[0].im;
 */
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


