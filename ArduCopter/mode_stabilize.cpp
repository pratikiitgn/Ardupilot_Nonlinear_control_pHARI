#include "Copter.h"
#include "mycontroller_usercode.h"
#define PI 3.14159265359

float human_roll_command        = 0.0;
float human_pitch_command       = 0.0;
float human_yaw_rate_command    = 0.0;
float human_throttle_command    = 0.0;
float human_des_yaw_command     = 0.0;

float TRO_target_roll_angle     = 0.0;     //  [-45 45] degrees
float TRO_target_pitch_angle    = 0.0;     //  [-45 45] degrees
float TRO_target_yaw_rate       = 0.0;      //  [-20 20] degrees/sec
float TRO_target_throttle       = 0.0;     //  [0 1] 0.38 - is for hovering condition almost

float fil_qp1_array[]           = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_qp2_array[]           = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_qp3_array[]           = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

float fil_qp1_dot_array[]       = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_qp2_dot_array[]       = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_qp3_dot_array[]       = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

float fil_qc_21_array[]         = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_qc_22_array[]         = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_qc_23_array[]         = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

float fil_qc_21_dot_array[]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_qc_22_dot_array[]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_qc_23_dot_array[]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

float fil_phi_des_array[]       = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_theta_des_array[]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

float fil_Human_x_acceleration_array[]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_Human_y_acceleration_array[]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float fil_Human_z_acceleration_array[]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

Vector3f b1_quad(1.0,0.0,0.0);

Vector3f quad2_pos(1.0,0.0,0.0);
Vector3f quad2_vel(1.0,0.0,0.0);

Vector3f qc_2_fil(0.0,0.0,-1.0);
Vector3f qc_2_dot_fil(0.0,0.0,0.0);
Vector3f qc_2_des(0.0,0.0,-1.0);

Vector3f qp_fil(1.0,0.0,0.0);
Vector3f qp_dot_fil(0.0,0.0,0.0);
Vector3f qp_des(1.0,0.0,0.0);

Vector3f eqc_2(0.0,0.0,0.0);
Vector3f eqc_2_dot(0.0,0.0,0.0);

Vector3f eq_p(0.0,0.0,0.0);
Vector3f eq_p_dot(0.0,0.0,0.0);

Vector3f u2_PAC(0.0,0.0,0.0);
Vector3f u2_CAC2(0.0,0.0,0.0);
Vector3f u2(0.0,0.0,0.0);

float mq                = 1.236;
float gravity_acc       = 9.81;

Vector3f b_1_des(1.0,0.0,0.0);
Vector3f b_2_des(0.0,1.0,0.0);
Vector3f b_3_des(0.0,0.0,1.0);
Vector3f b_1_c(1.0,0.0,0.0);

Vector3f yaw_current_vector(1.0,0.0,0.0);
Vector3f yaw_desired_vector(1.0,0.0,0.0);
Vector3f error_yaw_vector(1.0,0.0,0.0);
float third_value_of_error_yaw_vector       = 0.0;
float third_value_of_error_yaw_vector_old   = 0.0;
float third_value_of_error_yaw_vector_dot   = 0.0;
float third_value_of_error_yaw_vector_dot_array[]     = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

float quad_roll             = 0.0;
float quad_pitch            = 0.0;
float quad_yaw              = 0.0;
float quad_roll_dot         = 0.0;
float quad_pitch_dot        = 0.0;
float quad_yaw_dot          = 0.0;

float yaw_initially         = 0.0;

Vector3f rpy_des(0.0,0.0,0.0);
float ph_des                = 0.0;
float th_des                = 0.0;
float ps_des                = 0.0;

////////////////// PD gains

float kp_qp_1       = 2.85;             // 2.85 (two quad best) // 2.7 (outdoor best) // 2.5 (Harness)
float kd_qp_1       = 16.0;             // 16.0 (two quad best) // 13.0 (outdoor best) // 10 (Harness)
// float ki_qp_1    = 0.0;

float kp_qp_2       = 2.85;             // 2.85 (two quad best) // 2.7 (outdoor best) // 2.5 (harness)   // 4 (good 1)
float kd_qp_2       = 16.0;             // 16.0 (two quad best)  // 13.0 (outdoor best)  // 10  (harness)    // 2 (good 1)
// float ki_qp_2    = 0.0;

float kp_qp_3       = 12.0;             // 12.0 (two quad best) // 12.0 (outdoor best) //  12  (best 2, harness)     18.0     (best)
float kd_qp_3       = 150.0;            // 130 (two quad best)  // 120.0 (outdoor best)  //  120 (best 2, harness)     100.0    (best)
// float ki_qp_3

float kp_qc_2_1     = 3.0;              // 3.0 (two quad best)  // 3.0 (outdoor best)  // 4  (Harness)
float kd_qc_2_1     = 45.0;             // 45 (two quad best)  // 45.0 (outdoor best)  // 45 (Harness)
// float ki_qp_1    = 0.0;

float kp_qc_2_2     = 3.0;              // 3.0 (two quad best)  // 3.0 (outdoor best)   // 4  (Harness)
float kd_qc_2_2     = 45.0;             // 45 (two quad best)  // 45.0 (outdoor best)   // 45 (Harness)
// float ki_qp_2    = 0.0;

float kp_qc_2_3     = 0.0;        //
float kd_qc_2_3     = 0.0;        //  
// float ki_qp_3    = 0.0;

float delta_yaw     = 0.0;

int print_counter   = 0;
float ch_8_state    = 0.0;

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero
               || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
        // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block

        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    float pilot_desired_throttle = get_pilot_desired_throttle();

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
}
