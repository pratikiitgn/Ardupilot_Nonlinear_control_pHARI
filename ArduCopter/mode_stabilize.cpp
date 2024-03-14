#include "Copter.h"
#include "mycontroller_usercode.h"

float human_roll_command        = 0.0;
float human_pitch_command       = 0.0;
float human_yaw_rate_command    = 0.0;
float human_throttle_command    = 0.0;


float TRO_target_roll_angle     = 0.0;     //  [-45 45] degrees
float TRO_target_pitch_angle    = 0.0;     //  [-45 45] degrees
float TRO_target_yaw_rate       = 0.0;      //  [-20 20] degrees/sec
float TRO_target_throttle       = 0.0;     //  [0 1] 0.38 - is for hovering condition almost

Vector3f qp_old(0.0,0.0,0.0);


// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll, target_pitch;
    // target_roll = 10*10;

    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    
    // hal.console->printf("%3.3f,",  qp[0]);

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


//////////////////////////////////////////////////////////////////////////////////
/////////////////////        Human commands              /////////////////////////
//////////////////////////////////////////////////////////////////////////////////
    // target_roll              [-4500 to 4500]
    // target_pitch             [-4500 to 4500]
    // target_yaw_rate          [-20250 to 20250]
    // pilot_desired_throttle   [0 to 1]

    human_roll_command          = target_roll;
    human_pitch_command         = target_pitch;
    human_yaw_rate_command      = target_yaw_rate;
    human_throttle_command      = pilot_desired_throttle;


//////////////////////////////////////////////////////////////////////////////////
//////////        estimate the required state of the system            ///////////
//////////////////////////////////////////////////////////////////////////////////

    qp_dot[0]       = qp[0] - qp_old[0];
    qp_dot[1]       = qp[1] - qp_old[1];
    qp_dot[2]       = qp[2] - qp_old[2];

    qp_old          = qp;

    hal.console->printf("%3.3f,",   qp[2]);
    hal.console->printf("%3.3f\n",  qp_dot[2]);

//////////////////////////////////////////////////////////////////////////////////
/////////////////////        Final control inputs        /////////////////////////
//////////////////////////////////////////////////////////////////////////////////

    float kp_qp3 = 1;
    float kd_qp3 = 1;

    TRO_target_throttle       = kp_qp3 * (qp[2]) + kd_qp3 * qp_dot[2];     //  [0 1] 0.38 - is for hovering condition almost


    TRO_target_roll_angle     = 10.0;     //  [-45 45] degrees
    TRO_target_pitch_angle    = 10.0;     //  [-45 45] degrees
    TRO_target_yaw_rate       = 0.0;      //  [-20 20] degrees/sec



//////////////////////////////////////////////////////////////////////////////////
////////////        final altitude and attitude controller    ////////////////////
//////////////////////////////////////////////////////////////////////////////////

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(TRO_target_roll_angle*100, TRO_target_pitch_angle*100, TRO_target_yaw_rate*1000);

    // output pilot's throttle
    attitude_control->set_throttle_out(TRO_target_throttle, true, g.throttle_filt);

}

float limit_on_desired_angles(float angle)
{
    if (angle > 45.0)
    {
        angle = 45.0;
    }

    if (angle < -45.0)
    {
        angle = -45.0;
    }
    return angle;
}