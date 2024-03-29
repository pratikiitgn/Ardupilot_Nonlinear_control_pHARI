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

Vector3f b1_quad(1.0,0.0,0.0);

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

float quad_x                = 0.0;
float quad_y                = 0.0;
float quad_z                = 0.0;
float quad_x_dot            = 0.0;
float quad_y_dot            = 0.0;
float quad_z_dot            = 0.0;

float quad_roll             = 0.0;
float quad_pitch            = 0.0;
float quad_yaw              = 0.0;
float quad_roll_dot         = 0.0;
float quad_pitch_dot        = 0.0;
float quad_yaw_dot          = 0.0;

float yaw_initially         = 0.0;
float quad_x_ini            = 0.0;
float quad_y_ini            = 0.0;
float quad_z_ini            = 0.0;

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
float kd_qp_3       = 130.0;            // 130 (two quad best)  // 120.0 (outdoor best)  //  120 (best 2, harness)     100.0    (best)
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

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{
    pilot_input();
    quad_states();

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

        float quad_x_ini_inertial   =  inertial_nav.get_position_neu_cm().x / 100.0;
        float quad_y_ini_inertial   =  inertial_nav.get_position_neu_cm().y / 100.0;

        yaw_initially               = quad_yaw;    // degrees [0 360]
        human_des_yaw_command       = quad_yaw;    // degrees [0 360]

        // if (PAMD_yaw > human_des_yaw_command)
        // {   
        delta_yaw = human_des_yaw_command - PAMD_yaw;
        
        // }

        quad_x_ini =  cosf((yaw_initially*PI/180.0))*quad_x_ini_inertial + sinf((yaw_initially*PI/180.0))*quad_y_ini_inertial;
        quad_y_ini = -sinf((yaw_initially*PI/180.0))*quad_x_ini_inertial + cosf((yaw_initially*PI/180.0))*quad_y_ini_inertial;

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
//////////        filtering the payload attitude and its rate          ///////////



    float qp_1_fil      = limit_on_q(simple_fil_low_pos(5, fil_qp1_array, qp[0]));
    float qp_2_fil      = limit_on_q(simple_fil_low_pos(5, fil_qp2_array, qp[1]));
    float qp_3_fil      = limit_on_q(simple_fil_low_pos(5, fil_qp3_array, qp[2]));

    float qp_1_dot_fil  = limit_on_q_dot(simple_fil_low_pos(5, fil_qp1_dot_array, qp_dot[0]));
    float qp_2_dot_fil  = limit_on_q_dot(simple_fil_low_pos(5, fil_qp2_dot_array, qp_dot[1]));
    float qp_3_dot_fil  = limit_on_q_dot(simple_fil_low_pos(5, fil_qp3_dot_array, qp_dot[2]));

//////////        filtering the payload attitude and its rate          ///////////

    float qc_2_1_fil    = limit_on_q(simple_fil_low_pos(5, fil_qc_21_array, qc_2[0]));
    float qc_2_2_fil    = limit_on_q(simple_fil_low_pos(5, fil_qc_22_array, qc_2[1]));
    float qc_2_3_fil    = limit_on_q(simple_fil_low_pos(5, fil_qc_23_array, qc_2[2]));

    float qc_2_1_dot_fil  = limit_on_q_dot(simple_fil_low_pos(5, fil_qc_21_dot_array, qc_2_dot[0]));
    float qc_2_2_dot_fil  = limit_on_q_dot(simple_fil_low_pos(5, fil_qc_22_dot_array, qc_2_dot[1]));
    float qc_2_3_dot_fil  = limit_on_q_dot(simple_fil_low_pos(5, fil_qc_23_dot_array, qc_2_dot[2]));

    // hal.console->printf("%3.3f,",   qc_2_1_fil);
    // hal.console->printf("%3.3f,",   qc_2_2_fil);
    // hal.console->printf("%3.3f\n",  qp_3_fil);
    // hal.console->printf("%3.3f\n",  100*qc_2_1_dot_fil);
    // hal.console->printf("%3.3f\n",  100*qc_2_2_dot_fil);

//////////////////////////////////////////////////////////////////////////////////
//////////////////        cable attitude controller    /////////////////////////
//////////////////////////////////////////////////////////////////////////////////
    qc_2_fil[0]     = qc_2_1_fil;
    qc_2_fil[1]     = qc_2_2_fil;
    qc_2_fil[2]     = qc_2_3_fil;

    qc_2_dot_fil[0] = qc_2_1_dot_fil;
    qc_2_dot_fil[1] = qc_2_2_dot_fil;
    qc_2_dot_fil[2] = qc_2_3_dot_fil;

    eqc_2           = Matrix_vector_mul(hatmap(qc_2_fil), Matrix_vector_mul(hatmap(qc_2_fil), qc_2_des));
    eqc_2_dot       = qc_2_dot_fil;

    u2_CAC2[0]      =  kp_qc_2_1 * eqc_2[0] + kd_qc_2_1 * eqc_2_dot[0];
    u2_CAC2[1]      =  kp_qc_2_2 * eqc_2[1] + kd_qc_2_2 * eqc_2_dot[1];
    u2_CAC2[2]      =  kp_qc_2_3 * eqc_2[2] + kd_qc_2_3 * eqc_2_dot[2];

//////////////////////////////////////////////////////////////////////////////////
//////////////////        Payload attitude controller    /////////////////////////
//////////////////////////////////////////////////////////////////////////////////
    qp_fil[0]       = qp_1_fil;
    qp_fil[1]       = qp_2_fil;
    qp_fil[2]       = qp_3_fil;

    Matrix3f R_z_delta_yaw_deg(
        cosf((delta_yaw/180.0)*PI),   -sinf((delta_yaw/180.0)*PI),      0.0,
        sinf((delta_yaw/180.0)*PI),    cosf((delta_yaw/180.0)*PI),      0.0,
        0.0,                             0.0,                           1.0);

    qp_fil = Matrix_vector_mul(R_z_delta_yaw_deg, qp_fil);

    qp_dot_fil[0]   = qp_1_dot_fil;
    qp_dot_fil[1]   = qp_2_dot_fil;
    qp_dot_fil[2]   = qp_3_dot_fil;

    qp_des[0]       = cosf(human_des_yaw_command/180.0*PI);
    qp_des[1]       = sinf(human_des_yaw_command/180.0*PI);
    qp_des[2]       = 0.0;



    // hal.console->printf("%3.3f,",  qp_des[0]);
    // hal.console->printf("%3.3f,",  qp_des[1]);
    // hal.console->printf("%3.3f ---- ", qp_des[2]);

    // hal.console->printf("%3.3f,",  qp[0]);
    // hal.console->printf("%3.3f,",  qp[1]);
    // hal.console->printf("%3.3f\n", qp[2]);

    eq_p            = Matrix_vector_mul(hatmap(qp_fil), Matrix_vector_mul(hatmap(qp_fil), qp_des));
    eq_p_dot        = qp_dot_fil;
    // hal.console->printf("%3.3f,%3.3f,%3.3f,", eq_p[0], eq_p[1], eq_p[2]);
    // hal.console->printf("%3.3f,%3.3f \n", eq_p[2], 10*eq_p_dot[2]);
    // hal.console->printf("%3.3f,%3.3f \n", eq_p[2], 10*eq_p_dot[2]);

    u2_PAC[0]       =  kp_qp_1 * eq_p[0] + kd_qp_1 * eq_p_dot[0];
    u2_PAC[1]       =  kp_qp_2 * eq_p[1] + kd_qp_2 * eq_p_dot[1];
    u2_PAC[2]       =  kp_qp_3 * eq_p[2] + kd_qp_3 * eq_p_dot[2];

    // hal.console->printf("%3.3f,%3.3f \n", kp_qp_3 * eq_p[2], kd_qp_3 * eq_p_dot[2]);
    // hal.console->printf("%3.3f,%3.3f, | ", kp_qp_1 * eq_p[0], kd_qp_1 * eq_p_dot[0]);
    // hal.console->printf("%3.3f,%3.3f, | ", kp_qp_2 * eq_p[1], kd_qp_2 * eq_p_dot[1]);

    // hal.console->printf("u2_PAC -> %3.3f, %3.3f, %3.3f, | ", u2_PAC[0], u2_PAC[1], u2_PAC[2]);

/////////////        summation of all the control inputs        //////////////////
    float coeficiant_constant_hover_value = 0.45*2;

    Vector3f gravity_compensation_vec(0.0,0.0,coeficiant_constant_hover_value*mq*gravity_acc);

    u2                  = u2_PAC + u2_CAC2 + gravity_compensation_vec;

    // u2[0]               = 5;
    // u2[1]               = 5;
    // u2[2]               = 15;

    if (u2[2] < 0.0){u2[2] = 0.0;}

    float u2_norm       = norm_of_vector(u2);

    b_3_des[0]          = u2[0] / u2_norm;
    b_3_des[1]          = u2[1] / u2_norm;
    b_3_des[2]          = u2[2] / u2_norm;
    // hal.console->printf("%3.3f, %3.3f, %3.3f \n", b_3_des[0], b_3_des[1], b_3_des[2]);
    // human_des_yaw_command = 10.0;
    b_1_c[0]            = cosf((human_des_yaw_command/180.0*PI));
    b_1_c[1]            = sinf((human_des_yaw_command/180.0*PI));
    b_1_c[2]            = 0.0;
    // hal.console->printf("%3.3f, %3.3f, %3.3f \n", b_1_c[0], b_1_c[1], b_1_c[2]);

    b_2_des             = cross_product(b_3_des, b_1_c);
    b_1_des             = cross_product(b_2_des, b_3_des);

    hal.console->printf("Human Yaw des -> %3.3f \n", human_des_yaw_command);

    hal.console->printf("b_1_des -> %3.3f, %3.3f, %3.3f, |\n", b_1_des[0], b_1_des[1], b_1_des[2]);
    hal.console->printf("b_2_des -> %3.3f, %3.3f, %3.3f, |\n", b_2_des[0], b_2_des[1], b_2_des[2]);
    hal.console->printf("b_3_des -> %3.3f, %3.3f, %3.3f, |\n", b_3_des[0], b_3_des[1], b_3_des[2]);


    Matrix3f R_quad_attitude(
            b_1_des[0], b_2_des[0], b_3_des[0],
            b_1_des[1], b_2_des[1], b_3_des[1],
            b_1_des[2], b_2_des[2], b_3_des[2]
            );

    rpy_des = Rotation_matrix_to_Euler_angle(R_quad_attitude);
    // hal.console->printf("%3.3f Yaw_des -> ", human_des_yaw_command);
    hal.console->printf("RPY_des ->  %3.3f, %3.3f, %3.3f \n", rpy_des[0], rpy_des[1], rpy_des[2]);
    hal.console->printf("Quad_RPY -> %3.3f, %3.3f, %3.3f, |\n", quad_roll, quad_pitch, quad_yaw);
    // hal.console->printf("%3.3f, %3.3f \n", quad_yaw, rpy_des[2]);

    if (u2_norm > 2*mq*gravity_acc) { u2_norm = 2*mq*gravity_acc; }
    if (u2_norm < 0.0){u2_norm = 0.0; }

    float u2_scaled = u2_norm / (2*mq*gravity_acc);
    // hal.console->printf("%3.3f \n", u2_scaled);

/////////////////////        yaw angle controller        /////////////////////////

    yaw_current_vector[0]   = cosf((PI/2.0) - (quad_yaw/180.0*PI));
    yaw_current_vector[1]   = sinf((PI/2.0) - (quad_yaw/180.0*PI));
    yaw_current_vector[2]   = 0.0;

    yaw_desired_vector[0]   = cosf((PI/2.0) - (human_des_yaw_command/180.0*PI));
    yaw_desired_vector[1]   = sinf((PI/2.0) - (human_des_yaw_command/180.0*PI));
    yaw_desired_vector[2]   = 0.0;

    // yaw_desired_vector[0]   = cosf( -(rpy_des[2]/180.0*PI) + (PI/2.0));
    // yaw_desired_vector[1]   = sinf( -(rpy_des[2]/180.0*PI) + (PI/2.0));
    // yaw_desired_vector[2]   = 0.0;

    // hal.console->printf("%3.3f, %3.3f,\n", quad_yaw, human_des_yaw_command);

    error_yaw_vector        = Matrix_vector_mul(hatmap(yaw_current_vector), yaw_desired_vector);
    third_value_of_error_yaw_vector     = error_yaw_vector[2];
    third_value_of_error_yaw_vector_dot = (third_value_of_error_yaw_vector - third_value_of_error_yaw_vector_old);

    third_value_of_error_yaw_vector_old = third_value_of_error_yaw_vector;

    float third_value_of_error_yaw_vector_dot_fil = 200*simple_fil_low_pos(5, third_value_of_error_yaw_vector_dot_array, third_value_of_error_yaw_vector_dot);

    // float error_yaw_vector_norm = norm_of_vector(error_yaw_vector);
    // hal.console->printf("%3.3f, %3.3f\n", third_value_of_error_yaw_vector, third_value_of_error_yaw_vector_dot_fil);

    if (third_value_of_error_yaw_vector_dot_fil >  3.0) { third_value_of_error_yaw_vector_dot_fil = 3.0; }
    if (third_value_of_error_yaw_vector_dot_fil < -3.0) { third_value_of_error_yaw_vector_dot_fil = -3.0; }

    if (third_value_of_error_yaw_vector >  1.0) { third_value_of_error_yaw_vector = 1.0; }
    if (third_value_of_error_yaw_vector < -1.0) { third_value_of_error_yaw_vector = -1.0; }

//////////////////////////////////////////////////////////////////////////////////
/////////////////////        Final control inputs        /////////////////////////
//////////////////////////////////////////////////////////////////////////////////

    float phi_fil      = simple_fil_low_pos(3, fil_phi_des_array, rpy_des[0]);
    float theta_fil    = simple_fil_low_pos(3, fil_theta_des_array, rpy_des[1]);

    TRO_target_throttle       = u2_scaled;                              //  [0 1] 0.38 - is for hovering condition almost
    // TRO_target_throttle       = 0.38;                                   //  [0 1] 0.38 - is for hovering condition almost

    TRO_target_roll_angle     = phi_fil;                                //  [-45 45] degrees
    // TRO_target_roll_angle     = 0.0;                                    //  [-45 45] degrees

    TRO_target_pitch_angle    = -theta_fil;                              //  [-45 45] degrees
    // TRO_target_pitch_angle    = 0.0;                                    //  [-45 45] degrees

    TRO_target_yaw_rate       = 20.0 * third_value_of_error_yaw_vector + 3.0 * third_value_of_error_yaw_vector_dot_fil;     //  [-20 20] degrees/sec
    // TRO_target_yaw_rate       = human_yaw_rate_command/1000.0;       //  [-20 20] degrees/sec    

    // hal.console->printf("%3.3f, %3.3f, %3.3f \n", quad_yaw, human_des_yaw_command, error_yaw_vector_norm);

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
    // if (print_counter < 5){
    //     print_counter+=1;
    // }
    // else{
    // b1_quad[0]       = cosf(quad_yaw/180.0*PI);
    // b1_quad[1]       = sinf(quad_yaw/180.0*PI);
    // b1_quad[2]       = 0.0;

        print_counter   = 0;
        // hal.console->printf("(%3.1f,%3.1f,%3.1f)", PAMD_yaw, quad_yaw, human_des_yaw_command);
        // hal.console->printf("%3.1f,%3.1f\n", PAMD_yaw, quad_yaw);

        // hal.console->printf(" | ");
        // hal.console->printf("(%1.2f,", qp_fil[0]);
        // hal.console->printf("%1.2f,",  qp_fil[1]);
        // hal.console->printf("%1.2f)",  qp_fil[2]);

        // hal.console->printf(" | ");
        // hal.console->printf("(%1.2f,", b1_quad[0]);
        // hal.console->printf("%1.2f,",  b1_quad[1]);
        // hal.console->printf("%1.2f)",  b1_quad[2]);
        
        // hal.console->printf("(%3.2f,", human_des_yaw_command);
        // hal.console->printf("(%1.2f,", b_1_c[0]);
        // hal.console->printf("%1.2f,",  b_1_c[1]);
        // hal.console->printf("%1.2f)\n",  b_1_c[2]);

        // hal.console->printf(" | ");
        // hal.console->printf("(%1.2f,", u2_PAC[0]);
        // hal.console->printf("%1.2f,",  u2_PAC[1]);
        // hal.console->printf("%1.2f)\n",  u2_PAC[2]);
    // }


//////////////////////////////////////////////////////////////////////////////////
////////////        final altitude and attitude controller    ////////////////////
//////////////////////////////////////////////////////////////////////////////////
/////////////////////        set limits on the inputs       /////////////////////////
    TRO_target_roll_angle       = limit_on_desired_angles(TRO_target_roll_angle);
    TRO_target_pitch_angle      = limit_on_desired_angles(TRO_target_pitch_angle);
    TRO_target_yaw_rate         = limit_on_yaw_rate(TRO_target_yaw_rate);
    TRO_target_throttle         = limit_on_thurst_val(TRO_target_throttle);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(TRO_target_roll_angle*100, TRO_target_pitch_angle*100, TRO_target_yaw_rate*1000);

    // output pilot's throttle
    attitude_control->set_throttle_out(TRO_target_throttle, true, g.throttle_filt);



}

float ModeStabilize::simple_fil_low_pos(int iteration, float array[], float current_value)
{
    for (int h = iteration-1; h >= 0; h--)
    {
        if (h >= 1)
        {
            array[h] = array[h-1];
        }
        else if (h == 0)
        {
            array[0] = current_value;
        }
    }
    float array_sum = 0.0;
    for (int h = 0; h < iteration; h++)
    {
        array_sum += array[h];
    }

    float filterd_value = array_sum/iteration;

    return filterd_value;
}

float ModeStabilize::limit_on_desired_angles(float angle)
{
    float max_angle_value = 45.0;
    // float max_angle_value = 20.0;
    if (angle > max_angle_value)
    {
        angle = max_angle_value;
    }

    if (angle < -max_angle_value)
    {
        angle = -max_angle_value;
    }
    return angle;
}

float ModeStabilize::limit_on_yaw_rate(float angle_dot)
{
    float value = 20.0;
    if (angle_dot > value)
    {
        angle_dot = value;
    }

    if (angle_dot < -value)
    {
        angle_dot = -value;
    }
    return angle_dot;
}

float ModeStabilize::limit_on_thurst_val(float val)
{
    float max_value = 1.0;
    float min_value = 0.0;

    if (val > max_value)
    {
        val = max_value;
    }

    if (val < min_value)
    {
        val = min_value;
    }
    return val;
}

float ModeStabilize::limit_on_q(float q_)
{
    float max_value = 1.0;

    if (q_ > max_value)
    {
        q_ = max_value;
    }

    if (q_ < -max_value)
    {
        q_ = -max_value;
    }
    return q_;
}

float ModeStabilize::limit_on_q_dot(float q_)
{
    float max_value = 3.0;

    if (q_ > max_value)
    {
        q_ = max_value;
    }

    if (q_ < -max_value)
    {
        q_ = -max_value;
    }
    return q_;
}

Matrix3f ModeStabilize::hatmap(Vector3f v){
    Matrix3f R (
               0,    -v[2],      v[1],
               v[2],    0,     -v[0],
               -v[1],      v[0],       0);
    return R;
}

Vector3f ModeStabilize::Matrix_vector_mul(Matrix3f R, Vector3f v){
    Vector3f mul_vector(
                        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2] ,
                        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2] ,
                        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
                        );
    return mul_vector;
}

Matrix3f ModeStabilize::matrix_transpose(Matrix3f R){
    
    Matrix3f R_T(
            R[0][0],R[1][0],R[2][0],
            R[0][1],R[1][1],R[2][1],
            R[0][2],R[1][2],R[2][2]
            );
    return R_T;
}

float ModeStabilize::norm_of_vector(Vector3f v)
{
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

void ModeStabilize::quad_states(){
    // Position in inertial reference frame
    // inertial_nav.get_position_neu_cm().x;
    inertial_nav.get_velocity_neu_cms().x;

    float quad_x_inertial =  inertial_nav.get_position_neu_cm().x / 100.0;
    float quad_y_inertial =  inertial_nav.get_position_neu_cm().y / 100.0;

    // position in body reference frame
    quad_x =  ( cosf((quad_yaw*PI/180.0))*quad_x_inertial + sinf((quad_yaw*PI/180.0))*quad_y_inertial) - quad_x_ini;
    quad_y =  (-sinf((quad_yaw*PI/180.0))*quad_x_inertial + cosf((quad_yaw*PI/180.0))*quad_y_inertial) - quad_y_ini;

    quad_y =  -quad_y;
    quad_z =  (inertial_nav.get_position_neu_cm().z / 100.0) - quad_z_ini;

    // linear velocity in inertial frame of reference
    float quad_x_dot_inertial =  inertial_nav.get_velocity_neu_cms().x /100.0;
    float quad_y_dot_inertial =  inertial_nav.get_velocity_neu_cms().y /100.0;
    quad_z_dot                =  inertial_nav.get_velocity_neu_cms().z /100.0;

    // linear velocity in body reference frame
    quad_x_dot =  (cosf((quad_yaw*PI/180.0))*quad_x_dot_inertial + sinf((quad_yaw*PI/180.0))*quad_y_dot_inertial);
    quad_y_dot = (-sinf((quad_yaw*PI/180.0))*quad_x_dot_inertial + cosf((quad_yaw*PI/180.0))*quad_y_dot_inertial);

    quad_roll        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
    quad_pitch       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
    quad_yaw         =  360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 
    quad_roll_dot    =   (ahrs.get_gyro().x);             // degrees/second
    quad_pitch_dot   =  -(ahrs.get_gyro().y);             // degrees/second    
    quad_yaw_dot     =  -(ahrs.get_gyro().z);             // degrees/second

}

void ModeStabilize::pilot_input()
{
    float H_yaw_rate__        = human_yaw_rate_command / 1000.0;
    float dt_yaw            = 1.0/100.0;
    human_des_yaw_command   = wrap_360(human_des_yaw_command - H_yaw_rate__*dt_yaw);
}

Vector3f ModeStabilize::cross_product(Vector3f v1, Vector3f v2)
{
    return Matrix_vector_mul(hatmap(v1), v2);
}

Vector3f ModeStabilize::Rotation_matrix_to_Euler_angle(Matrix3f R)
{
    // if (R[2][0] != 1.0)
    // {
    //     if (R[2][0] != 1.0)
    //     {
        // th_des = -asinf(R[2][0]);
        // float th2 = PI - th1;

        // ph_des = atan2f( R[2][1] / (cosf(th_des)), R[2][2] / (cosf(th_des)));
        // float ps2 = atan2f( R[2][1] / (cosf(th2)), R[2][2] / (cosf(th2)));

        // ps_des = atan2f( R[1][0] / (cosf(th_des)), R[0][0] / (cosf(th_des)));
    
        // }
    // }

    // ph_des  = asinf(R[2][1]);
    // th_des  = asinf(-cosf(R[2][0]));
    // // th_des  = atan2f(-R[2][1], R[2][2]);
    // ps_des  = atan2f(R[0][1], R[1][1]);

    // float th____ = asinf(cosf(human_des_yaw_command) * R[0][2] + sinf(human_des_yaw_command) * R[1][2]);
    // hal.console->printf("%3.3f ---- > \n",th____*180.0/PI);s

    float sy = sqrtf(R[0][0] * R[0][0] +  R[1][0] * R[1][0]);
    ph_des = atan2f(R[2][1] , R[2][2]);
    th_des = atan2f(-R[2][0], sy);
    ps_des = atan2f(R[1][0], R[0][0]);


    Vector3f des_roll_pitch_yaw(ph_des*180.0/PI, th_des*180.0/PI, ps_des*180.0/PI);

    return des_roll_pitch_yaw;
}