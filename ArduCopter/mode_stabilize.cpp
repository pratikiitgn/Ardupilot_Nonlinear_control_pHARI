#include "Copter.h"
#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include <math.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Motors/AP_Motors_Class.h>
#include <AP_Logger/LogStructure.h>
#include <AP_Logger/AP_Logger.h>  
#include "mycontroller_usercode.h"
#include "Parameters.h"

// float code_starting_flag = 0;
#define ESC_HZ 490
#define PI 3.14159265359
int code_starting_flag = 0;

// gains values

float Kp_x      = 1.0;    // 0.5 (best)
float Kd_x      = 0.0;    // 0.2 (best)

float Kp_y      = 1.0;    // 0.5 (best)
float Kd_y      = 0.0;    // 0.2 (best)

float Kp_z      = 6.5;    // 0.5 (best)
float Kd_z      = 1.6;    // 0.2 (best)

// Gains for geometric controller
float KR1           = 0.0;
float KOmega1       = 0.0;
float KI1           = 0.0;

float KR2           = 0.0;
float KOmega2       = 0.0;
float KI2           = 0.0;

float KR3           = 0.0;
float KOmega3       = 0.0;
float KI3           = 0.0;

Vector3f e_I_val_old (0.0,0.0,0.0);

int yaw_flag_start  = 0;

float current_time  = 0.0;
float H_roll        = 0.0;
float H_pitch       = 0.0;
float H_yaw_rate    = 0.0;
float H_throttle    = 0.0;
float H_yaw         = 0.0;

float H_roll_dot    = 0.0;
float H_pitch_dot   = 0.0;

float H_pitch_prev  = 0.0;
float H_roll_prev   = 0.0;

float imu_roll      = 0.0;      // degrees 
float imu_pitch     = 0.0;      // degrees 
float imu_yaw       = 0.0;      // degrees 

float imu_roll_dot  = 0.0;      // degrees/sec
float imu_pitch_dot = 0.0;      // degrees/sec
float imu_yaw_dot   = 0.0;      // degrees/sec
float battvolt      = 0.0;

float quad_x = 0.0;
float quad_y = 0.0;
float quad_z = 0.0;

float quad_x_ini = 0.0;
float quad_y_ini = 0.0;
float quad_z_ini = 0.0;

float quad_x_dot = 0.0;
float quad_y_dot = 0.0;
float quad_z_dot = 0.0;

float latitude  = 0.0;
float longitude = 0.0;

float arm_length = 0.1768;

int pwm__thrust_measurement = 1000;
int flag_thrust_measurement = 0;

float x_des  = 0.0;
float y_des  = 0.0;
float z_des  = 0.0;

float x_des_dot  = 0.0;
float y_des_dot  = 0.0;
float z_des_dot  = 0.0;

float yaw_initially = 0.0;

int arm_disarm_flag = 0;

float landing_timer = 0.0;
int landing_timer_flag = 0;
float landing_timer_start = 0.0;

// global variables for pos controller
int y_des_flag = 0;
float timer_y_des = 0.0;
int timer_y_des_flag = 0;
float timer_y_des_start = 0.0;

int x_des_flag          = 0;
float timer_x_des       = 0.0;
int timer_x_des_flag    = 0;
float timer_x_des_start = 0.0;

//quadcopter parameter
float gravity = 9.81;
float mass_quad = 1.256;

// to get the cable states
Vector3f qc(0.0,0.0,0.0);
Vector3f qc_dot(0.0,0.0,0.0);
Vector3f qc_old(0.0,0.0,0.0);

//  
float Final_roll_angle_TRO_single_quad  = 0.0;  // rarnge [-3500 3500]
float Final_pitch_angle_TRO_single_quad = 0.0;  // rarnge [-3500 3500]
float Final_yaw_rate_TRO_single_quad    = 0.0;  // rarnge [-20250 20250]
float Final_throttle_TRO_single_quad    = 0.0;  // rarnge [ 0 1] - offset = 0.219

uint16_t PWM1 = 1000;
uint16_t PWM2 = 1000;
uint16_t PWM3 = 1000;
uint16_t PWM4 = 1000;

float F         = 0.0;
float Mb1       = 0.0;
float Mb2       = 0.0;
float Mb3       = 0.0;



void ModeStabilize::run()
{

    if (code_starting_flag == 0){

        // To intialize the code
        copter.init_rc_out();
        hal.rcout->set_freq( 15, ESC_HZ); //0xFF  0x0F->b'00001111'
        hal.rcout->enable_ch(0);
        hal.rcout->enable_ch(1);
        hal.rcout->enable_ch(2);
        hal.rcout->enable_ch(3);
        code_starting_flag = 1;

    }else{
    // hal.console("%d\n",my_new_parameter);
    /////////////////////////////////////
    /// Let's initialize the necessary functions/variables
    ///////////////////////////////////

    ///////////// Checking batter voltage  /////////////
        battery_check();
    ///////////// getting states of quadcopter /////////////
        quad_states();
    ///////////// getting states of quadcopter /////////////
        cable_states();
    ///////////// Taking pilot inputs  /////////////
        pilot_input();

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////// Main logic starts here //////////////////////////
    ////////////////////////////////////////////////////////////////////////////

        if (RC_Channels::get_radio_in(CH_7) < 1200){
            // hal.console->printf("%3.3f,%3.3f,%3.3f,%3.3f\n", target_roll, target_pitch, target_yaw_rate,pilot_desired_throttle);

            //////////////////////////////////////////////////////////
            /// Before turning into the TRO code lets initialize and reset the states of the system
            //////////////////////////////////////////////////////////

            yaw_initially = 0.0;
            H_yaw   = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees ;

            float quad_x_ini_inertial =  inertial_nav.get_position_xy_cm().x / 100.0;
            float quad_y_ini_inertial =  inertial_nav.get_position_xy_cm().y / 100.0;

            quad_x_ini =  cosf(yaw_initially)*quad_x_ini_inertial + sinf(yaw_initially)*quad_y_ini_inertial;
            quad_y_ini = -sinf(yaw_initially)*quad_x_ini_inertial + cosf(yaw_initially)*quad_y_ini_inertial;
            quad_z_ini =  inertial_nav.get_position_z_up_cm() / 100.0;
            x_des      =  quad_x;
            y_des      =  quad_y;
            z_des      =  quad_z;
            PWM1 = 1000;
            PWM2 = 1000;
            PWM3 = 1000;
            PWM4 = 1000;
            // hal.console->printf("%3.3f,%3.3f\n", quad_z,z_des);
            // hal.console->printf("%3.3f,%3.3f\n", quad_x,x_des);
        }
        else if (RC_Channels::get_radio_in(CH_7) > 1400 )
        {
            // hal.console->printf("%3.3f,%3.3f\n", quad_z,z_des);

            /////////////////////////////////////
            /// TRO 23 controller for single quad starts here
            ////////////////////////////////////

            if (copter.motors->armed()){
                Non_linear_controller_single_quad();
            }else{
                PWM1 = 1000;
                PWM2 = 1000;
                PWM3 = 1000;
                PWM4 = 1000;
            }
            
            /////////////////////////////////////
            /// TRO 23 controller for single quad ends here
            ////////////////////////////////////
        }
    }
}

void ModeStabilize::Non_linear_controller_single_quad(){

        // X direction desired position
        if (H_pitch > -2.0 && H_pitch < 2.0){H_pitch = 0.0;}
        if (H_pitch < -2.0){H_pitch = H_pitch + 2.0;}
        if (H_pitch >  2.0){H_pitch = H_pitch - 2.0;}
        float dt_x = 1.0/5000.0;
        x_des       =  x_des + (-H_pitch) * dt_x;
        if (x_des > 5.0){x_des = 5.0;}
        if (x_des < -5.0){x_des = -5.0;}
        x_des_dot = 0.0;

        // Y direction desired position
        if (H_roll > -2.0 && H_roll < 2.0){H_roll = 0.0;}
        if (H_roll < -2.0){H_roll = H_roll + 2.0;}
        if (H_roll >  2.0){H_roll = H_roll - 2.0;}
        float dt_y = 1.0/5000.0;
        y_des       =  y_des + (-H_roll) * dt_y;
        if (y_des > 5.0){y_des = 5.0;}
        if (y_des < -5.0){y_des = -5.0;}
        y_des_dot = 0.0;

        // Position controller
        if (H_throttle > -20.0 && H_throttle < 20.0){H_throttle = 0.0;}
        if (H_throttle < -20.0){H_throttle = H_throttle + 20.0;}
        if (H_throttle >  20.0){H_throttle = H_throttle - 20.0;}
        float dt_z = 1.0/50000.0;
        z_des       =  z_des + H_throttle * dt_z;
        if (z_des > 5.0){z_des = 5.0;}
        if (z_des < -5.0){z_des = -5.0;}
        z_des_dot = 0.0;

        // error defination
        float e_x       = x_des - quad_x;
        float e_x_dot   = x_des_dot - quad_x_dot;

        e_x = 0;
        e_x_dot = 0;

        float e_y       = y_des - quad_y;
        float e_y_dot   = y_des_dot - quad_y_dot;

        e_y = 0;
        e_y_dot = 0;

        float e_z       = z_des - quad_z;
        float e_z_dot   = z_des_dot - quad_z_dot;
        hal.console->printf("zd= %3.3f, z= %3.3f, ez= %3.3f, z_dot= %3.3f", z_des, quad_z, e_z, quad_z_dot);

        Matrix3f K_xq(
                Kp_x,0.0,0.0,
                0.0,Kp_y,0.0,
                0.0,0.0,Kp_z
                );

        Matrix3f K_xq_dot(
                Kd_x,0.0,0.0,
                0.0,Kd_y,0.0,
                0.0,0.0,Kd_z
                );

        Vector3f e_xq(e_x,e_y,e_z);
        Vector3f e_xq_dot(e_x_dot,e_y_dot,e_z_dot);
        Vector3f e3_with_gravity(0.0,0.0, mass_quad * gravity);
        Vector3f u_quad_pos(Matrix_vector_mul(K_xq,e_xq) + Matrix_vector_mul(K_xq_dot,e_xq_dot) + e3_with_gravity );
        hal.console->printf("  |  u_q_pos_1 -> %3.3f,u_q_pos_1 -> %3.3f, u_q_pos_1 -> %3.3f\n", u_quad_pos[0],u_quad_pos[1],u_quad_pos[2]);
        Vector3f b1c(cosf(imu_yaw*PI/180.0), sinf(imu_yaw*PI/180.0), 0);       // imu_yaw - degrees
        // hal.console->printf("%3.3f,%3.3f,%3.3f,%3.3f\n", b1c[0],b1c[1],b1c[2],imu_yaw);
        Vector3f b3d( u_quad_pos[0]/vector_norm(u_quad_pos), u_quad_pos[1]/vector_norm(u_quad_pos), u_quad_pos[2]/vector_norm(u_quad_pos));
        // hal.console->printf("%3.3f,%3.3f,%3.3f,%3.3f,", b3d[0],b3d[1],b3d[2],imu_yaw);
        Vector3f b2d(Matrix_vector_mul(hatmap(b3d),b1c));
        Vector3f b1d(Matrix_vector_mul(hatmap(b2d),b3d));

        Matrix3f R_d(   b1d[0], b2d[0], b3d[0],
                        b1d[1], b2d[1], b3d[1],
                        b1d[2], b2d[2], b3d[2]
                        );
        Vector3f Omegad_quadcopter(0.0,0.0,0.0);

        custom_geometric_controller_with_Rotation_matrix(R_d, Omegad_quadcopter, u_quad_pos);
}


void ModeStabilize::custom_geometric_controller_with_Rotation_matrix(Matrix3f Rd, Vector3f Omegad ,Vector3f u1){

        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);  // AP_Motors class.cpp libraries

        ///////////////////// Altitude controller /////////////////////

            F  =  two_norm(u1) + 0.5;
            // F = 10.0;
            if (F > 20.0){ F = 20.0; }
            if (F < 0.0){ F =  0.0;  }

        ///////////////////// geometric attitude controller /////////////////////

            Vector3f rpy(imu_roll*PI/180.0,imu_pitch*PI/180.0,imu_yaw*PI/180.0);
            Vector3f Omega(imu_roll_dot*PI/180.0,imu_pitch_dot*PI/180.0,imu_yaw_dot*PI/180.0);

            Matrix3f R(eulerAnglesToRotationMatrix(rpy));

            Matrix3f R_d(   0,0,0,
                        0,0,0,
                        0,0,0
                        );

            float c2 = 2.0;
            // float c1 = 1.0;

            Vector3f e_R_val        = e_R(R,Rd);
            Vector3f e_Omega_val    = e_Omega(R,Rd,Omega,Omegad);
            Vector3f e_I_val        = e_Omega_val + Vector3f(e_R_val[0]*c2,e_R_val[1]*c2,e_R_val[2]*c2);
            Vector3f e_I_val_sum    = sat_e_I(e_I_val_old + e_I_val);
            e_I_val_old             = e_I_val;

        /////////////////////// Manual gain tuning  ///////////////////////

            KR1         = 0.9;  // 0.6 (TB best)  //0.4
            KOmega1     = 21;   // 10.5(TB best)  //0.5
            KI1         = 0.0;  // 0.1 (TB best)  //0.1

            KR2         = 1.3;  // 1.3  (TB good)
            KOmega2     = 24.0; // 24 (TB good)
            KI2         = 0.0;  // 0.1  (TB good)

            KR3         = 4.5;  // 6.0  (TB good)
            KOmega3     = 14.5; // 10.5 (TB good)
            KI3         = 0.1;  // 0.1  (TB good)

            Matrix3f KR(
                        KR1,0.0,0.0,
                        0.0,KR2,0.0,
                        0.0,0.0,KR3
                        );
            Matrix3f KOmega(
                        KOmega1,0.0,0.0,
                        0.0,KOmega2,0.0,
                        0.0,0.0,KOmega3
                        );
            Matrix3f KI(
                        KI1,0.0,0.0,
                        0.0,KI2,0.0,
                        0.0,0.0,KI3
                        );
            // Intertia matrix
            Matrix3f JJ(
                    0.0113, 0.0 , 0.0,
                    0.0, 0.0133,0.0,
                    0.0,0.0,0.0187
            );

            Vector3f M( Matrix_vector_mul(KR,e_R_val) + Matrix_vector_mul(KOmega,e_Omega_val) + Matrix_vector_mul(KI,e_I_val_sum) + Omega % Matrix_vector_mul(JJ,Omega));
            M = ( Matrix_vector_mul(KR,e_R_val));
            // M = ( Matrix_vector_mul(KR,e_R_val) + Matrix_vector_mul(KOmega,e_Omega_val));

            // Mb1 =  M[0];
            // Mb2 =  M[1];
            // Mb3 =  M[2];
            
            // hal.console->printf("%f,%f,%f,%f,%f,%f\n",Mb1,Mb2,Mb3,H_yaw,imu_yaw,F);

            Mb1 =  0.0;
            Mb2 =  0.0;
            Mb3 =  0.0;

            float FM_devided_FF = 0.0105;

            float function_F1 = F/4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
            float function_F2 = F/4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
            float function_F3 = F/4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
            float function_F4 = F/4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

            PWM1 = Inverse_thrust_function(function_F1);
            PWM2 = Inverse_thrust_function(function_F2);
            PWM3 = Inverse_thrust_function(function_F3);
            PWM4 = Inverse_thrust_function(function_F4);

            // hal.console->printf("PWM1-> %d, PWM2-> %d, PWM3-> %d, PWM4-> %d  \n", PWM1, PWM2, PWM3, PWM4);

            // hal.console->printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n",H_roll,imu_roll,e_R_val[0],H_pitch,imu_pitch,e_R_val[1],H_yaw,imu_yaw,e_R_val[2]);
            // hal.console->printf("%f,%f,%f,%f,%f,%f\n",H_roll,imu_roll,e_R_val[0],H_pitch,imu_pitch,e_R_val[1]);
            // hal.console->printf("%f,%f,%f\n",e_R_val[0],e_R_val[1],e_R_val[2]);
            // hal.console->printf("%f,%f,%f\n",e_R_val[0],e_R_val[1],e_R_val[2]);

            // PWM1 = 1000;
            // PWM2 = 1000;
            // PWM3 = 1000;
            // PWM4 = 1000;
}


void ModeStabilize::battery_check(){
    // battvolt=copter.battery_volt();
}

Vector3f ModeStabilize::Matrix_vector_mul(Matrix3f R, Vector3f v){
    Vector3f mul_vector(
                        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2] ,
                        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2] ,
                        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
                        );
    return mul_vector;
}

void ModeStabilize::quad_states(){
    // Position in inertial reference frame
    float quad_x_inertial =  inertial_nav.get_position_xy_cm().x / 100.0;
    float quad_y_inertial =  inertial_nav.get_position_xy_cm().y / 100.0;

    // position in body reference frame
    quad_x =  (cosf(yaw_initially)*quad_x_inertial + sinf(yaw_initially)*quad_y_inertial) - quad_x_ini;
    quad_y = (-sinf(yaw_initially)*quad_x_inertial + cosf(yaw_initially)*quad_y_inertial) - quad_y_ini;

    quad_y = -quad_y;

    quad_z =  (inertial_nav.get_position_z_up_cm() / 100.0) - quad_z_ini;

    // if (quad_z < 0){quad_z = 0;}
    if (quad_z > 5.0){quad_z = 5.0;}

    // linear velocity in inertial frame of reference
    float quad_x_dot_inertial =  inertial_nav.get_velocity_xy_cms().x /100.0;
    float quad_y_dot_inertial =  inertial_nav.get_velocity_xy_cms().y /100.0;
    quad_z_dot                =  inertial_nav.get_velocity_z_up_cms() /100.0;

    // linear velocity in body reference frame
    quad_x_dot =  (cosf(yaw_initially)*quad_x_dot_inertial + sinf(yaw_initially)*quad_y_dot_inertial);
    quad_y_dot = (-sinf(yaw_initially)*quad_x_dot_inertial + cosf(yaw_initially)*quad_y_dot_inertial);

    imu_roll        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
    imu_pitch       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
    imu_yaw         = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 
    imu_roll_dot    =  (ahrs.get_gyro().x);             // degrees/second
    imu_pitch_dot   =  -(ahrs.get_gyro().y);             // degrees/second    
    imu_yaw_dot     = -(ahrs.get_gyro().z);             // degrees/second

    // hal.console->printf("%3.3f,%3.3f\n", imu_yaw,imu_yaw_dot);

}

void ModeStabilize::cable_states(){
    
    // float rate_of_Mode_stabilize = 400.0;
    float rate_of_Mode_stabilize = 1.0;
    qc_dot = (qc - qc_old)/rate_of_Mode_stabilize;
    qc_old = qc;

    // hal.console->printf("%3.3f,%3.3f,%3.3f\n", qc[0],qc[1],qc[2]);
    // hal.console->printf("%3.3f,%3.3f\n", qc[0],qc_dot[0]);
}

void ModeStabilize::pilot_input(){

    H_roll      = (double)(channel_roll->get_control_in())/100.0;
    H_roll_dot  = (H_roll - H_roll_prev)/400.0;
    H_roll_prev = H_roll;

    H_pitch     = (double)(channel_pitch->get_control_in())/100.0;
    H_pitch_dot = (H_pitch - H_pitch_prev)/400.0;
    H_pitch_prev= H_pitch;

    H_yaw_rate  = -(double)(channel_yaw->get_control_in()) / 100.0;
    H_throttle  =  (double)channel_throttle->get_control_in()-500.0;

    float dt_yaw = 1.0/100.0;
    H_yaw = wrap_360(H_yaw + H_yaw_rate*dt_yaw);

}
void ModeStabilize::Satuation_func_Final_roll_angle(float angle){
    float max_angle_allow = 3500.0;
    if (angle > max_angle_allow)
    {
        Final_roll_angle_TRO_single_quad = max_angle_allow;
    }
    if (angle < -max_angle_allow)
    {
        Final_roll_angle_TRO_single_quad = -max_angle_allow;
    }
}

void ModeStabilize::Satuation_func_Final_pitch_angle(float angle){
    float max_angle_allow = 3500.0;
    if (angle > max_angle_allow)
    {
        Final_pitch_angle_TRO_single_quad = max_angle_allow;
    }
    if (angle < -max_angle_allow)
    {
        Final_pitch_angle_TRO_single_quad = -max_angle_allow;
    }
}

void ModeStabilize::Satuation_func_Final_yaw_rate(float angle_rate){
    float max_angle_rate_allow = 20000;
    if (angle_rate > max_angle_rate_allow)
    {
        Final_yaw_rate_TRO_single_quad = max_angle_rate_allow;
    }
    if (angle_rate < -max_angle_rate_allow)
    {
        Final_yaw_rate_TRO_single_quad = -max_angle_rate_allow;
    }
}

void ModeStabilize::Satuation_func_Final_throttle(float throttle_value){
    float max_throttle_value = 1;
    float min_throttle_value = 0;
    if (throttle_value > max_throttle_value)
    {
        Final_throttle_TRO_single_quad = max_throttle_value;
    }
    if (throttle_value < min_throttle_value)
    {
        Final_throttle_TRO_single_quad = min_throttle_value;
    }
}

float ModeStabilize::Satuation_func_final_thrust_In_Newton(float thrust){

    float max_thrust_in_Newton = 2* mass_quad * gravity;
    float min_thrust_in_Newton = 0;
    if (thrust > max_thrust_in_Newton)
    {
        thrust = max_thrust_in_Newton;
    }
    if (thrust < min_thrust_in_Newton)
    {
        thrust = min_thrust_in_Newton;
    }

    return thrust;
}

float ModeStabilize::Satuation_func_final_thrust_from_zero_to_one(float thrust){
    thrust = thrust / (2* mass_quad * gravity);
    return thrust;
}

float ModeStabilize::vector_norm(Vector3f v){
    float norm_ = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    return norm_;
}

Matrix3f ModeStabilize::hatmap(Vector3f v){
    Matrix3f R (
               0,         -v[2],      v[1],
               v[2],         0 ,     -v[0],
               -v[1],      v[0],       0);
    return R;
}

Vector3f ModeStabilize::RotationMatrixToeulerAngles(Matrix3f R){
    
    float phi   = asinf(R[2][1]);
    float theta = asinf(-R[2][0] / cosf(phi));
    float psi   = asinf(-R[0][1] / cosf(phi));

    Vector3f rpy(phi,theta,psi);
    return rpy;
}

int ModeStabilize::Inverse_thrust_function(float value){
    int PWM = 1100;
    
    float p1 = 4.0972;
    float p2 = -5.1304;
    float p3 = 2.9273;
    float p4 = 1.1487;

    PWM = p1 * value*value*value + p2*value*value + p3*value + p4;
    
    if (PWM > 1950){PWM = 1950;}
    if (PWM < 1100){PWM = 1100;}

    return PWM;
}

int ModeStabilize::Inverse_thrust_function(float value){
    int PWM = 1100;
    
    float p1 = 0.0043;
    float p2 = -0.0533;
    float p3 = 0.2984;
    float p4 = 1.1487;
    
    PWM = p1 * value*value*value + p2*value*value + p3*value + p4;
    
    if (PWM > 1950){PWM = 1950;}
    if (PWM < 1100){PWM = 1100;}

    return PWM;
}

Matrix3f ModeStabilize::eulerAnglesToRotationMatrix(Vector3f rpy){
     // Calculate rotation about x axis
    Matrix3f R_x (
               1,       0,              0,
               0,       cosf(rpy[0]),   -sinf(rpy[0]),
               0,       sinf(rpy[0]),   cosf(rpy[0]));
    // Calculate rotation about y axis
    Matrix3f R_y (
               cosf(rpy[1]),    0,      sinf(rpy[1]),
               0,               1,      0,
               -sinf(rpy[1]),   0,      cosf(rpy[1]));
    // Calculate rotation about z axis
    Matrix3f R_z (
               cosf(rpy[2]),    -sinf(rpy[2]),      0,
               sinf(rpy[2]),    cosf(rpy[2]),       0,
               0,               0,                  1);
    // Combined rotation matrix
    Matrix3f R = R_z * R_x * R_y;

    return R;
}

Vector3f ModeStabilize::e_R(Matrix3f R, Matrix3f Rd){

    Vector3f error_vec(vee_map(matrix_transpose(Rd)*R - matrix_transpose(R)*Rd));
    error_vec[0] = 0.5*error_vec[0];
    error_vec[1] = 0.5*error_vec[1];
    error_vec[2] = 0.5*error_vec[2];

    return error_vec;

}


float ModeStabilize::two_norm(Vector3f v){
    float norm_val = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    return sqrtf(norm_val);
}

Vector3f ModeStabilize::e_Omega(Matrix3f R, Matrix3f Rd, Vector3f Omega, Vector3f Omegad){

    Vector3f error_vec(Omega - (matrix_transpose(R)*Rd)*Omegad);
    return error_vec;

}
Matrix3f ModeStabilize::matrix_transpose(Matrix3f R){
    
    Matrix3f R_T(
            R[0][0],R[1][0],R[2][0],
            R[0][1],R[1][1],R[2][1],
            R[0][2],R[1][2],R[2][2]
            );
    return R_T;
}

Vector3f ModeStabilize::vee_map(Matrix3f R){

    Vector3f vector(R[2][1],R[0][2],R[1][0]);
    return vector;
}

Vector3f ModeStabilize::sat_e_I(Vector3f vec){
    float sat_lim = 0.5;
    for (int ii=0; ii<3; ii++){
        if (vec[ii] > sat_lim){
            vec[ii] = sat_lim;
        }
        if (vec[ii] < -sat_lim){
            vec[ii] = -sat_lim;
        }
    }
    return vec;
}
