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

// for integral term
float e_x_sum   = 0.0;
float e_x_old   = 0.0;

float e_y_sum   = 0.0;
float e_y_old   = 0.0;

float e_z_sum   = 0.0;
float e_z_old   = 0.0;

Vector3f qc_des(0.0,0.0,-1.0);
Vector3f qc_des_dot(0.0,0.0,0.0);
Vector3f qc_des_dot_dot(0.0,0.0,0.0);
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

float imu_roll_prev     = 0.0;      // degrees 
float imu_pitch_prev    = 0.0;      // degrees 
float imu_yaw_prev      = 0.0;      // degrees 

float imu_roll_dot  = 0.0;      // degrees/sec
float imu_pitch_dot = 0.0;      // degrees/sec
float imu_yaw_dot   = 0.0;      // degrees/sec

float imu_roll_prev_dot     = 0.0;      // degrees 
float imu_pitch_prev_dot    = 0.0;      // degrees 
float imu_yaw_prev_dot      = 0.0;      // degrees 

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

float quad_x_dot_prev = 0.0;
float quad_y_dot_prev = 0.0;
float quad_z_dot_prev = 0.0;

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

float x_des_prev = 0.0;
float y_des_prev = 0.0;
float z_des_prev = 0.0;

float yaw_initially = 0.0;

int arm_disarm_flag = 0;

//quadcopter parameter
float gravity = 9.81;
float mass_quad = 1.256;

// to get the cable states
Vector3f qc(0.0,0.0,0.0);
Vector3f qc_dot(0.0,0.0,0.0);

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

Vector3f u_final(0.0,0.0,0.0);

float human_x_dot = 0.0;
float human_y_dot = 0.0;
float human_z_dot = 0.0;
float human_yaw_dot = 0.0;

Vector3f u_cable_log(0.0,0.0,0.0);
Vector3f u_quad_log(0.0,0.0,0.0);
Vector3f u_total_log(0.0,0.0,0.0);

int PWM_1_log = 1000;
int PWM_2_log = 1000;
int PWM_3_log = 1000;
int PWM_4_log = 1000;
int PWM_5_log = 1000;
int PWM_6_log = 1000;
int PWM_7_log = 1000;
int PWM_8_log = 1000;

float f_final_log = 0.0;
float M1_final_log = 0.0;
float M2_final_log = 0.0;
float M3_final_log = 0.0;

// For Trajectory planning 
Vector3f x1 (0.0,0.0,0.0);
Vector3f x2 (0.0,0.0,3.0);
Vector3f x3 (6.0,0.0,3.0);
Vector3f x4 (0.0,0.0,3.0);
Vector3f x5 (6.0,0.0,3.0);
Vector3f x6 (0.0,0.0,3.0);
Vector3f x7 (0.0,0.0,0.0);

float max_speed = 1.25;

float Traj_generation_start_time = 0.0;
float Traj_generation_start_time_reset = 0.0;

float t12 = 0.0;
float t23 = 0.0;
float t34 = 0.0;
float t45 = 0.0;
float t56 = 0.0;
float t67 = 0.0;

float t1 = 0.0;
float t2 = 0.0;
float t3 = 0.0;
float t4 = 0.0;
float t5 = 0.0;
float t6 = 0.0;
float t7 = 0.0;

float PWM1_prev = 1000;
float PWM2_prev = 1000;
float PWM3_prev = 1000;
float PWM4_prev = 1000;

Vector3f qc_prev(0.0,0.0,-1.0);
Vector3f qc_prev_dot(0.0,0.0,-1.0);

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
    /////////////////////////////////////
    /// Let's initialize the necessary functions/variables
    ///////////////////////////////////

    ///////////// Checking batter voltage  /////////////
        battery_check();
    ///////////// getting states of quadcopter /////////////
        quad_states();
    ///////////// Taking pilot inputs  /////////////
        pilot_input();

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////// Main logic starts here //////////////////////////
    ////////////////////////////////////////////////////////////////////////////

        PWM_5_log = RC_Channels::get_radio_in(CH_5);
        PWM_6_log = RC_Channels::get_radio_in(CH_6);
        PWM_7_log = RC_Channels::get_radio_in(CH_7);
        PWM_8_log = RC_Channels::get_radio_in(CH_8);

        if (RC_Channels::get_radio_in(CH_7) < 1200){
            // hal.console->printf("%3.3f,%3.3f,%3.3f,%3.3f\n", target_roll, target_pitch, target_yaw_rate,pilot_desired_throttle);

            //////////////////////////////////////////////////////////
            /// Before turning into the TRO code lets initialize and reset the states of the system
            //////////////////////////////////////////////////////////

            yaw_initially = 0.0;
            H_yaw   = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees ;

            float quad_x_ini_inertial =  inertial_nav.get_position_xy_cm().x / 100.0;
            float quad_y_ini_inertial =  -inertial_nav.get_position_xy_cm().y / 100.0;

            quad_x_ini =  cosf(yaw_initially)*quad_x_ini_inertial + sinf(yaw_initially)*quad_y_ini_inertial;
            quad_y_ini = -sinf(yaw_initially)*quad_x_ini_inertial + cosf(yaw_initially)*quad_y_ini_inertial;
            quad_z_ini =  inertial_nav.get_position_z_up_cm() / 100.0;
            x_des      =  quad_x;
            y_des      =  quad_y;
            z_des      =  quad_z;

            if (copter.motors->armed()){
                PWM1 = 1000;
                PWM2 = 1000;
                PWM3 = 1000;
                PWM4 = 1000;
            }else{
                PWM1 = 1000;
                PWM2 = 1000;
                PWM3 = 1000;
                PWM4 = 1000;
            }

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
                if (RC_Channels::get_radio_in(CH_7) > 1600 )
                {
                    Traj_generation_start_time = AP_HAL::millis()/1000.0 - Traj_generation_start_time_reset;
                }else 
                {
                    Traj_generation_start_time_reset = AP_HAL::millis()/1000.0;
                    Traj_generation_start_time = 0.0;
                }
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
        human_x_dot = H_pitch;
        float dt_x = 1.0/5000.0;
        x_des       =  x_des + (-H_pitch) * dt_x;

        if (x_des > 8.0){x_des = 8.0;}
        if (x_des < -8.0){x_des = -8.0;}
        x_des_dot = 0.0;

        // Y direction desired position
        if (H_roll > -2.0 && H_roll < 2.0){H_roll = 0.0;}
        if (H_roll < -2.0){H_roll = H_roll + 2.0;}
        if (H_roll >  2.0){H_roll = H_roll - 2.0;}
        human_y_dot = H_roll;
        float dt_y = 1.0/5000.0;
        y_des       =  y_des + (-H_roll) * dt_y;
        if (y_des > 8.0){y_des = 8.0;}
        if (y_des < -8.0){y_des = -8.0;}
        y_des_dot = 0.0;

        // Z direction desired position
        if (H_throttle > -20.0 && H_throttle < 20.0){H_throttle = 0.0;}
        if (H_throttle < -20.0){H_throttle = H_throttle + 20.0;}
        if (H_throttle >  20.0){H_throttle = H_throttle - 20.0;}
        human_z_dot = H_throttle;
        float dt_z = 1.0/150000.0;
        z_des       =  z_des + H_throttle * dt_z;
        if (z_des > 8.0){z_des = 8.0;}
        if (z_des < -8.0){z_des = -8.0;}
        z_des_dot = 0.0;

        x_des      = low_pass_filter_20_HZ(x_des,x_des_prev);
        x_des_prev = x_des;

        y_des      = low_pass_filter_20_HZ(y_des,y_des_prev);
        y_des_prev = y_des;

        z_des      = low_pass_filter_100_HZ(z_des,z_des_prev);
        z_des_prev = z_des;

        // hal.console->printf("%3.3f, %3.3f, %3.3f\n", x_des,y_des ,z_des);


        // Desired yaw angle of the quadcopter
        human_yaw_dot = H_yaw;

        //////// For auto trajectory generation
        ////////////////

        if (RC_Channels::get_radio_in(CH_7) > 1600 )
        {

            t12 = distance_(x1,x2)/max_speed ;
            t12 = 5.0;
            t23 = distance_(x2,x3)/max_speed ;
            t34 = distance_(x3,x4)/max_speed ;
            t45 = distance_(x4,x5)/max_speed ;
            t56 = distance_(x5,x6)/max_speed ;
            t67 = distance_(x6,x7)/max_speed ;
            t67 = 5.0;

            t1 = 0;
            t2 = t1 + t12;
            t3 = t1 + t12 + t23;
            t4 = t1 + t12 + t23 + t34;
            t5 = t1 + t12 + t23 + t34 + t45;
            t6 = t1 + t12 + t23 + t34 + t45 + t56;
            t7 = t1 + t12 + t23 + t34 + t45 + t56 + t67;

            float t = Traj_generation_start_time;

            if (t <= t2 && t >= t1)
            {
                x_des = Traj_generation(x1[0],x2[0],t1,t2,t);
                y_des = Traj_generation(x1[1],x2[1],t1,t2,t);
                z_des = Traj_generation(x1[2],x2[2],t1,t2,t);
            }

            if (t <= t3 && t >= t2)
            {
                x_des = Traj_generation(x2[0],x3[0],t2,t3,t);
                y_des = Traj_generation(x2[1],x3[1],t2,t3,t);
                z_des = Traj_generation(x2[2],x3[2],t2,t3,t);
            }


            if (t <= t4 && t >= t3)
            {
                x_des = Traj_generation(x3[0],x4[0],t3,t4,t);
                y_des = Traj_generation(x3[1],x4[1],t3,t4,t);
                z_des = Traj_generation(x3[2],x4[2],t3,t4,t);
            }


            if (t <= t5 && t >= t4)
            {
                x_des = Traj_generation(x4[0],x5[0],t4,t5,t);
                y_des = Traj_generation(x4[1],x5[1],t4,t5,t);
                z_des = Traj_generation(x4[2],x5[2],t4,t5,t);
            }

            if (t <= t6 && t >= t5)
            {
                x_des = Traj_generation(x5[0],x6[0],t5,t6,t);
                y_des = Traj_generation(x5[1],x6[1],t5,t6,t);
                z_des = Traj_generation(x5[2],x6[2],t5,t6,t);
            }

            if (t <= t7 && t >= t6)
            {
                x_des = Traj_generation(x6[0],x7[0],t6,t7,t);
                y_des = Traj_generation(x6[1],x7[1],t6,t7,t);
                z_des = Traj_generation(x6[2],x7[2],t6,t7,t);
            }

            if (t > t7)
            {
                x_des = 0.0;
                y_des = 0.0;
                z_des = 0.0;
            }
        }

        // hal.console->printf("%3.3f, %3.3f, %3.3f, %3.3f\n", Traj_generation_start_time, x_des,y_des ,z_des);

        //////// Controller design
        ////////////////////////

        // error defination
        float e_x       = x_des - quad_x;
        float e_x_dot   = x_des_dot - quad_x_dot;

        // hal.console->printf("%3.3f,%3.3f\n",quad_x,quad_x_dot);
        // hal.console->printf("%3.3f\n",10*quad_x);

        // e_x = H_pitch/45;
        // e_x_dot = 0;

        float e_y       = y_des - quad_y;
        float e_y_dot   = y_des_dot - quad_y_dot;
        // hal.console->printf("%3.3f,%3.3f\n",quad_y,quad_y_dot);
        
        // e_y = H_roll/45;
        // e_y_dot = 0;

        float e_z       = z_des - quad_z;
        float e_z_dot   = z_des_dot - quad_z_dot;
        
        e_x = Satuation_func_position_error(e_x);
        e_x_sum = sat_e_X(e_x + e_x_old);
        e_x_old = e_x;

        e_y = Satuation_func_position_error(e_y);
        e_y_sum = sat_e_Y(e_y + e_y_old);
        e_y_old = e_y;
        
        e_z = Satuation_func_position_error(e_z);
        e_z_sum = sat_e_Z(e_z + e_z_old);
        e_z_old = e_z;

        // hal.console->printf("zd= %3.3f, z= %3.3f, ez= %3.3f, z_dot= %3.3f", z_des, quad_z, e_z, quad_z_dot);
        // hal.console->printf("Xd->[%2.2f,%2.2f,%2.2f]", x_des,y_des,z_des);

        float Kp_x      = g.TRO_quad_pos_Kp_x;    // 3.5 (best)
        float Kd_x      = g.TRO_quad_pos_Kd_x;    // 0.8 (best)
        float kI_x      = g.TRO_quad_pos_Ki_x;    // 

        float Kp_y      = g.TRO_quad_pos_Kp_y;    // 3.0 (best)
        float Kd_y      = g.TRO_quad_pos_Kd_y;    // 0.3 (best)
        float kI_y      = g.TRO_quad_pos_Ki_y;    // 

        float Kp_z      = g.TRO_quad_pos_Kp_z;    // 15.0 (best)
        float Kd_z      = g.TRO_quad_pos_Kd_z;    // 4.0 (best)
        float kI_z      = g.TRO_quad_pos_Ki_z;    // 

        // hal.console->printf("P gains-> [%3.3f,%3.3f,%3.3f] ", Kp_x, Kp_y, Kp_z);
        // hal.console->printf("D gains-> [%3.3f,%3.3f,%3.3f] ", Kd_x, Kd_y, Kd_z);
        // hal.console->printf("I gains-> [%3.3f,%3.3f,%3.3f] \n", kI_x, kI_y, kI_z);

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
        Vector3f e_xq_integral(kI_x * e_x_sum, kI_y * e_y_sum,kI_z * e_z_sum);
        Vector3f e3_with_gravity(0.0,0.0, mass_quad * gravity);
        Vector3f u_quad_pos(Matrix_vector_mul(K_xq,e_xq) + Matrix_vector_mul(K_xq_dot,e_xq_dot) + e3_with_gravity + e_xq_integral);

        if (u_quad_pos[2] < 8.0)
        {
            u_quad_pos[2] = 8.0;
        }

        // hal.console->printf("  |  u_q_pos->[%3.3f,%3.3f,%3.3f]\n", u_quad_pos[0],u_quad_pos[1],u_quad_pos[2]);

        //////////
        // cable attitude controller
        //////////

        float Kq1       = g.TRO_cable_Kq1;
        float Kq2       = g.TRO_cable_Kq2;
        float Kq3       = g.TRO_cable_Kq3;

        float Kq1_dot       = g.TRO_cable_Kq1_dot;
        float Kq2_dot       = g.TRO_cable_Kq2_dot;
        float Kq3_dot       = g.TRO_cable_Kq3_dot;

        // hal.console->printf("PWMs-> [%3.3f,%3.3f,%3.3f] ", Kq1, Kq2, Kq3);
        // hal.console->printf("PWMs-> [%3.3f,%3.3f,%3.3f] \n", Kq1_dot, Kq2_dot, Kq3_dot);

        Matrix3f Kq(
                Kq1,0.0,0.0,
                0.0,Kq2,0.0,
                0.0,0.0,Kq3
                );

        Matrix3f Kq_dot(
                Kq1_dot,0.0,0.0,
                0.0,Kq2_dot,0.0,
                0.0,0.0,Kq3_dot
                );

        for (int ii=0; ii<3; ii++)
        {
            qc[ii]      = low_pass_filter_50_HZ(qc[ii],qc_prev[ii]);
            qc_prev[ii] = qc[ii];

            qc_dot[ii]  = low_pass_filter_50_HZ(qc_dot[ii],qc_prev_dot[ii]);
            qc_prev_dot[ii] = qc_dot[ii];
        }

        Vector3f eq(attitude_error_on_s2(qc,qc_des));
        Vector3f eq_dot(attitude_dot_error_on_s2(qc,qc_des, qc_dot, qc_des_dot));

        // hal.console->printf("%3.3f,%3.3f\n", eq[0],eq_dot[0]);
        // hal.console->printf("%3.3f,%3.3f\n", qc[1],qc_dot[1]);

        Vector3f u_cable(Matrix_vector_mul(Kq,eq) + Matrix_vector_mul(Kq_dot,eq_dot));

        // hal.console->printf("eq-> [%3.3f,%3.3f,%3.3f]\n", eq[0],eq[1],eq[2]);

        u_final = u_quad_pos;

        // To enable and disable CAC
        if (RC_Channels::get_radio_in(CH_8) > 1600 )
        {
            u_final = u_quad_pos + u_cable;
        }

        u_cable_log     = u_cable;
        u_quad_log      = u_quad_pos;
        u_total_log     = u_final;

        // u_final         = e3_with_gravity;

        // Vector3f u_final(u_quad_pos + u_cable);

        Vector3f b1c(cosf(H_yaw*PI/180.0), sinf(H_yaw*PI/180.0), 0);       // imu_yaw - degrees
        // hal.console->printf("%3.3f,%3.3f,%3.3f,%3.3f\n", b1c[0],b1c[1],b1c[2],imu_yaw);
        Vector3f b3d( u_final[0]/vector_norm(u_final), u_final[1]/vector_norm(u_final), u_final[2]/vector_norm(u_final));

        Vector3f b2d(Matrix_vector_mul(hatmap(b3d),b1c));
        Vector3f b1d(Matrix_vector_mul(hatmap(b2d),b3d));
        // hal.console->printf("b1d->[%3.3f,%3.3f,%3.3f] | ", b1d[0],b1d[1],b1d[2]);
        // hal.console->printf("b2d->[%3.3f,%3.3f,%3.3f] | ", b2d[0],b2d[1],b2d[2]);
        // hal.console->printf("b3d->[%3.3f,%3.3f,%3.3f] | %3.3f | ", b3d[0],b3d[1],b3d[2],imu_yaw);

        Matrix3f R_d(   b1d[0], b2d[0], b3d[0],
                        b1d[1], b2d[1], b3d[1],
                        b1d[2], b2d[2], b3d[2]
                        );

        // if (RC_Channels::get_radio_in(CH_7) > 1600 )
        // {
            // Vector3f rpy_human((H_roll*PI/180.0)/2.0,(-H_pitch*PI/180.0)/2.0,H_yaw*PI/180.0);
        //     // hal.console->printf("Hpitch-> %3.3f | ",-H_pitch);
            // R_d = eulerAnglesToRotationMatrix(rpy_human);
        // }

        Vector3f Omegad_quadcopter(0.0,0.0,0.0);

        custom_geometric_controller_with_Rotation_matrix(R_d, Omegad_quadcopter, u_final);
}


void ModeStabilize::custom_geometric_controller_with_Rotation_matrix(Matrix3f Rd, Vector3f Omegad ,Vector3f u1){

        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);  // AP_Motors class.cpp libraries

        ///////////////////// Altitude controller /////////////////////

            F  =  two_norm(u1);
            // F = 10.0;
            if (F > 23.54){ F = 23.54; }
            if (F < 8.0){ F =  8.0;  }

            // hal.console->printf("F-> [%3.3f] \n", F);

        ///////////////////// geometric attitude controller /////////////////////

            Vector3f rpy(imu_roll*PI/180.0,imu_pitch*PI/180.0,imu_yaw*PI/180.0);
            Vector3f Omega(imu_roll_dot*PI/180.0,imu_pitch_dot*PI/180.0,imu_yaw_dot*PI/180.0);

            Matrix3f R(eulerAnglesToRotationMatrix(rpy));

            float c2 = 2.0;
            // float c1 = 1.0;

            Vector3f e_R_val        = e_R(R,Rd);
            e_R_val = Satuation_func_e_R(e_R_val);
            // hal.console->printf("eR-> [%3.3f,%3.3f,%3.3f], \n", e_R_val[0], e_R_val[1], e_R_val[2]);

            Vector3f e_Omega_val    = e_Omega(R,Rd,Omega,Omegad);
            Vector3f e_I_val        = e_Omega_val + Vector3f(e_R_val[0]*c2,e_R_val[1]*c2,e_R_val[2]*c2);
            Vector3f e_I_val_sum    = sat_e_I(e_I_val_old + e_I_val);
            e_I_val_old             = e_I_val;
            // hal.console->printf("eR_1 -> %3.3f, e_Omega_val_1 -> %3.3f\n ", e_R_val[0], e_Omega_val[0]);
            // hal.console->printf("%3.3f,%3.3f\n ", e_R_val[2], 10*e_Omega_val[2]);

            // Intertia matrix
            Matrix3f JJ(
                    0.0113, 0.0 , 0.0,
                    0.0, 0.0133,0.0,
                    0.0,0.0,0.0187
            );

            // Gains for geometric controller
            float KR1           = g.TRO_quad_att_KR1;
            float KR2           = g.TRO_quad_att_KR2;
            float KR3           = g.TRO_quad_att_KR3;

            float KOmega1       = g.TRO_quad_att_KOmega1;
            float KOmega2       = g.TRO_quad_att_KOmega2;
            float KOmega3       = g.TRO_quad_att_KOmega3;

            float KI1           = g.TRO_quad_att_KI1;
            float KI2           = g.TRO_quad_att_KI2;
            float KI3           = g.TRO_quad_att_KI3;

            // hal.console->printf("Kp-> [%3.3f,%3.3f,%3.3f] ", KR1, KR2, KR3);
            // hal.console->printf("Kd-> [%3.3f,%3.3f,%3.3f] ", KOmega1, KOmega2, KOmega3);
            // hal.console->printf("Ki-> [%3.3f,%3.3f,%3.3f] \n", KI1, KI2, KI3);

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


            Vector3f M( Matrix_vector_mul(KR,e_R_val) + Matrix_vector_mul(KOmega,e_Omega_val) + Matrix_vector_mul(KI,e_I_val_sum));
            //  + Omega % Matrix_vector_mul(JJ,Omega));
            // M = ( Matrix_vector_mul(KR,e_R_val));
            // M = ( Matrix_vector_mul(KR,e_R_val) + Matrix_vector_mul(KOmega,e_Omega_val));

            Mb1 =  Satuation_func_moments(-M[0]);
            Mb2 =  Satuation_func_moments(-M[1]);
            Mb3 =  Satuation_func_moments(M[2]);

            // hal.console->printf("F-> %3.3f | Mb-> [%3.3f,%3.3f,%3.3f] | \n", F, Mb1,Mb2,Mb3);

            // hal.console->printf("M1->%f,M2->%f,M3->%f,F->%f\n",M[0],M[1],M[2],F);

            // Mb1 =  0.0;
            // Mb2 =  0.0;
            // Mb3 =  0.0;

            // Mb1 = 0.1;
            // Mb2 = 0.0;
            // Mb3 = 0.0;
            // F = 12;

            float FM_devided_FF = 0.0105;

            float function_F1 = F/4.0 - Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
            float function_F2 = F/4.0 + Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) +  Mb3 / (4.0 * FM_devided_FF);
            float function_F3 = F/4.0 + Mb1 / (4.0 * arm_length) - Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);
            float function_F4 = F/4.0 - Mb1 / (4.0 * arm_length) + Mb2 / (4.0 * arm_length) -  Mb3 / (4.0 * FM_devided_FF);

            PWM1 = Inverse_thrust_function(function_F1);
            PWM2 = Inverse_thrust_function(function_F2);
            PWM3 = Inverse_thrust_function(function_F3);
            PWM4 = Inverse_thrust_function(function_F4);
            
            PWM1 = (int) low_pass_filter_100_HZ(PWM1,PWM1_prev);
            PWM2 = (int) low_pass_filter_100_HZ(PWM2,PWM2_prev);
            PWM3 = (int) low_pass_filter_100_HZ(PWM3,PWM3_prev);
            PWM4 = (int) low_pass_filter_100_HZ(PWM4,PWM4_prev);

            PWM1_prev = PWM1;
            PWM2_prev = PWM2;
            PWM3_prev = PWM3;
            PWM4_prev = PWM4;

            PWM_1_log = PWM1;
            PWM_2_log = PWM2;
            PWM_3_log = PWM3;
            PWM_4_log = PWM4;

            // hal.console->printf("PWMs-> [%d,%d,%d,%d] \n", PWM1, PWM2, PWM3, PWM4);

            // hal.console->printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n",H_roll,imu_roll,e_R_val[0],H_pitch,imu_pitch,e_R_val[1],H_yaw,imu_yaw,e_R_val[2]);
            // hal.console->printf("%f,%f,%f,%f,%f,%f\n",H_roll,imu_roll,e_R_val[0],H_pitch,imu_pitch,e_R_val[1]);
            // hal.console->printf("%f,%f,%f\n",e_R_val[0],e_R_val[1],e_R_val[2]);
            // hal.console->printf("%f,%f,%f\n",e_R_val[0],e_R_val[1],e_R_val[2]);

            // PWM1 = 1000;
            // PWM2 = 1000;
            // PWM3 = 1000;
            // PWM4 = 1000;

            f_final_log = F;
            M1_final_log = Mb1;
            M2_final_log = Mb2;
            M3_final_log = Mb3;
            
}


void ModeStabilize::battery_check(){
    // battvolt=copter.battery_volt();
}

Vector3f ModeStabilize::Matrix_vector_mul(Matrix3f R, Vector3f v){

    float first_value   = R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2];
    float second_value  = R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2];
    float third_value   = R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2];
    Vector3f mul_vector(first_value , second_value, third_value);

    return mul_vector;
}

void ModeStabilize::quad_states(){
    // Position in inertial reference frame
    float quad_x_inertial =  inertial_nav.get_position_xy_cm().x / 100.0;
    float quad_y_inertial =  -inertial_nav.get_position_xy_cm().y / 100.0;

    // position in body reference frame
    quad_x =  (cosf(yaw_initially)*quad_x_inertial + sinf(yaw_initially)*quad_y_inertial) - quad_x_ini;
    quad_y = (-sinf(yaw_initially)*quad_x_inertial + cosf(yaw_initially)*quad_y_inertial) - quad_y_ini;

    // quad_y = -quad_y;

    quad_z =  (inertial_nav.get_position_z_up_cm() / 100.0) - quad_z_ini;

    // if (quad_z < 0){quad_z = 0;}
    if (quad_z > 10.0){quad_z = 10.0;}

    // linear velocity in inertial frame of reference
    float quad_x_dot_inertial =  inertial_nav.get_velocity_xy_cms().x /100.0;
    float quad_y_dot_inertial =  -inertial_nav.get_velocity_xy_cms().y /100.0;
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

    // low pass filter in quadcopter's translational velocity

    quad_x_dot      = low_pass_filter_50_HZ(quad_x_dot, quad_x_dot_prev);
    quad_x_dot_prev = quad_x_dot;

    quad_y_dot      = low_pass_filter_50_HZ(quad_y_dot, quad_y_dot_prev);
    quad_y_dot_prev = quad_y_dot;

    quad_z_dot      = low_pass_filter_50_HZ(quad_z_dot, quad_z_dot_prev);
    quad_z_dot_prev = quad_z_dot;

    // low pass filter in quadcopter's attitude

    imu_roll        = low_pass_filter_50_HZ(imu_roll, imu_roll_prev);
    imu_roll_prev   = imu_roll;

    imu_pitch       = low_pass_filter_50_HZ(imu_pitch, imu_pitch_prev);
    imu_pitch_prev  = imu_pitch;

    imu_yaw         = low_pass_filter_50_HZ(imu_yaw, imu_yaw_prev);
    imu_yaw_prev    = imu_yaw;

    // low pass filter in quadcopter's angular velocity

    imu_roll_dot        = low_pass_filter_50_HZ(imu_roll_dot, imu_roll_prev_dot);
    imu_roll_prev_dot   = imu_roll_dot;

    imu_pitch_dot       = low_pass_filter_50_HZ(imu_pitch_dot, imu_pitch_prev_dot);
    imu_pitch_prev_dot  = imu_pitch_dot;

    imu_yaw_dot         = low_pass_filter_50_HZ(imu_yaw_dot, imu_yaw_prev_dot);
    imu_yaw_prev_dot    = imu_yaw_dot;

    // hal.console->printf("%3.3f,%3.3f\n", imu_yaw,imu_yaw_dot);
    // hal.console->printf("%3.3f,%3.3f,%3.3f\n", imu_roll,imu_pitch,imu_yaw);
    // parameter.dsd

    // float pratik = g.my_new_parameter;
    // hal.console->printf("%3.3f\n",pratik);

}

float ModeStabilize::Satuation_func_position_error(float value)
{
    float value_max_error_in_meters = 1.5;
    if (value < -value_max_error_in_meters)
    {
        value = -value_max_error_in_meters;
    }
    if (value > value_max_error_in_meters)
    {
        value = value_max_error_in_meters;
    }
    return value;
}



// void ModeStabilize::cable_states(){
    
//     // float rate_of_Mode_stabilize = 400.0;
//     float rate_of_Mode_stabilize = 1.0;
//     qc_dot = (qc - qc_old)/rate_of_Mode_stabilize;
//     qc_old = qc;

//     // hal.console->printf("%3.3f,%3.3f,%3.3f\n", qc[0],qc[1],qc[2]);
//     // hal.console->printf("%3.3f,%3.3f\n", qc[0],qc_dot[0]);
// }

void ModeStabilize::pilot_input(){

    H_roll      = (double)(channel_roll->get_control_in())/100.0;
    H_roll_dot  = (H_roll - H_roll_prev)/400.0;
    H_roll_prev = H_roll;

    H_pitch     = (double)(channel_pitch->get_control_in())/100.0;
    H_pitch_dot = (H_pitch - H_pitch_prev)/400.0;
    H_pitch_prev= H_pitch;

    H_yaw_rate  = -(double)(channel_yaw->get_control_in()) / 100.0;
    if (H_yaw_rate > -2.0 && H_yaw_rate < 2.0){H_yaw_rate = 0.0;}
    if (H_yaw_rate < -2.0){H_yaw_rate = H_yaw_rate + 2.0;}
    if (H_yaw_rate >  2.0){H_yaw_rate = H_yaw_rate - 2.0;}

    H_throttle  =  (double)channel_throttle->get_control_in()-500.0;
    // hal.console->printf("%f\n",H_yaw_rate);
    float dt_yaw = 1.0/300.0;
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
    
    float p1 = 0.0043;
    float p2 = -0.0533;
    float p3 = 0.2984;
    float p4 = 1.1487;
    
    float PWM_ = 1000.0 * (p1 * value * value * value + p2 * value * value + p3 * value + p4);

    PWM = (int) PWM_;

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

    // Vector3f error_vec(Omega - (matrix_transpose(R)*Rd)*Omegad);
    Vector3f error_vec(Omega);
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

float ModeStabilize::sat_e_X(float value)
{
    float allowable_max_thrust_for_I = 3.0;
    float allowable_min_thrust_for_I = -3.0;

    if (value > allowable_max_thrust_for_I)
    {
        value = allowable_max_thrust_for_I;
    }
    if (value < allowable_min_thrust_for_I){
        value = allowable_min_thrust_for_I;
    }
    return value;
}

float ModeStabilize::sat_e_Y(float value)
{
    float allowable_max_thrust_for_I = 3.0;
    float allowable_min_thrust_for_I = -3.0;

    if (value > allowable_max_thrust_for_I)
    {
        value = allowable_max_thrust_for_I;
    }
    if (value < allowable_min_thrust_for_I){
        value = allowable_min_thrust_for_I;
    }
    return value;
}

float ModeStabilize::sat_e_Z(float value)
{
    float allowable_max_thrust_for_I = 3.0;
    float allowable_min_thrust_for_I = -3.0;

    if (value > allowable_max_thrust_for_I)
    {
        value = allowable_max_thrust_for_I;
    }
    if (value < allowable_min_thrust_for_I){
        value = allowable_min_thrust_for_I;
    }
    return value;
}

float ModeStabilize::Satuation_func_moments(float moment)
{
    float max_thrust_motor_produce = 0.6*9.81;

    float max_thrust_in_Newton =  max_thrust_motor_produce*arm_length*4.0;
    float min_thrust_in_Newton = -max_thrust_motor_produce*arm_length*4.0;
    if (moment > max_thrust_in_Newton)
    {
        moment = max_thrust_in_Newton;
    }
    if (moment < min_thrust_in_Newton)
    {
        moment = min_thrust_in_Newton;
    }

    return moment;
};

Vector3f ModeStabilize::Satuation_func_e_R(Vector3f vec)
{
    float max_e_R =  0.2;
    float min_e_R = -0.2;

    for (int ii=0; ii<3; ii++){
        if (vec[ii] > max_e_R){
            vec[ii] = max_e_R;
        }
        if (vec[ii] < min_e_R){
            vec[ii] = min_e_R;
        }
    }
    return vec;
};


Vector3f ModeStabilize::attitude_error_on_s2(Vector3f q, Vector3f qd)
{
    float q_dot_qd = q[0]*qd[0] + q[1]*qd[1] + q[2]*qd[2];
    
    float eq_1 = q[0]*q_dot_qd - qd[0];
    float eq_2 = q[1]*q_dot_qd - qd[1];
    float eq_3 = q[2]*q_dot_qd - qd[2];

    Vector3f eq_(eq_1, eq_2, eq_3);

    return eq_;
};

Vector3f ModeStabilize::attitude_dot_error_on_s2(Vector3f q, Vector3f qd,Vector3f q_dot, Vector3f qd_dot)
{
    float q_dot_qd      = q[0]*qd[0] + q[1]*qd[1] + q[2]*qd[2];
    float q_dot_qd_dot  = q[0]*qd_dot[0] + q[1]*qd_dot[1] + q[2]*qd_dot[2];

    float qd_dot_mul_q_dot_qd_1 = qd_dot[0]*q_dot_qd ;
    float qd_dot_mul_q_dot_qd_2 = qd_dot[1]*q_dot_qd ;
    float qd_dot_mul_q_dot_qd_3 = qd_dot[2]*q_dot_qd ;

    float qd_mul_q_dot_qd_dot_1 = qd[0]*q_dot_qd_dot ;
    float qd_mul_q_dot_qd_dot_2 = qd[1]*q_dot_qd_dot ;
    float qd_mul_q_dot_qd_dot_3 = qd[2]*q_dot_qd_dot ;

    float eq_dot_1_ = q_dot[0] - qd_dot_mul_q_dot_qd_1 + qd_mul_q_dot_qd_dot_1;
    float eq_dot_2_ = q_dot[1] - qd_dot_mul_q_dot_qd_2 + qd_mul_q_dot_qd_dot_2;
    float eq_dot_3_ = q_dot[2] - qd_dot_mul_q_dot_qd_3 + qd_mul_q_dot_qd_dot_3;

    Vector3f eq_dot_ (eq_dot_1_, eq_dot_2_, eq_dot_3_);

    return eq_dot_;
};

float ModeStabilize::Traj_generation(float p1, float p2, float tt1, float tt2, float t)
{

    float denomenator       = (tt1 - tt2)*(tt1 - tt2)*(tt1 - tt2);
    float first_coeffi      = -(-p2 * tt1 * tt1 * tt1 + 3 * p2 * tt1 * tt1 * tt2 - 3 * p1 * tt1 * tt2 * tt2 + p1 * tt2*tt2*tt2) / denomenator;
    float second_coeffi     = (-6*tt1*tt2 * (p1 - p2)) / denomenator;
    float third_coeffi      = (3*(tt1+tt2) * (p1 - p2)) / denomenator;
    float forth_coeffi      = (-2*(p1 - p2))/denomenator;

    float value = first_coeffi + t * second_coeffi + t*t * third_coeffi + t*t*t* forth_coeffi;
    return value;
};

float ModeStabilize::distance_(Vector3f x, Vector3f y)
{
    Vector3f z(x - y);
    float val = sqrtf(z[0]*z[0] + z[1]*z[1] + z[2]*z[2]);
    return val;
};

float ModeStabilize::low_pass_filter_100_HZ(float PWM_now, float PWM_prev)
{
    float b1_100 = 0.43990085;
    float b2_100 = 0.43990085;
    float a1_100 = 0.12019831;

    float filtered_data = a1_100 * PWM_prev + b1_100 * PWM_now + b2_100 * PWM_prev;
    return filtered_data;
};

float ModeStabilize::low_pass_filter_50_HZ(float now, float prev)
{
    float b1_50 = 0.2819698;
    float b2_50 = 0.2819698;
    float a1_50 = 0.4360604;

    float filtered_data = a1_50 * prev + b1_50 * now + b2_50 * prev;
    return filtered_data;
};

float ModeStabilize::low_pass_filter_20_HZ(float now, float prev)
{
    float b1_20 = 0.13575525;
    float b2_20 = 0.13575525;
    float a1_20 = 0.7284895;

    float filtered_data = a1_20 * prev + b1_20 * now + b2_20 * prev;
    return filtered_data;
};