#include "Copter.h"
#include "mycontroller_usercode.h"
#include <AP_HAL/HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/LogStructure.h>
#include <AP_Logger/AP_Logger.h>  
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <AP_GPS/AP_GPS.h>
#include "GCS_Mavlink.h"

#define PI 3.14159265359

float encoder_roll_feedback     = 0.0;
float encoder_pitch_feedback    = 0.0;

float encoder_roll_dot_feedback     = 0.0;
float encoder_pitch_dot_feedback    = 0.0;

float imu_roll_log      = 0.0;
float imu_pitch_log     = 0.0;
float imu_yaw_log       = 0.0;
char attitude[]         = "50000_50000";
char roll_char[]        = "11111";
char pitch_char[]       = "11111";

Vector3f qc_old(0.0,0.0,0.0);


#ifdef USERHOOK_INIT
void Copter::userhook_init()
{

    // put your initialisation code here
    // this will be called once at start-up

    // hal.serial(1)->begin(115200); // telemetry 1 port Pixhawk Cube Orange
    // hal.serial(2)->begin(115200); // telemetry 2 port Pixhawk Cube Orange
    // hal.serial(3)->begin(115200); // GPS1 port Pixhawk Cube Orange
    hal.serial(4)->begin(115200); // GPS2 port Pixhawk Cube Orange

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    // Log_Write_position();
    // Log_Write_velocity();
    // log_attitude_tracking();
    // log_sys_ID_ph_func();

    log_TRO1_pos_();
    log_TRO1_vel_();
    log_TRO1_hum_();
    log_TRO1_uc_();
    log_TRO1_uq_();
    log_TRO1_ut_();
    log_TRO1_PWM_();
    log_TRO1_FM_();

    // hal.console->printf("Pf %d PWM1 %d PWM2 %d PWM3 %d PWM4 %d Roll %f time %f \n",Pf,PWM1,PWM2,PWM3,PWM4,imu_roll,t_ph_sys_ID);

    imu_roll_log        =  (ahrs.roll_sensor)  / 100.0;     // degrees 
    imu_pitch_log       = -(ahrs.pitch_sensor) / 100.0;     // degrees 
    imu_yaw_log         = 360.0-(ahrs.yaw_sensor)   / 100.0;     // degrees 
    // hal.console->printf("IMU-> [%3.3f,%3.3f,%3.3f] | ",imu_roll_log,imu_pitch_log, imu_yaw_log);

    // hal.console->printf("From usercode \n");
    getEncoderData();
    // hal.console->printf("ph_p %f, th_p - %f\n",encoder_roll_feedback,encoder_pitch_feedback); 

    ///////////// getting states of quadcopter /////////////
    cable_states();

    // char H_roll_[5]    = "";
    // char H_pitch_[5]   = "";
    // char imu_roll_[5]  = "";
    // char imu_pitch_[5] = "";

    // int imu_roll_         = imu_roll*100;
    // int H_roll_           = H_roll*100;
    // int H_pitch_        = H_pitch*100;
    // int imu_roll_dot_       = imu_roll_dot*100;
    // int imu_pitch_      = imu_pitch*100;

// if (H_roll_ > 4500){
//     H_roll_ = 4500;
// }

// if (H_roll_ < -4500){
//     H_roll_ = -4500;
// }

// if (imu_roll_ > 4500){
//     imu_roll_ = 4500;
// }

// if (imu_roll_ < -4500){
//     imu_roll_ = -4500;
// }

    // hal.serial(2)->printf("%f",quad_x);
    // hal.serial(2)->printf(",");
    // hal.serial(2)->printf("%f",quad_y);
    // hal.serial(2)->printf(",");
    // hal.serial(2)->printf("%f\n",quad_z );

    // hal.serial(2)->printf("%f",z_des);
    // hal.serial(2)->printf(",");
    // hal.serial(2)->printf("%f\n",quad_z );

    //////////// For RPI 3 ////////////
    //// Data logging for system identification
    //arm_disarm_flag,Pf,Pm1,Pm2,Pm3,Pm4,PWM1,PWM2,PWM3,PWM4,ph,th,ps,z
    // hal.serial(2)->printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f\n",arm_disarm_flag,Pf,Pm1,Pm2,Pm3,Pm4,PWM1,PWM2,PWM3,PWM4,imu_roll,imu_pitch,imu_yaw,quad_z);

    //// Data logging outdoor////
    // hal.serial(2)->printf("%d,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n",arm_disarm_flag,quad_x,quad_y,quad_z,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw_rate,H_yaw,F,Mb1,Mb2,Mb3);
    // hal.serial(2)->printf("%d,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f\n",arm_disarm_flag,quad_x,quad_y,quad_z,x_des,y_des,z_des,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw_rate,H_yaw,F,Mb1,Mb2,Mb3);

    // hal.console->printf("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",arm_disarm_flag,quad_x,quad_y,quad_z,imu_roll,imu_pitch,imu_yaw,H_roll,H_pitch,H_yaw_rate,H_yaw,F,Mb1,Mb2,Mb3);
    
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // gcs().send_text(MAV_SEVERITY_INFO,"commandGCS: Hi Pratik %3.3f \n",imu_roll_log);

}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
    // getEncoderData();
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::Log_Write_position()
{
    struct log_position pkt = {
        LOG_PACKET_HEADER_INIT(LOG_POSI_MSG),
        time_us  : AP_HAL::micros64(),
        x        : quad_x,
        y        : quad_y,
        z        : quad_z,
        phi      : imu_roll_log,
        theta    : imu_pitch_log,
        psi      : imu_yaw_log,
        phi_p    : encoder_roll_feedback,
        theta_p  : encoder_pitch_feedback,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));

}

void Copter::Log_Write_velocity()
{
    struct log_velocity pkt = {
        LOG_PACKET_HEADER_INIT(LOG_VELO_MSG),
        time_us  : AP_HAL::micros64(),
        x_dot        : quad_x_dot,
        y_dot        : quad_y_dot,
        z_dot        : quad_z_dot,
        phi_dot      : imu_roll_dot,
        theta_dot    : imu_pitch_dot,
        psi_dot      : imu_yaw_dot,
        phi_p_dot    : encoder_roll_dot_feedback,
        theta_p_dot  : encoder_pitch_dot_feedback,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_attitude_tracking()
{
    struct log_att_trac pkt = {
        LOG_PACKET_HEADER_INIT(LOG_ATT_TRA_MSG),
        time_us  : AP_HAL::micros64(),
        phi      : imu_roll_log,
        theta    : imu_pitch_log,
        psi      : imu_yaw_log,
        phi_h    : H_roll,
        theta_h  : H_pitch,
        psi_h    : H_yaw,
        psi_h_dot: H_yaw_rate,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_TRO1_pos_()
{
    struct log_tro1_pos pkt = {
        LOG_PACKET_HEADER_INIT(LOG_TRO1_POS_MSG),
        time_us  : AP_HAL::micros64(),
        pos_x       :quad_x,
        pos_y       :quad_y,
        pos_z       :quad_z,
        q1          :qc[0],
        q2          :qc[1],
        q3          :qc[2],
        att_roll    :imu_roll,
        att_pitch   :imu_pitch,
        att_yaw     :imu_yaw,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_TRO1_vel_()
{
    struct log_tro1_vel pkt = {
        LOG_PACKET_HEADER_INIT(LOG_TRO1_VEL_MSG),
        time_us  : AP_HAL::micros64(),
        vel_x       :quad_x_dot,
        vel_y       :quad_y_dot,
        vel_z       :quad_z_dot,
        q1_dot      :qc_dot[0],
        q2_dot      :qc_dot[1],
        q3_dot      :qc_dot[2],
        roll_dot    :imu_roll_dot,
        pitch_dot   :imu_pitch_dot,
        yaw_dot     :imu_yaw_dot,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_TRO1_hum_()
{
    struct log_tro1_hum pkt = {
        LOG_PACKET_HEADER_INIT(LOG_TRO1_HUM_MSG),
        time_us  : AP_HAL::micros64(),
        h_x_dot       :human_x_dot,
        h_y_dot       :human_y_dot,
        h_z_dot       :human_z_dot,
        h_psi_dot     :human_yaw_dot,
        h_x_des       :x_des,
        h_y_des       :y_des,
        h_z_des       :z_des,
        h_yaw_des     :H_yaw,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_TRO1_uc_()
{
    struct log_tro1_uc pkt = {
        LOG_PACKET_HEADER_INIT(LOG_TRO1_UC_MSG),
        time_us  : AP_HAL::micros64(),
        uc_1       :u_cable_log[0],
        uc_2       :u_cable_log[1],
        uc_3       :u_cable_log[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_TRO1_uq_()
{
    struct log_tro1_uq pkt = {
        LOG_PACKET_HEADER_INIT(LOG_TRO1_UQ_MSG),
        time_us  : AP_HAL::micros64(),
        uq_1       :u_quad_log[0],
        uq_2       :u_quad_log[1],
        uq_3       :u_quad_log[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_TRO1_ut_()
{
    struct log_tro1_ut pkt = {
        LOG_PACKET_HEADER_INIT(LOG_TRO1_UT_MSG),
        time_us  : AP_HAL::micros64(),
        ut_1       :u_total_log[0],
        ut_2       :u_total_log[1],
        ut_3       :u_total_log[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_TRO1_PWM_()
{
    struct log_tro1_pwm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_TRO1_PWM_MSG),
        time_us  : AP_HAL::micros64(),
        pwm__1       : (float) PWM_1_log,
        pwm__2       : (float) PWM_2_log,
        pwm__3       : (float) PWM_3_log,
        pwm__4       : (float) PWM_4_log,
        pwm__5       : (float) PWM_5_log,
        pwm__6       : (float) PWM_6_log,
        pwm__7       : (float) PWM_7_log,
        pwm__8       : (float) PWM_8_log,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::log_TRO1_FM_()
{
    struct log_tro1_fm pkt = {
        LOG_PACKET_HEADER_INIT(LOG_TRO1_FM_MSG),
        time_us  : AP_HAL::micros64(),
        f_log    : f_final_log,
        M1_log   : M1_final_log,
        M2_log   : M2_final_log,
        M3_log   : M3_final_log,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

Matrix3f Copter::eulerAnglesToRotationMatrix(Vector3f rpy){
     // Calculate rotation about x axis
    Matrix3f R_x (
               1,       0,              0,
               0,       cosf(rpy[0]),   -sinf(rpy[0]),
               0,       sinf(rpy[0]),   cosf(rpy[0])
               );

    // Calculate rotation about y axis
    Matrix3f R_y (
               cosf(rpy[1]),    0,      sinf(rpy[1]),
               0,               1,      0,
               -sinf(rpy[1]),   0,      cosf(rpy[1])
               );

    // Calculate rotation about z axis
    Matrix3f R_z (
               cosf(rpy[2]),    -sinf(rpy[2]),      0,
               sinf(rpy[2]),    cosf(rpy[2]),       0,
               0,               0,                  1);

    // Combined rotation matrix
    Matrix3f R = R_z * R_y * R_x;
    return R;
}

Vector3f Copter::Matrix_vector_mul(Matrix3f R, Vector3f v){
    Vector3f mul_vector(
                        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2] ,
                        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2] ,
                        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2]
                        );
    return mul_vector;
}

Matrix3f Copter::hatmap(Vector3f v){
    Matrix3f R (
               0,         -v[2],      v[1],
               v[2],         0 ,     -v[0],
               -v[1],      v[0],       0);
    return R;
}


void Copter::getEncoderData()
{
    bool receiving_data = false;
    int index = 0;
    char startChar = ',';
    char endChar = '/';
    bool new_data = false;

    // hal.console->printf("serial data -> %c\n",hal.serial(4)->read());

    while (hal.serial(4)->available()>0 && new_data == false)
        {
            char temp = hal.serial(4)->read();
            if (receiving_data == true)
            {
                if (temp != endChar)
                {   
                    attitude[index] = temp;
                    index++;
                }
                else
                {
                    // hal.console->printf("Index number -> %d\n",index);
                    attitude[index] = '\0';
                    receiving_data = false;
                    new_data = false;
                    index = 0;
                }
            }
            else if (temp == startChar)
            {
                receiving_data = true;
                index = 0; 
            }
            // hal.console->printf("attitude from while loop-> %s\n",attitude);
        }

        // hal.console->printf("attitude -> %s\n",attitude);

        for (int i = 0; i < 11; i++)
        {
            if (i < 5)  
            {
                roll_char[i]                = attitude[i];
            } else if (i >= 6 && i < 11 )
            {
                pitch_char[i - 6]           = attitude[i];
            }
        }

        int encoder_roll_int      = atoi(roll_char);
        int encoder_pitch_int     = atoi(pitch_char);

        encoder_roll_feedback  = -(float)((encoder_roll_int  - 50000.0) / 100.0);
        encoder_pitch_feedback = -(float)((encoder_pitch_int - 50000.0) / 100.0);
        encoder_pitch_feedback = encoder_pitch_feedback - 8.5;
        // hal.console->printf("%3.3f,", encoder_roll_feedback);
        // hal.console->printf("%3.3f\n", encoder_pitch_feedback);

        if (encoder_roll_feedback > 60.0){
            encoder_roll_feedback = 60.0;
        }
        if (encoder_roll_feedback < -60.0){
            encoder_roll_feedback = -60.0;
        }

        if (encoder_pitch_feedback > 60.0){
            encoder_pitch_feedback = 60.0;
        }
        if (encoder_pitch_feedback < -60.0){
            encoder_pitch_feedback = -60.0;
        }

        // hal.console->printf("ENCO-> [%3.3f,%3.3f] | ",encoder_roll_feedback,encoder_pitch_feedback);

        // hal.console->printf("%3.3f,", encoder_roll_feedback);
        // hal.console->printf("%3.3f\n", encoder_pitch_feedback);

        // hal.serial(2)->printf("%3.3f,", encoder_roll_feedback);
        // hal.serial(2)->printf("%3.3f\n", encoder_pitch_feedback);

        // hal.console->printf("CAM_device_data -> %f,%f\n",encoder_roll_feedback,encoder_pitch_feedback);

        Vector3f rpy(imu_roll*PI/180.0,imu_pitch*PI/180.0,imu_yaw*PI/180.0);
        Vector3f e_3_neg(0,0,-1);
        Matrix3f R(eulerAnglesToRotationMatrix(rpy));

        // Calculate rotation about pitch axis of CAM device
        Matrix3f CAM_R_y (
                cosf(encoder_pitch_feedback*PI/180),    0,      sinf(encoder_pitch_feedback*PI/180),
                0,               1,      0,
                -sinf(encoder_pitch_feedback*PI/180),   0,      cosf(encoder_pitch_feedback*PI/180)
                );

        // Calculate rotation about roll axis of CAM device
        Matrix3f CAM_R_x (
                1,       0,              0,
                0,       cosf(encoder_roll_feedback*PI/180),   -sinf(encoder_roll_feedback*PI/180),
                0,       sinf(encoder_roll_feedback*PI/180),   cosf(encoder_roll_feedback*PI/180)
                );

        qc = Matrix_vector_mul(R,Matrix_vector_mul(CAM_R_x,Matrix_vector_mul(CAM_R_y,e_3_neg)));
        qc = sat_q(qc);

        // hal.console->printf("qc-> [%3.3f,%3.3f,%3.3f] \n", qc[0],qc[1],qc[2]);
}

void Copter::cable_states(){
    
    // float rate_of_Mode_stabilize = 400.0;
    float rate_of_Mode_stabilize = 1.0;
    qc_dot = (qc - qc_old)/rate_of_Mode_stabilize;
    qc_old = qc;

    qc_dot = sat_q_dot(qc_dot);

    // hal.console->printf("%3.3f,%3.3f,%3.3f\n", qc[0],qc[1],qc[2]);
    // hal.console->printf("%3.3f,%3.3f\n", qc[1],qc_dot[1]);
}

Vector3f Copter::sat_q(Vector3f vec){
    float sat_lim = 1;
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

Vector3f Copter::sat_q_dot(Vector3f vec){
    float sat_lim = 3;
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