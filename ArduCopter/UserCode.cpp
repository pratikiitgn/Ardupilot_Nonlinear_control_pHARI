#include "Copter.h"
#include <AP_HAL/HAL.h>
#include "mycontroller_usercode.h"

#define PI 3.14159265359

float CAM_roll      = 0.0;
float CAM_pitch     = 0.0;

float HHD_encoder_1 = 0.0;
float HHD_encoder_2 = 0.0;
float HHD_safety_swith_state = 0.0;

float HHD_roll      = 0.0;
float HHD_pitch     = 0.0;
float HHD_yaw       = 0.0;

float HHD_local_acc_X = 0.0;
float HHD_local_acc_Y = 0.0;
float HHD_local_acc_Z = 0.0;

char cable_attitude[]           = "50000_50000";
char HHD_Attitude_data[]        = "500000_500000_500000";
char HHD_Acceleration_data[]    = "500000_500000_500000";
char HHD_Encoder_data[]         = "50000_50000_0";

char CAM_roll_char[]            = "50000";
char CAM_pitch_char[]           = "50000";

char HHD_first_encoder_char[]   = "50000";
char HHD_second_encoder_char[]  = "50000";
char HHD_safety_switch_char[]   = "0";

char HHD_roll_char[]            = "500000";
char HHD_pitch_char[]           = "500000";
char HHD_yaw_char[]             = "500000";

char HHD_local_acc_X_char[]     = "500000";
char HHD_local_acc_Y_char[]     = "500000";
char HHD_local_acc_Z_char[]     = "500000";

Vector3f HHD_Acceleration(0.0,0.0,0.0);
Vector3f HHD_local_acceleration(0.0,0.0,0.0);

int flag_start_stop_int         = 0;

Vector3f qc_2(0.0,0.0,-1.0);
Vector3f qc_2_dot(0.0,0.0,0.0);
Vector3f qc_2_old(0.0,0.0,0.0);

Vector3f qp(1.0,0.0,0.0);
Vector3f qp_local(1.0,0.0,0.0);
Vector3f qp_dot(0.0,0.0,0.0);
Vector3f qp_old(0.0,0.0,0.0);

Matrix3f R_HHD(
    1.0,0.0,0.0,
    0.0,1.0,0.0,
    0.0,0.0,1.0);

int HHD_Acceleration_port   = 2;
int HHD_Attitude_port       = 3;
int HHD_Encoders_port       = 1;
int CAM_device_port         = 4;

//           parameter prefix 
// UART4  -  SERIAL3_           - GPS      - GPS1   - now we are setting it as a Payload Attitude
// UART8  -  SERIAL4_           - SERIAL4  - GPS2

// USART2 -  SERIAL1_             TELEM1     TELEM1
// USART3 -  SERIAL2_             TELEM2     TELEM2

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    hal.serial(CAM_device_port)->begin(115200);                 // GPS 2        port Pixhawk Cube Orange - CAM  device
    hal.serial(HHD_Attitude_port)->begin(230400);               // GPS 1        port Pixhawk Cube Orange - HHD Attitude
    hal.serial(HHD_Acceleration_port)->begin(230400);           // telemetry 2  port Pixhawk Cube Orange - PAMD device
    hal.serial(HHD_Encoders_port)->begin(115200);                // telemetry 1  port Pixhawk Cube Orange - 

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here

    // hal.console->printf("Roll -> %f\n",quad_roll);
    get_HHD_attitude_data();
    get_HHD_Acceleration_data();
    get_HHD_Encoders_Data();
    get_CAM_device_Data();

    // Enable data logging functions
    Log_quad_pos_data_follower();                   // log_quad_pos_        | LOG_QUAD_POS_MSG  |   QPOS
    Log_quad_vel_data_follower();                   // log_quad_vel_        | LOG_QUAD_VEL_MSG  |   QVEL
    Log_quad_RPY_data_follower();                   // log_quad_RPY_        | LOG_QUAD_RPY_MSG  |   QRPY
    Log_quad_angular_velocity_data_follower();      // log_quad_ANG_VEL_    | LOG_QUAD_AVG_MSG  |   QAVG
    Log_cable_2_attitude_data_follower();           // log_Cab2_ATT_        | LOG_CABL_ATT_MSG  |   CATT
    Log_cable_2_attitude_dot_data_follower();       // log_Cab2_ATT_dot_    | LOG_CABL_DOT_MSG  |   CDOT
    Log_payload_attitude_data_follower();           // log_Pay_ATT_CMD_     | LOG_PYLD_ATT_MSG  |   PATT
    Log_payload_attitude_dot_data_follower();       // log_Pay_ATT_dot_     | LOG_PYLD_DOT_MSG  |   PDOT
    Log_u2_PAC_follower();                          // log_u2_PAC           | LOG_U2_PAC_MSG    |   UPAC
    Log_u2_CAC2_follower();                         // log_u2_CAC2          | LOG_U2_CAC2_MSG   |   U2C2
    Log_u2_follower();                              // log_u2_              | LOG_U2_MSG        |   U2U2
    Log_qp_des_from_HHD();                          // log_qp_des_          | LOG_QP_DES_MSG    |   QPDD
    Log_Exp_start_stop();                           // log_exp_st_sp        | LOG_EXP_STP_MSG   |   EXSP      

    Log_cable_2_attitude_data_follower();           // log_Cab2_ATT_        | LOG_CABL_ATT_MSG  |   CATT
    Log_HHD_attitude();                             // log_hhd_att          | LOG_HHD_ATT_MSG   |   HHDA
    Log_HHD_acceleration();                         // log_hhd_acc          | LOG_HHD_ACC_MSG   |   HHDC
    Log_HHD_encoders();                             // log_HHD_ENCO         | LOG_HHD_ENCO_MSG  |   HHDE
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here    
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here

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
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::get_HHD_attitude_data()

{
    bool receiving_data = false;
    int index           = 0;
    char startChar      = ',';
    char endChar        = '/';
    bool new_data       = false;

    // hal.console->printf("serial data -> %c\n",hal.serial(HHD_Attitude_port)->read());

    while (hal.serial(HHD_Attitude_port)->available()>0 && new_data == false)
        {
            char temp = hal.serial(HHD_Attitude_port)->read();
            // hal.console->printf("serial data -> %c\n",temp);
            if (receiving_data == true)
            {
                if (temp != endChar)
                {
                    HHD_Attitude_data[index] = temp;
                    index++;
                }
                else
                {
                    // hal.console->printf("Index number -> %d\n",index);
                    HHD_Attitude_data[index] = '\0';
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
        }

        // hal.console->printf("attitude -> %s\n",HHD_Attitude_data);

        for (int i = 0; i < 7; i++)
        {
                HHD_roll_char[i]       = HHD_Attitude_data[i];
                HHD_pitch_char[i]      = HHD_Attitude_data[i+7];
                HHD_yaw_char[i]        = HHD_Attitude_data[i+14];
        }

        int HHD_roll_int       = atoi(HHD_roll_char);
        int HHD_pitch_int      = atoi(HHD_pitch_char);
        int HHD_yaw_int        = atoi(HHD_yaw_char);

        HHD_roll  =  (float)((HHD_roll_int  - 500000.0) / 100.0);
        HHD_pitch =  (float)((HHD_pitch_int - 500000.0) / 100.0);
        float HHD_yaw_raw   =  (float)((HHD_yaw_int - 500000.0) / 100.0);

        if (HHD_yaw_raw > 0.0){
            HHD_yaw = HHD_yaw_raw;
        }else{
            HHD_yaw = 360.0 + HHD_yaw_raw;
        }

        HHD_yaw = HHD_yaw - 90.0;

        if (HHD_yaw < 0.0){
            HHD_yaw = 360.0 + HHD_yaw;
        }

        // hal.console->printf("%3.2f, %3.2f \n", HHD_yaw, quad_yaw);

        if (HHD_roll > 90.0){
            HHD_roll = 90.0;
        }
        if (HHD_roll < -90.0){
            HHD_roll = -90.0;
        }

        if (HHD_pitch > 90.0){
            HHD_pitch = 90.0;
        }
        if (HHD_pitch < -90.0){
            HHD_pitch = -90.0;
        }

        // if (HHD_yaw > 180.0){
        //     HHD_yaw = 180.0;
        // }
        // if (HHD_yaw < -180.0){
        //     HHD_yaw = -180.0;
        // }

        // hal.console->printf("HHD_attitude-> ");
        // hal.console->printf("%3.3f,", HHD_roll);
        // hal.console->printf("%3.3f,", HHD_pitch);
        // hal.console->printf("%3.3f\n", HHD_yaw);
        // hal.console->printf(" | ");

        Vector3f rpy_vector(HHD_roll*PI/180.0,HHD_pitch*PI/180.0,HHD_yaw*PI/180.0);
        R_HHD   = eulerAnglesToRotationMatrix(rpy_vector);

        Vector3f e_1(1.0,0,0.0);

        // qp_des  = Matrix_vector_mul(R_HHD,e_1);

        // hal.console->printf("%3.3f,", HHD_encoder_1);
        // hal.console->printf("%3.3f\n", HHD_encoder_2);

        Matrix3f HH_R_theta (
               cosf(HHD_encoder_1*PI/180.0),    0,      sinf(HHD_encoder_1*PI/180.0),
               0,               1,      0,
               -sinf(HHD_encoder_1*PI/180.0),   0,      cosf(HHD_encoder_1*PI/180.0)
               );

        Matrix3f HH_R_phi (
              cosf(HHD_encoder_2*PI/180),    -sinf(HHD_encoder_2*PI/180),      0,
               sinf(HHD_encoder_2*PI/180),    cosf(HHD_encoder_2*PI/180),      0,
               0,               0,                  1);

        Vector3f p_1(1.0,0,0.0);

        qp_local = Matrix_vector_mul(HH_R_phi,Matrix_vector_mul(HH_R_theta,p_1));
        // qp       = Matrix_vector_mul(R_HHD,qp_local);
        qp       = qp_local;

        Vector3f qp_temp(Matrix_vector_mul(HH_R_theta,p_1));

        // hal.console->printf("%3.3f,",  qp[0]);
        // hal.console->printf("%3.3f,",  qp[1]);
        // hal.console->printf("%3.3f\n", qp[2]);

        // hal.console->printf("%3.3f,",  qp[0]);
        // hal.console->printf("%3.3f,",  qp[1]);
        // hal.console->printf("%3.3f\n", qp[2]);

        // hal.console->printf("%3.3f,",  qp_des[0]);
        // hal.console->printf("%3.3f,",  qp_des[1]);
        // hal.console->printf("%3.3f\n", qp_des[2]);

        // qp_local    = Matrix_vector_mul(R_payload,e_1);
        // hal.console->printf("%3.3f,", qp[0]);
        // hal.console->printf("%3.3f,", qp[1]);
        // hal.console->printf("%3.3f\n", qp[2]);

        // float constant_orientation = -PI/2.0;
        // Matrix3f R_z_plus_90_deg(
        //        cosf(constant_orientation),   -sinf(constant_orientation),      0,
        //        sinf(constant_orientation),    cosf(constant_orientation),      0,
        //        0,                             0,                               1);

        // qp = Matrix_vector_mul(R_z_plus_90_deg, Matrix_vector_mul(R_payload, e_1));

        qp_dot[0]       = qp[0] - qp_old[0];
        qp_dot[1]       = qp[1] - qp_old[1];
        qp_dot[2]       = qp[2] - qp_old[2];

        qp_old          = qp;

        // hal.console->printf("%3.3f,",  qp[0]);
        // hal.console->printf("%3.3f,",  qp[1]);
        // hal.console->printf("%3.3f\n", qp[2]);

        // hal.console->printf("%3.3f,",  R_payload[0][0]);
        // hal.console->printf("%3.3f,",  R_payload[1][0]);
        // hal.console->printf("%3.3f,",  R_payload[2][0]);

        // hal.console->printf("%3.3f,",  R_payload[0][1]);
        // hal.console->printf("%3.3f,",  R_payload[1][1]);
        // hal.console->printf("%3.3f,",  R_payload[2][1]);

        // hal.console->printf("%3.3f,",  R_payload[0][2]);
        // hal.console->printf("%3.3f,",  R_payload[1][2]);
        // hal.console->printf("%3.3f\n", R_payload[2][2]);

}

void Copter::get_HHD_Acceleration_data()

{
    bool receiving_data = false;
    int index           = 0;
    char startChar      = ',';
    char endChar        = '/';
    bool new_data       = false;

    // hal.console->printf("serial data -> %c\n",hal.serial(HHD_Acceleration_port)->read());

    while (hal.serial(HHD_Acceleration_port)->available()>0 && new_data == false)
        {
            char temp = hal.serial(HHD_Acceleration_port)->read();
            // hal.console->printf("serial data -> %c\n",temp);
            if (receiving_data == true)
            {
                if (temp != endChar)
                {
                    HHD_Acceleration_data[index] = temp;
                    index++;
                }
                else
                {
                    // hal.console->printf("Index number -> %d\n",index);
                    HHD_Acceleration_data[index] = '\0';
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
        }

        // hal.console->printf("attitude -> %s\n",HHD_Attitude_data);

        for (int i = 0; i < 7; i++)
        {
                HHD_local_acc_X_char[i]     = HHD_Acceleration_data[i];
                HHD_local_acc_Y_char[i]     = HHD_Acceleration_data[i+7];
                HHD_local_acc_Z_char[i]     = HHD_Acceleration_data[i+14];
        }

        int HHD_local_acc_X_int     = atoi(HHD_local_acc_X_char);
        int HHD_local_acc_Y_int     = atoi(HHD_local_acc_Y_char);
        int HHD_local_acc_Z_int     = atoi(HHD_local_acc_Z_char);

        HHD_local_acc_X  =  (float)((HHD_local_acc_X_int  - 500000.0) / 100.0);
        HHD_local_acc_Y  =  (float)((HHD_local_acc_Y_int  - 500000.0) / 100.0);
        HHD_local_acc_Z  =  (float)((HHD_local_acc_Z_int  - 500000.0) / 100.0);

        float max_magnitude_to_acceleration = 20.0;     // m/s^2

        if (HHD_local_acc_X > max_magnitude_to_acceleration){
            HHD_local_acc_X = max_magnitude_to_acceleration;
        }
        if (HHD_local_acc_X < -max_magnitude_to_acceleration){
            HHD_local_acc_X = -max_magnitude_to_acceleration;
        }

        if (HHD_local_acc_Y > max_magnitude_to_acceleration){
            HHD_local_acc_Y = max_magnitude_to_acceleration;
        }
        if (HHD_local_acc_Y < -max_magnitude_to_acceleration){
            HHD_local_acc_Y = -max_magnitude_to_acceleration;
        }

        if (HHD_local_acc_Z > max_magnitude_to_acceleration){
            HHD_local_acc_Z = max_magnitude_to_acceleration;
        }
        if (HHD_local_acc_Z < -max_magnitude_to_acceleration){
            HHD_local_acc_Z = -max_magnitude_to_acceleration;
        }

        HHD_local_acceleration[0] = HHD_local_acc_X;
        HHD_local_acceleration[1] = HHD_local_acc_Y;
        HHD_local_acceleration[2] = HHD_local_acc_Z;

        // hal.console->printf("HHD_Acce->");
        // hal.console->printf("%3.3f,",   HHD_local_acc_X);
        // hal.console->printf("%3.3f,",   HHD_local_acc_Y);
        // hal.console->printf("%3.3f\n",  HHD_local_acc_Z);
        // hal.console->printf(" | ");

}

float Copter::limit_on_forces_from_quad1(float u)
{

    if (u < 0.0){ u = 0.0;}
    if (u > 15.0){ u = 15.0;}

    return u;
}

float Copter::limit_on_yawrate_for_qpd_from_quad1(float u)
{
    float max_value = 20.25;
    if (u < -max_value){ u = -max_value;}
    if (u > max_value){ u =  max_value;}

    return u;
}

void Copter::get_CAM_device_Data()
{
    bool receiving_data = false;
    int index           = 0;
    char startChar      = ',';
    char endChar        = '/';
    bool new_data       = false;

    while (hal.serial(CAM_device_port)->available()>0 && new_data == false)
        {
            char temp = hal.serial(CAM_device_port)->read();
            // hal.console->printf("serial data -> %c\n",temp);
            if (receiving_data == true)
            {
                if (temp != endChar)
                {
                    cable_attitude[index] = temp;
                    index++;
                }
                else
                {
                    // hal.console->printf("Index number -> %d\n",index);
                    cable_attitude[index] = '\0';
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
            // hal.console->printf("attitude from while loop-> %s\n",cable_attitude);
        }

        // hal.console->printf("attitude -> %s\n",cable_attitude);
        // hal.console->printf("Hi Pratik from Ardupilot \n");

        for (int i = 0; i < 11; i++)
        {
            if (i < 5)  
            {
                CAM_roll_char[i]                = cable_attitude[i];
            } else if (i >= 6 && i < 11 )
            {
                CAM_pitch_char[i - 6]           = cable_attitude[i];
            }
        }

        int encoder_roll_int      = atoi(CAM_roll_char);
        int encoder_pitch_int     = atoi(CAM_pitch_char);

        CAM_roll  =  (float)((encoder_roll_int  - 50000.0) / 100.0);
        CAM_pitch =  (float)((encoder_pitch_int - 50000.0) / 100.0); 

        hal.console->printf("%3.3f,", CAM_roll);
        hal.console->printf("%3.3f\n", CAM_pitch);

        if (CAM_roll > 60.0){
            CAM_roll = 60.0;
        }
        if (CAM_roll < -60.0){
            CAM_roll = -60.0;
        }

        if (CAM_pitch > 60.0){
            CAM_pitch = 60.0;
        }
        if (CAM_pitch < -60.0){
            CAM_pitch = -60.0;
        }

        // hal.console->printf("ENCO-> [%3.3f,%3.3f] | ",CAM_roll,CAM_pitch);

        // hal.console->printf("CAM_device->");
        // hal.console->printf("%3.3f,", CAM_roll);
        // hal.console->printf("%3.3f\n", CAM_pitch);

        // hal.serial(2)->printf("%3.3f,", CAM_roll);
        // hal.serial(2)->printf("%3.3f\n", CAM_pitch);

        // hal.console->printf("CAM_device_data -> %f,%f\n",CAM_roll,CAM_pitch);

        Vector3f rpy(quad_roll*PI/180.0,quad_pitch*PI/180.0,quad_yaw*PI/180.0);
        Vector3f e_3_neg(0,0,-1);
        Matrix3f R(eulerAnglesToRotationMatrix(rpy));

        // Calculate rotation about pitch axis of CAM device
        Matrix3f CAM_R_y (
                cosf(CAM_pitch*PI/180),    0,      sinf(CAM_pitch*PI/180),
                0,               1,      0,
                -sinf(CAM_pitch*PI/180),   0,      cosf(CAM_pitch*PI/180)
                );

        // Calculate rotation about roll axis of CAM device
        Matrix3f CAM_R_x (
                1,       0,              0,
                0,       cosf(CAM_roll*PI/180),   -sinf(CAM_roll*PI/180),
                0,       sinf(CAM_roll*PI/180),   cosf(CAM_roll*PI/180)
                );

        qc_2 = Matrix_vector_mul(R,Matrix_vector_mul(CAM_R_x,Matrix_vector_mul(CAM_R_y,e_3_neg)));
        qc_2 = sat_q(qc_2);

        // hal.console->printf("%3.3f,%3.3f,%3.3f\n", qc_2[0], qc_2[1], qc_2[2]);

        qc_2_dot[0]     = qc_2[0] - qc_2_old[0];
        qc_2_dot[1]     = qc_2[1] - qc_2_old[1];
        qc_2_dot[2]     = qc_2[2] - qc_2_old[2];

        qc_2_old        = qc_2;

}

void Copter::get_HHD_Encoders_Data()
{
    bool receiving_data = false;
    int index           = 0;
    char startChar      = ',';
    char endChar        = '/';
    bool new_data       = false;

    // hal.console->printf("serial data -> %c\n",hal.serial(3)->read());

    while (hal.serial(HHD_Encoders_port)->available()>0 && new_data == false)
        {
            char temp = hal.serial(HHD_Encoders_port)->read();
            // hal.console->printf("serial data -> %c\n",temp);
            if (receiving_data == true)
            {
                if (temp != endChar)
                {
                    HHD_Encoder_data[index] = temp;
                    index++;
                }
                else
                {
                    // hal.console->printf("Index number -> %d\n",index);
                    HHD_Encoder_data[index] = '\0';
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
            // hal.console->printf("attitude from while loop-> %s\n",HHD_Encoder_data);
        }

        // hal.console->printf("HHD_Encoders_data -> %s\n",HHD_Encoder_data);
        // hal.console->printf("Hi Pratik from Ardupilot \n");

        for (int i = 0; i < 13; i++)
        {
            if (i < 5)  
            {
                HHD_first_encoder_char[i]               = HHD_Encoder_data[i];
            } else if (i >= 6 && i < 11 )
            {
                HHD_second_encoder_char[i - 6]          = HHD_Encoder_data[i];
            } else if (i==12)
            {
                HHD_safety_switch_char[i-12]            = HHD_Encoder_data[i];
            }
        }

        int HHD_first_encoder_int      = atoi(HHD_first_encoder_char);
        int HHD_second_encoder_int     = atoi(HHD_second_encoder_char);
        int HHD_HHD_safety_switch_int     = atoi(HHD_safety_switch_char);

        HHD_encoder_1           = (float)((HHD_first_encoder_int  - 50000.0) / 100.0);
        HHD_encoder_2           = (float)((HHD_second_encoder_int - 50000.0) / 100.0); 
        HHD_safety_swith_state  = (float)(HHD_HHD_safety_switch_int);

        // hal.console->printf("HHD_encoders -> ");
        // hal.console->printf("%3.3f,", HHD_encoder_1);
        // hal.console->printf("%3.3f,", HHD_encoder_2);
        // hal.console->printf("%3.3f\n", HHD_safety_swith_state);
        // hal.console->printf(" | ");

        // if (CAM_roll > 60.0){
        //     CAM_roll = 60.0;
        // }
        // if (CAM_roll < -60.0){
        //     CAM_roll = -60.0;
        // }

        // if (CAM_pitch > 60.0){
        //     CAM_pitch = 60.0;
        // }
        // if (CAM_pitch < -60.0){
        //     CAM_pitch = -60.0;
        // }

        // hal.console->printf("ENCO-> [%3.3f,%3.3f] | ",CAM_roll,CAM_pitch);

        // hal.console->printf("%3.3f,", CAM_roll);
        // hal.console->printf("%3.3f\n", CAM_pitch);

        // hal.serial(2)->printf("%3.3f,", CAM_roll);
        // hal.serial(2)->printf("%3.3f\n", CAM_pitch);

        // hal.console->printf("CAM_device_data -> %f,%f\n",CAM_roll,CAM_pitch);

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
    Matrix3f R = R_z * R_x * R_y;
    return R;
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

float Copter::dot_product(Vector3f v1, Vector3f v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

Matrix3f Copter::Matrix_to_matrix_multiplication(Matrix3f M1, Matrix3f M2)
{
    Matrix3f M;

    Vector3f M1_r1(M1[0][0], M1[0][1], M1[0][2]);
    Vector3f M1_r2(M1[1][0], M1[1][1], M1[1][2]);
    Vector3f M1_r3(M1[2][0], M1[2][1], M1[2][2]);

    Vector3f M2_c1(M2[0][0], M2[1][0], M2[2][0]);
    Vector3f M2_c2(M2[0][1], M2[1][1], M2[2][1]);
    Vector3f M2_c3(M2[0][2], M2[1][2], M2[2][2]);

    M[0][0]     = dot_product(M1_r1, M2_c1);
    M[0][1]     = dot_product(M1_r1, M2_c2);
    M[0][2]     = dot_product(M1_r1, M2_c3);
    
    M[1][0]     = dot_product(M1_r2, M2_c1);
    M[1][1]     = dot_product(M1_r2, M2_c2);
    M[1][2]     = dot_product(M1_r2, M2_c3);

    M[2][0]     = dot_product(M1_r3, M2_c1);
    M[2][1]     = dot_product(M1_r3, M2_c2);
    M[2][2]     = dot_product(M1_r3, M2_c3);

    return M;

}

void Copter::Log_quad_pos_data_follower()
{
    struct log_quad_pos_ pkt = {
    LOG_PACKET_HEADER_INIT(LOG_QUAD_POS_MSG),
    time_us  : AP_HAL::micros64(),
    x2_TRO        : quad2_pos[0],
    y2_TRO        : quad2_pos[1],
    z2_TRO        : quad2_pos[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_quad_vel_data_follower()
{
    struct log_quad_vel_ pkt = {
    LOG_PACKET_HEADER_INIT(LOG_QUAD_VEL_MSG),
    time_us  : AP_HAL::micros64(),
    x2dot    : quad2_vel[0],
    y2dot    : quad2_vel[1],
    z2dot    : quad2_vel[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_quad_RPY_data_follower()
{
    struct log_quad_RPY_ pkt = {
    LOG_PACKET_HEADER_INIT(LOG_QUAD_RPY_MSG),
    time_us  : AP_HAL::micros64(),
    ph2      : quad_roll,
    th2      : quad_pitch,
    psi2     : quad_yaw,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_quad_angular_velocity_data_follower()
{
    struct log_quad_ANG_VEL_ pkt = {
    LOG_PACKET_HEADER_INIT(LOG_QUAD_AVG_MSG),
    time_us  : AP_HAL::micros64(),
    ph2dot   : quad_roll_dot,
    th2dot   : quad_pitch_dot,
    psi2dot   : quad_yaw_dot,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_cable_2_attitude_data_follower()
{
    struct log_Cab2_ATT_ pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CABL_ATT_MSG),
    time_us  : AP_HAL::micros64(),
    qc2_1_log  : qc_2[0],
    qc2_2_log  : qc_2[1],
    qc2_3_log  : qc_2[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_cable_2_attitude_dot_data_follower()
{
    struct log_Cab2_ATT_dot_ pkt = {
    LOG_PACKET_HEADER_INIT(LOG_CABL_DOT_MSG),
    time_us  : AP_HAL::micros64(),
    qc2_1dot_log : qc_2_dot[0],
    qc2_2dot_log : qc_2_dot[1],
    qc2_3dot_log : qc_2_dot[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}


void Copter::Log_payload_attitude_data_follower()
{
    struct log_Pay_ATT_CMD_ pkt = {
    LOG_PACKET_HEADER_INIT(LOG_PYLD_ATT_MSG),
    time_us  : AP_HAL::micros64(),
    qp1_log  : qp[0],
    qp2_log  : qp[1],
    qp3_log  : qp[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_payload_attitude_dot_data_follower()
{
    struct log_Pay_ATT_dot_ pkt = {
    LOG_PACKET_HEADER_INIT(LOG_PYLD_DOT_MSG),
    time_us  : AP_HAL::micros64(),
    qp1dot_log : qp_dot[0],
    qp2dot_log : qp_dot[1],
    qp3dot_log : qp_dot[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_u2_PAC_follower()
{
    struct log_u2_PAC pkt = {
    LOG_PACKET_HEADER_INIT(LOG_U2_PAC_MSG),
    time_us  : AP_HAL::micros64(),
    u2_PAC_1_log : u2_PAC[0],
    u2_PAC_2_log : u2_PAC[1],
    u2_PAC_3_log : u2_PAC[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_u2_CAC2_follower()
{
    struct log_u2_CAC2 pkt = {
    LOG_PACKET_HEADER_INIT(LOG_U2_CAC2_MSG),
    time_us  : AP_HAL::micros64(),
    u2_CAC2_1_log : u2_CAC2[0],
    u2_CAC2_2_log : u2_CAC2[1],
    u2_CAC2_3_log : u2_CAC2[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_u2_follower()
{
    struct log_u2_ pkt = {
    LOG_PACKET_HEADER_INIT(LOG_U2_MSG),
    time_us  : AP_HAL::micros64(),
    u2_1_log : u2[0],
    u2_2_log : u2[1],
    u2_3_log : u2[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_qp_des_from_HHD()
{
    struct log_qp_des_ pkt = {
    LOG_PACKET_HEADER_INIT(LOG_QP_DES_MSG),
    time_us  : AP_HAL::micros64(),
    qpdes_1_log : qp_des[0],
    qpdes_2_log : qp_des[1],
    qpdes_3_log : qp_des[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_HHD_attitude()
{
    struct log_hhd_att pkt = {
    LOG_PACKET_HEADER_INIT(LOG_HHD_ATT_MSG),
    time_us  : AP_HAL::micros64(),
    hhd_att_1_log : HHD_roll,
    hhd_att_2_log : HHD_pitch,
    hhd_att_3_log : HHD_yaw,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_HHD_acceleration()
{
    struct log_hhd_acc pkt = {
    LOG_PACKET_HEADER_INIT(LOG_HHD_ACC_MSG),
    time_us  : AP_HAL::micros64(),
    hhd_acc_1_log : HHD_local_acceleration[0],
    hhd_acc_2_log : HHD_local_acceleration[1],
    hhd_acc_3_log : HHD_local_acceleration[2],
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_HHD_encoders()
{
    struct log_HHD_ENCO pkt = {
    LOG_PACKET_HEADER_INIT(LOG_HHD_ENCO_MSG),
    time_us  : AP_HAL::micros64(),
    hhd_enco_1_log : HHD_encoder_1,
    hhd_enco_2_log : HHD_encoder_2,
    hhd_switch_log : HHD_safety_swith_state,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}


void Copter::Log_Exp_start_stop()
{
    struct log_exp_st_sp pkt = {
    LOG_PACKET_HEADER_INIT(LOG_EXP_STP_MSG),
    time_us  : AP_HAL::micros64(),
    exp_stp_st : flag_start_stop_int,
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}