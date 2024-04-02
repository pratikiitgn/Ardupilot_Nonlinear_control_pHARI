#include "Copter.h"
#include <AP_HAL/HAL.h>
#include "mycontroller_usercode.h"

#define PI 3.14159265359

float CAM_roll     = 0.0;
float CAM_pitch    = 0.0;

float PAMD_roll     = 0.0;
float PAMD_pitch    = 0.0;
float PAMD_yaw      = 0.0;

char cable_attitude[]       = "50000_50000";
char payload_attitude[]     = "500000_500000&500000";
char Quad1POS_CAM1_PAC[]    = "5000_5000_5000_5000_5000_5000_5000_5000_5000_5000_5000_5000";

char CAM_roll_char[]        = "50000";
char CAM_pitch_char[]       = "50000";

char PAMD_roll_char[]       = "500000";
char PAMD_pitch_char[]      = "500000";
char PAMD_yaw_char[]        = "500000";

float u1_POS_1          = 0.0;
float u1_POS_2          = 0.0;
float u1_POS_3          = 0.0;

float u1_CAC_1          = 0.0;
float u1_CAC_2          = 0.0;
float u1_CAC_3          = 0.0;

float u1_PAC_1          = 0.0;
float u1_PAC_2          = 0.0;
float u1_PAC_3          = 0.0;

char u1_POS_1_char[]    = "5000";
char u1_POS_2_char[]    = "5000";
char u1_POS_3_char[]    = "5000";

char u1_CAC_1_char[]    = "5000";
char u1_CAC_2_char[]    = "5000";
char u1_CAC_3_char[]    = "5000";

char u1_PAC_1_char[]    = "5000";
char u1_PAC_2_char[]    = "5000";
char u1_PAC_3_char[]    = "5000";

char qpd_1_char[]    = "5000";
char qpd_2_char[]    = "5000";
char qpd_3_char[]    = "5000";

int u1_POS_1_int     = 0.0;
int u1_POS_2_int     = 0.0;
int u1_POS_3_int     = 0.0;

int u1_CAC_1_int     = 0.0;
int u1_CAC_2_int     = 0.0;
int u1_CAC_3_int     = 0.0;

int u1_PAC_1_int     = 0.0;
int u1_PAC_2_int     = 0.0;
int u1_PAC_3_int     = 0.0;

int qpd_1_int        = 0.0;
int qpd_2_int        = 0.0;
int qpd_3_int        = 0.0;

Vector3f qc_2(0.0,0.0,-1.0);
Vector3f qc_2_dot(0.0,0.0,0.0);
Vector3f qc_2_old(0.0,0.0,0.0);

Vector3f qp(1.0,0.0,0.0);
Vector3f qp_dot(0.0,0.0,0.0);
Vector3f qp_old(0.0,0.0,0.0);

int QuadCam1qpd_port    = 1;
int PAMD_device_port    = 2;
int CAM_device_port     = 4;

Vector3f qp_des_from_quad_1(0.0,0.0,0.0);

//           parameter prefix 
// UART4  -  SERIAL3_           - GPS      - GPS1
// UART8  -  SERIAL4_           - SERIAL4  - GPS2

// USART2 -  SERIAL1_             TELEM1     TELEM1
// USART3 -  SERIAL2_             TELEM2     TELEM2

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    hal.serial(QuadCam1qpd_port)->begin(230400);        // telemetry 1  port Pixhawk Cube Orange - 
    hal.serial(PAMD_device_port)->begin(230400);        // telemetry 2  port Pixhawk Cube Orange - PAMD device
    hal.serial(CAM_device_port)->begin(230400);         // GPS 2        port Pixhawk Cube Orange - CAM  device
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here

    // hal.console->printf("Roll -> %f\n",quad_roll);
    get_CAM_device_Data();
    get_PAMD_device_Data();
    get_Quad1_CAM1_qpd_Data();

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

void Copter::get_PAMD_device_Data()

{
    bool receiving_data = false;
    int index           = 0;
    char startChar      = ',';
    char endChar        = '/';
    bool new_data       = false;

    // hal.console->printf("serial data -> %c\n",hal.serial(PAMD_device_port)->read());

    while (hal.serial(PAMD_device_port)->available()>0 && new_data == false)
        {
            char temp = hal.serial(PAMD_device_port)->read();
            // hal.console->printf("serial data -> %c\n",temp);
            if (receiving_data == true)
            {
                if (temp != endChar)
                {
                    payload_attitude[index] = temp;
                    index++;
                }
                else
                {
                    // hal.console->printf("Index number -> %d\n",index);
                    payload_attitude[index] = '\0';
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

        // hal.console->printf("attitude -> %s\n",payload_attitude);

        for (int i = 0; i < 7; i++)
        {
                PAMD_roll_char[i]       = payload_attitude[i];
                PAMD_pitch_char[i]      = payload_attitude[i+7];
                PAMD_yaw_char[i]        = payload_attitude[i+14];
        }

        int PAMD_roll_int       = atoi(PAMD_roll_char);
        int PAMD_pitch_int      = atoi(PAMD_pitch_char);
        int PAMD_yaw_int        = atoi(PAMD_yaw_char);

        PAMD_roll  =  (float)((PAMD_roll_int  - 500000.0) / 100.0);
        PAMD_pitch =  (float)((PAMD_pitch_int - 500000.0) / 100.0);
        float PAMD_yaw_raw   =  (float)((PAMD_yaw_int - 500000.0) / 100.0);

        if (PAMD_yaw_raw > 0.0){
            PAMD_yaw = PAMD_yaw_raw;
        }else{
            PAMD_yaw = 360.0 + PAMD_yaw_raw;
        }

        PAMD_yaw = PAMD_yaw - 90.0;

        if (PAMD_yaw < 0.0){
            PAMD_yaw = 360.0 + PAMD_yaw;
        }

        // hal.console->printf("%3.2f, %3.2f \n", PAMD_yaw, quad_yaw);

        if (PAMD_roll > 90.0){
            PAMD_roll = 90.0;
        }
        if (PAMD_roll < -90.0){
            PAMD_roll = -90.0;
        }

        if (PAMD_pitch > 90.0){
            PAMD_pitch = 90.0;
        }
        if (PAMD_pitch < -90.0){
            PAMD_pitch = -90.0;
        }

        // if (PAMD_yaw > 180.0){
        //     PAMD_yaw = 180.0;
        // }
        // if (PAMD_yaw < -180.0){
        //     PAMD_yaw = -180.0;
        // }

        // hal.console->printf("%3.3f,", PAMD_roll);
        // hal.console->printf("%3.3f,", PAMD_pitch);
        // hal.console->printf("%3.3f\n", PAMD_yaw);

        Vector3f rpy_vector(PAMD_roll*PI/180.0,PAMD_pitch*PI/180.0,PAMD_yaw*PI/180.0);
        Matrix3f R_payload(eulerAnglesToRotationMatrix(rpy_vector));
        Vector3f e_1(1.0,0,0.0);
        qp = Matrix_vector_mul(R_payload,e_1);

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

float Copter::limit_on_forces_from_quad1(float u)
{

    if (u < 0.0){ u = 0.0;}
    if (u > 15.0){ u = 15.0;}

    return u;
}

void Copter::get_Quad1_CAM1_qpd_Data()
{
    bool receiving_data = false;
    int index           = 0;
    char startChar      = ',';
    char endChar        = '/';
    bool new_data       = false;

    // char temp1 = hal.serial(QuadCam1qpd_port)->read();
    // hal.console->printf("%c\n", temp1);

    while (hal.serial(QuadCam1qpd_port)->available()>0 && new_data == false)
        {
            char temp = hal.serial(QuadCam1qpd_port)->read();
            // hal.console->printf("%c\n", temp);

            if (receiving_data == true)
            {
                if (temp != endChar)
                {
                    Quad1POS_CAM1_PAC[index] = temp;
                    index++;
                }
                else
                {
                    Quad1POS_CAM1_PAC[index] = '\0';
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

        // hal.console->printf("%s\n\n", Quad1POS_CAM1_PAC);

        for (int i = 0; i < 4; i++)
        {
            u1_POS_1_char[i]    = Quad1POS_CAM1_PAC[i];
            u1_POS_2_char[i]    = Quad1POS_CAM1_PAC[i+5];
            u1_POS_3_char[i]    = Quad1POS_CAM1_PAC[i+10];

            u1_CAC_1_char[i]    = Quad1POS_CAM1_PAC[i+15];
            u1_CAC_2_char[i]    = Quad1POS_CAM1_PAC[i+20];
            u1_CAC_3_char[i]    = Quad1POS_CAM1_PAC[i+25];

            u1_PAC_1_char[i]    = Quad1POS_CAM1_PAC[i+30];
            u1_PAC_2_char[i]    = Quad1POS_CAM1_PAC[i+35];
            u1_PAC_3_char[i]    = Quad1POS_CAM1_PAC[i+40];

            qpd_1_char[i]       = Quad1POS_CAM1_PAC[i+45];
            qpd_2_char[i]       = Quad1POS_CAM1_PAC[i+50];
            qpd_3_char[i]       = Quad1POS_CAM1_PAC[i+55];
        }

        // hal.console->printf("%s,%s,%s",   u1_POS_1_char, u1_POS_2_char, u1_POS_3_char);
        // hal.console->printf("%s,%s,%s",   u1_CAC_1_char, u1_CAC_2_char, u1_CAC_3_char);
        // hal.console->printf("%s,%s,%s\n", u1_PAC_1_char, u1_PAC_2_char, u1_PAC_3_char);

        u1_POS_1_int    = atoi(u1_POS_1_char);
        u1_POS_2_int    = atoi(u1_POS_2_char);
        u1_POS_3_int    = atoi(u1_POS_3_char);

        u1_CAC_1_int    = atoi(u1_CAC_1_char);
        u1_CAC_2_int    = atoi(u1_CAC_2_char);
        u1_CAC_3_int    = atoi(u1_CAC_3_char);

        u1_PAC_1_int    = atoi(u1_PAC_1_char);
        u1_PAC_2_int    = atoi(u1_PAC_2_char);
        u1_PAC_3_int    = atoi(u1_PAC_3_char);

        qpd_1_int       = atoi(qpd_1_char);
        qpd_2_int       = atoi(qpd_2_char);
        qpd_3_int       = atoi(qpd_3_char);

        // hal.console->printf("%d,%d,%d,",   u1_POS_1_int, u1_POS_2_int, u1_POS_3_int);
        // hal.console->printf("%d,%d,%d,",   u1_CAC_1_int, u1_CAC_2_int, u1_CAC_3_int);
        // hal.console->printf("%d,%d,%d,", u1_PAC_1_int, u1_PAC_2_int, u1_PAC_3_int);

        u1_POS_1          = (float)((u1_POS_1_int - 5000.0) / 100.0);
        u1_POS_2          = (float)((u1_POS_2_int - 5000.0) / 100.0);
        u1_POS_3          = (float)((u1_POS_3_int - 5000.0) / 100.0);

        u1_CAC_1          = (float)((u1_CAC_1_int - 5000.0) / 100.0);
        u1_CAC_2          = (float)((u1_CAC_2_int - 5000.0) / 100.0);
        u1_CAC_3          = (float)((u1_CAC_3_int - 5000.0) / 100.0);

        u1_PAC_1          = (float)((u1_PAC_1_int - 5000.0) / 100.0);
        u1_PAC_2          = (float)((u1_PAC_2_int - 5000.0) / 100.0);
        u1_PAC_3          = (float)((u1_PAC_3_int - 5000.0) / 100.0);

        qp_des_from_quad_1[0]          = (float)((qpd_1_int - 5000.0) / 1000.0);
        qp_des_from_quad_1[1]          = (float)((qpd_2_int - 5000.0) / 1000.0);
        qp_des_from_quad_1[2]          = (float)((qpd_3_int - 5000.0) / 1000.0);

        hal.console->printf("%2.2f,%2.2f,%2.2f,",   u1_POS_1, u1_POS_2, u1_POS_3);
        hal.console->printf("%2.2f,%2.2f,%2.2f,",   u1_CAC_1, u1_CAC_2, u1_CAC_3);
        hal.console->printf("%2.2f,%2.2f,%2.2f,", u1_PAC_1, u1_PAC_2, u1_PAC_3);
        hal.console->printf("%2.2f,%2.2f,%2.2f\n", qp_des_from_quad_1[0], qp_des_from_quad_1[1], qp_des_from_quad_1[2]);

        u1_POS_1 = limit_on_forces_from_quad1(u1_POS_1);
        u1_POS_2 = limit_on_forces_from_quad1(u1_POS_2);
        u1_POS_3 = limit_on_forces_from_quad1(u1_POS_3);

        u1_CAC_1 = limit_on_forces_from_quad1(u1_CAC_1);
        u1_CAC_2 = limit_on_forces_from_quad1(u1_CAC_2);
        u1_CAC_3 = limit_on_forces_from_quad1(u1_CAC_3);

        u1_PAC_1 = limit_on_forces_from_quad1(u1_PAC_1);
        u1_PAC_2 = limit_on_forces_from_quad1(u1_PAC_2);
        u1_PAC_3 = limit_on_forces_from_quad1(u1_PAC_3);

        // hal.console->printf("%3.2f, %3.2f, %3.2f,  ", u1_POS_1, u1_POS_2, u1_POS_3);
        // hal.console->printf("%3.2f, %3.2f, %3.2f,  ", u1_CAC_1, u1_CAC_2, u1_CAC_3);
        // hal.console->printf("%3.2f, %3.2f, %3.2f \n", u1_PAC_1, u1_PAC_2, u1_PAC_3);

}

void Copter::get_CAM_device_Data()
{
    bool receiving_data = false;
    int index           = 0;
    char startChar      = ',';
    char endChar        = '/';
    bool new_data       = false;

    // hal.console->printf("serial data -> %c\n",hal.serial(3)->read());

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

        // hal.console->printf("%3.3f,", CAM_roll);
        // hal.console->printf("%3.3f\n", CAM_pitch);

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