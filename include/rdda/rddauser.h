#ifndef RDDA_RDDAUSER_H
#define RDDA_RDDAUSER_H

/** EtherCAT slave identifier */
typedef struct PACKED
{
    uint16 motor1;
    uint16 motor2;
    uint16 psensor;
}rdda_slavet;

//extern rdda_slavet *rdda_slave;

/** BEL drive CSP Mode inputs to master */
PACKED_BEGIN
typedef struct PACKED
{
    uint16 stat_wd;   /* status word (0x6041) */
    int32 act_pos;    /* position actual value (0x6064) */
    int32 pos_err;    /* position error (0x60F4) */
    int32 act_vel;    /* actual velocity (0x606C) */
    int16 act_tau;    /* torque actual value (0x6077) */
    int32 load_vel;   /* load encoder velocity (0x2231) */
    int32 load_pos;   /* load encoder position (0x2242) */
} in_motor_t;
PACKED_END

/** BEL drive CSP Mode outputs from master */
PACKED_BEGIN
typedef struct PACKED
{
    uint16 ctrl_wd;   /* control word (0x6040) */
    int32 tg_pos;     /* target position (0x607A) */
    int32 vel_off;    /* velocity offset (0x60B1) */
    int16 tau_off;    /* torque offset (0x60B2) */
} out_motor_t;
PACKED_END

/** EL3102 pressure sensor inputs to master */
PACKED_BEGIN
typedef struct PACKED
{
    uint8 stat1;
    int16 val1;
    uint8 stat2;
    int16 val2;
} in_pressure_t;
PACKED_END

double readMotor1Pos();
double readMotor2Pos();
double readMotor1Vel();
double readMotor2Vel();
double readLoad2Pos();
double readLoad2Vel();
double readPre1Val();
double readPre2Val();
void writeMotor1CtrlWd(uint16 value);
void writeMotor2CtrlWd(uint16 value);
void writeMotor1Pos(double value);
void writeMotor2Pos(double value);
void writeMotor1Tau(double value);
void writeMotor2Tau(double value);


#endif //RDDA_RDDAUSER_H
