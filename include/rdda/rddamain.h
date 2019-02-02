#ifndef RDDA_RDDAMAIN_H
#define RDDA_RDDAMAIN_H


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


#endif //RDDA_RDDAMAIN_H