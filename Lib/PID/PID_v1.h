#ifndef PID_V1_h
#define PID_V1_h


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


#define DIRECT  0
#define REVERSE  1
#define P_ON_E 0
#define P_ON_M 1


typedef struct{
  float Kp,Ki,Kd;//三个系数
  float DispKp,DispKi,DispKd;
  int ControllerDirection;
  int pOn;
  int FirstCompute;

  float Integral;
  float LastInput;

  float MinIntegral,MaxIntegral;//积分、积分限幅
  float OutMin,OutMax;//输出、输出限幅
}PID_Parameter;


//commonly used functions **************************************************************************
void PID_Init(PID_Parameter * PID); 

void PID_Compute(PID_Parameter * PID,  float Feedback ,float Reference ,uint16_t *PID_OutPut );

void SetTunings(PID_Parameter * PID,float KP, float KI, float KD);

void PID_Clear_Integral(PID_Parameter * PID);

void Loop_Competition_Buck(uint16_t Current_Loop,uint16_t Voltage_Loop,uint16_t *ProtectCompare);

void Loop_Competition_Boost(uint16_t Current_Loop,uint16_t Voltage_Loop,uint16_t *ProtectCompare);

#ifdef __cplusplus
}
#endif

#endif

