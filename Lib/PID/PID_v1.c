/**********************************************************************************************
 * Arduino PID Library - Version 1->2->1
 * by Brett Beauregard <br3ttb@gmail->com> brettbeauregard->com
 *
 * This Library is licensed under the MIT License
 **********************************************************************************************/

#include "PID_v1.h"

/*Constructor (->->->)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them->
 ***************************************************************************/
void PID_Init(PID_Parameter * PID)
{
   PID->DispKp = PID->Kp;
   PID->DispKi = PID->Ki;
   PID->DispKd = PID->Kd;

   // PID->MaxIntegral = PID->OutMax / PID->Ki;
   // PID->MinIntegral = PID->OutMin / PID->Ki;

   PID->FirstCompute =1;

   if(PID->pOn != P_ON_E)PID->pOn = P_ON_M;

   if(PID->ControllerDirection != DIRECT){
      PID->Kp = (0 - PID->Kp);
      PID->Ki = (0 - PID->Ki);
      PID->Kd = (0 - PID->Kd);
      PID->ControllerDirection = REVERSE;
   }
}



/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens->  this function should be called
 *   every time "void loop()" executes->  the function will decide for itself whether a new
 *   pid Output needs to be computed->  returns true when the OutPut is computed,
 *   false when nothing has been done->
 **********************************************************************************/
//PID为PID_Parameter结构体的PID参数
//Feedback为反馈值，也就是当前系统的输出值
//Reference为参考值，也就是系统期望的输出值
//PID_OutPut为PID运算的输出结果，用于调整系统输出
void PID_Compute(PID_Parameter * PID,  float Feedback ,float Reference ,uint16_t *PID_OutPut ){
   /*Compute all the working error variables*/
   if(PID->FirstCompute){
      PID->Integral = Feedback;
      PID->LastInput = Reference;
      if(PID->Integral > PID->OutMax) PID->Integral = PID->OutMax;
      else if(PID->Integral < PID->OutMin) PID->Integral = PID->OutMin;
      PID->FirstCompute =0;
   }

   float error = 0;
   float dInput = 0;
   
   error = Reference - Feedback;
   dInput = Feedback - PID->LastInput;
   PID->Integral = PID->Integral + PID->Ki * error;

   /*Add Proportional on Measurement, if P_ON_M is specified*/
   if(PID->pOn) PID->Integral = PID->Integral - PID->Kp * dInput;

   if(PID->Integral > PID->OutMax) PID->Integral= PID->OutMax;
   else if(PID->Integral < PID->OutMin) PID->Integral= PID->OutMin;

   /*Add Proportional on Error, if P_ON_E is specified*/
   float OutPut;
   if(!PID->pOn) OutPut = PID->Kp * error;
   else OutPut = 0;

   /*Compute Rest of PID Output*/
   OutPut += PID->Integral;
   // OutPut += (PID->Integral - PID->Kd * dInput);

   if(OutPut > PID->OutMax) OutPut = PID->OutMax;
   else if(OutPut < PID->OutMin) OutPut = PID->OutMin;

   *PID_OutPut =(uint16_t)OutPut;

   /*Remember some variables for next time*/
   PID->LastInput = Feedback;
}

/* SetTunings()*************************************************************
 * This function allows the controller's dynamic performance to be adjusted->
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void SetTunings(PID_Parameter * PID,float KP, float KI, float KD)
{
   if (KP<0 || KI<0 || KD<0) return;

   PID->DispKp = KP;
   PID->DispKi = KI;
   PID->DispKd = KD;

  if(PID->ControllerDirection == REVERSE)
   {
      PID->Kp = (0 - KP);
      PID->Ki = (0 - KI);
      PID->Kd = (0 - KD);
   }
}

void PID_Clear_Integral(PID_Parameter * PID){
   PID->FirstCompute =1;
   PID->LastInput =0;
   PID->Integral =0;
}


void Loop_Competition_Buck(uint16_t Current_Loop,uint16_t Voltage_Loop,uint16_t *ProtectCompare){
  if(Current_Loop < Voltage_Loop)*ProtectCompare = Current_Loop;
  else *ProtectCompare = Voltage_Loop;
}


void Loop_Competition_Boost(uint16_t Current_Loop,uint16_t Voltage_Loop,uint16_t *ProtectCompare){
  if(Current_Loop > Voltage_Loop)*ProtectCompare = Current_Loop;
  else *ProtectCompare = Voltage_Loop;
}