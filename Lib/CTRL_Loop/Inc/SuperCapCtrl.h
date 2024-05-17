#ifndef __SUPERCAPCTRL_H
#define __SUPERCAPCTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


typedef struct
{
    uint8_t TEST_bit :1;
    uint8_t SoftStart_bit :1;
    uint8_t Charge_bit : 1;
    uint8_t Discharge_bit :1;

    uint8_t UVP_bit :1;
    uint8_t OCP_bit :1;
    uint8_t Enable_bit :1;
} StateFlags;

typedef struct 
{
  uint8_t Flag;//超级电容开关指令，1为开，0为关
  uint8_t PowerLimint;//裁判系统功率限制
  uint8_t Dead; //机器人死亡状态，没用到
  uint8_t Move;//机器人行走状态，1为走动，0为不动
  uint8_t SuperCapCAN_Receive_T;//两个超级电容控制板之间互相通信使用的数据，对应控制板的状态标志（state）
}CAN_ReceiveData;//电控发过来的数据

typedef struct 
{
  uint8_t SuperCapID;//超级电容发送ID
  uint8_t SuperCapEnergy;//超级电容可用能量：0-100
  uint8_t ChassisPower; //底盘实时功率：0-255
  uint8_t SuperCapReady;//超级电容 可用标志：1为可用，0为不可用
  uint8_t SuperCapState;//超级电容 放电标志：1为正在放电，0为未放电
  uint8_t SuperCapProtect;//超级电容 保护标志：1为处于保护中，0为未保护
}CAN_TransmitData;//发送给电控的数据

typedef struct {
  uint16_t V_Buck;
  uint16_t I_Buck;
  uint16_t V_Boost;
  uint16_t I_Boost;
  uint16_t OVP_Buck;
  uint16_t OVP_Boost;
}Set_PID_Reference;

typedef struct {
  uint32_t Vcap;
  uint32_t Vbat;
  uint32_t Vboost;
  uint32_t Ibat;
  uint32_t Icap;
  uint32_t Tcore;
  uint32_t Tmos;
  uint32_t Tcap;
}ADC_Value;



#define PMOS_ON HAL_GPIO_WritePin(CAP_CTRL_GPIO_Port,CAP_CTRL_Pin,GPIO_PIN_SET)
#define PMOS_OFF HAL_GPIO_WritePin(CAP_CTRL_GPIO_Port,CAP_CTRL_Pin,GPIO_PIN_RESET)

#define TIM1_PWM2_BREAK HAL_GPIO_WritePin(TIM1_Break_CTRL_GPIO_Port,TIM1_Break_CTRL_Pin,GPIO_PIN_SET)
#define TIM1_PWM2_DISBREAK HAL_GPIO_WritePin(TIM1_Break_CTRL_GPIO_Port,TIM1_Break_CTRL_Pin,GPIO_PIN_RESET)

#define TEST_MODE HAL_GPIO_ReadPin(TEST_MODE_GPIO_Port,TEST_MODE_Pin)

#define TEST_OUT_HIGH  TEST_OUT_GPIO_Port->BSRR = (uint32_t)TEST_OUT_Pin
#define TEST_OUT_LOW  TEST_OUT_GPIO_Port->BRR = (uint32_t)TEST_OUT_Pin

#define LED_CAP_ON HAL_GPIO_WritePin(LED_CAP_GPIO_Port,LED_CAP_Pin,GPIO_PIN_SET)
#define LED_CAP_OFF HAL_GPIO_WritePin(LED_CAP_GPIO_Port,LED_CAP_Pin,GPIO_PIN_RESET)
#define LED_CAP_BLINK HAL_GPIO_TogglePin(LED_CAP_GPIO_Port,LED_CAP_Pin)

#define LED_CHASSIS_ON HAL_GPIO_WritePin(LED_CHASSIS_GPIO_Port,LED_CHASSIS_Pin,GPIO_PIN_SET)
#define LED_CHASSIS_OFF HAL_GPIO_WritePin(LED_CHASSIS_GPIO_Port,LED_CHASSIS_Pin,GPIO_PIN_RESET)
#define LED_CHASSIS_BLINK HAL_GPIO_TogglePin(LED_CHASSIS_GPIO_Port,LED_CHASSIS_Pin)

//分频系数，只能是2的幂次
#define DIV2 0x00000001
#define DIV4 0x00000003
#define DIV8 0x00000007
#define DIV16 0x0000000F
#define DIV32 0x0000001F
#define DIV64 0x0000003F
#define DIV128 0x0000007F
#define DIV256 0x000000FF
#define DIV512 0x000001FF
#define DIV1024 0x000003FF
#define DIV2048 0x000007FF
#define DIV4096 0x00000FFF
#define DIV8192 0x00001FFF

#define PID_BUCK_V_KP 1
#define PID_BUCK_V_KI 0.1
//Buck电压环PI

#define PID_BUCK_I_KP 0.1
#define PID_BUCK_I_KI 0.1
//Buck电流环PI


#define PID_BOOST_V_KP 1
#define PID_BOOST_V_KI 0.01
//Boost电压环PI

#define PID_BOOST_I_KP 0.001
#define PID_BOOST_I_KI 5
//Boost电流环PI

#define PID_DISCHARGE_BTA_PLIMIT_KP 0.5
#define PID_DISCHARGE_BTA_PLIMIT_KI 0.01
//电池与超电同时输出（功率环？）PI

#define BUCK_DUTY_COMPARE_MAX 25000
#define BUCK_DUTY_COMPARE_MIN 1300

#define BOOST_DUTY_COMPARE_MAX 26000
#define BOOST_DUTY_COMPARE_MIN 1400

#define CAN_SUPERCAP_ID 0x101
#define CAN_C_BOARD_ID 0x001
#define CAN_TEST_ID 0x255


#define VBAT_A 0.0073f
#define VBAT_B 0.3636f

#define VCAP_A 0.0078f
#define VCAP_B -0.1928f

#define VBOOST_A 0.0092f
#define VBOOST_B 0.1032f

#define IBAT_A 0.0026f
#define IBAT_B -0.2885f

#define ICAP_A -0.0086f
#define ICAP_B 22.189f

#define TO_ADC_IBAT(P,VADC) (110.962f + 10000000*(P) /(189.8 * (VADC)) + 9453.6)
#define TO_ADC_ICAP(P,VADC) (2580.116f - (P)/(0.00006708*(VADC) - 0.00165808f))

#define MAX_VCAP 19.5f
#define MAX_CHARGE_ICAP 10
#define MAX_ICAP_DISCHARGE 20
#define MAC_IBAT 7
#define MAX_VBAT 30

#define HARDWARE_PROTECT_VCAP 21
#define HARDWARE_PROTECT_IBAT 12
#define HARDWARE_PROTECT_ICAP 30
#define HARDWARE_ZERO_CURRENT_ADC_VALUE 2685

#define PBAT_ERROR_OFFSET 20//电池功率误差补偿
#define SAFE_CHARGE_ICAP 5//软起动充电安全电流
#define SAFE_CHARGE_VCAP 10



#define ADC_AVG_TIMES 1
#define ADC_CHANNEL_NUMBER 4 
#define ADC_BUFFER ADC_CHANNEL_NUMBER*ADC_AVG_TIMES

extern uint16_t ADC1Value[ADC_BUFFER];
extern uint16_t ADC2Value[ADC_BUFFER];



void Power_Loop_Parameter_Init(void);
void A_Timing_Ranking_Idea(void);


#ifdef __cplusplus
}
#endif

#endif /* SUPERCAP_CTRL_H */
