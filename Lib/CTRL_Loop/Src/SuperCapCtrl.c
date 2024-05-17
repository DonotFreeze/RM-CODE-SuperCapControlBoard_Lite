#include "SuperCapCtrl.h"
#include "tim.h"
#include "fdcan.h"

PID_Parameter PID_buck_V={0};
PID_Parameter PID_buck_I={0};
PID_Parameter PID_boost_V={0};
PID_Parameter PID_boost_I={0};
PID_Parameter PID_discharge_bat_Plimit={0};

ADC_Value ADC_Value_SUM ={0};
ADC_Value ADC_Value_AVG ={0};

FDCAN_RxHeaderTypeDef FDCAN1_RxHeader;
FDCAN_TxHeaderTypeDef FDCAN1_TxHeader;

CAN_TransmitData CAN_TX_data ={0};
CAN_ReceiveData CAN_RX_data={0};

StateFlags StateBit={0};
Set_PID_Reference test_loop_reference;

void ADC_Convert_To_Interim(void);
void ADC_Convert_To_Reality(void);
void Power_Loop(void);

void Power_Calculations(void);

void State_Change();
void State_Judgment(void);
void Power_On_Short_Current(void);
void TEST_Loop(void);
void Soft_Start_Loop(void);
void OCP_Loop(void);
void CAP_UVP_Loop(void);
void Charge_Loop(void);
void Discharge_Loop(void);
void Wait_Loop(void);
void Can_SendMess(CAN_TransmitData *TX_temp);
void ADC_Value_AVG_Compute(void);
void LED_Refresh(void);

void Test_Loop_Teference_Init(void);
void Buck_Voltage_Loop();
void Buck_Current_Loop_Without_OVP();
void Buck_Current_Loop_With_OVP();
void Boost_Voltage_Loop();
void Boost_Current_Loop_Without_OVP();
void Boost_Current_Loop_With_OVP();

//这部分是将给定的真实电压电流值转换为ADC对应的采样值，用于直接输入PID进行计算
const float ADC_MAX_VBOOST = (MAX_VBAT - VBOOST_B)/VBOOST_A;
const float ADC_MAX_VCAP = (MAX_VCAP - VCAP_B)/VCAP_A;
const float ADC_MAX_IBAT = (MAC_IBAT - IBAT_B)/IBAT_A;
const float ADC_MAX_ICAP_CHARGE = (MAX_CHARGE_ICAP-ICAP_B)/ICAP_A;
const float ADC_MAX_ICAP_DISCHARGE = (MAX_ICAP_DISCHARGE - ICAP_B)/ICAP_A;
const float ADC_SAFE_CHARGE_ICAP = (SAFE_CHARGE_ICAP - ICAP_B)/ICAP_A;


float ADC_Charge_ICAP=0;

uint16_t ADC1Value[ADC_BUFFER] = {0};
uint16_t ADC2Value[ADC_BUFFER] = {0};

float FB_Vbat = 0;//【ADC2_0】
float FB_Vcap = 0;//【ADC2_1】
float FB_Vboost = 0;//【ADC2_2】
float FB_Ibat = 0;//【ADC1_0】
float FB_Icap = 0;//【ADC1_1】
float FB_Pbat =0;//FB_Vbat * FB_Ibat
//环路计算中使用ADC原始值作为反馈值
uint16_t PID_CurrentLoopOut =0;
uint16_t PID_VoltageLoopOut =0;
uint16_t TIM1_PWM2_Comapre =0;

float Vbat;//【ADC2_0】电池电压、底盘供电电压
float Vcap;//【ADC2_1】超级电容组电压
float Vboost;//【ADC2_2】超级电容放电时，半桥Boost的输出电压
float Tcap;//【ADC2_3】超级电容组温度

float Ibat;//【ADC1_0】电池供电电流（裁判系统Chassis口电流）【功率限制】
float Icap;//【ADC1_1】超级电容组电流（充电为正，放电为负）
float Tcore;//【ADC1_2】G431核心温度
float Tmos;//【ADC1_3】超级电容控制板半桥温度

int16_t PowerLimitAfterOffset =0;
float PowerLimint_T; //裁判系统Chassis口功率限制
uint16_t BatPower; //裁判系统电源管理模块Chassis口功率
uint16_t ChassisPower; //底盘电机功率
int32_t SuperCapPower; //超级电容功率
float ChargePower; //充电功率
float DischargePower; //放电功率

float PowerLimitCurrent =0;

void Power_Loop_Parameter_Init(void){
    
    TIM1_PWM2_BREAK;
    PMOS_OFF;
    //开机把MOS全部关掉，防止浪涌电流

    PID_buck_V.Kp=PID_BUCK_V_KP;
    PID_buck_V.Ki=PID_BUCK_V_KI;
    PID_buck_V.OutMax = BUCK_DUTY_COMPARE_MAX;
    PID_buck_V.OutMin = BUCK_DUTY_COMPARE_MIN;
    PID_buck_V.ControllerDirection = DIRECT;//意思是，控制方向与变化方向相同
    PID_Init(&PID_buck_V);
    //Buck电压环PID参数初始化

    PID_buck_I.Kp=PID_BUCK_I_KP;
    PID_buck_I.Ki=PID_BUCK_I_KI;
    PID_buck_I.OutMax = BUCK_DUTY_COMPARE_MAX;
    PID_buck_I.OutMin = BUCK_DUTY_COMPARE_MIN;
    PID_buck_I.ControllerDirection = REVERSE;//意思是，控制方向与变化方向相同
    PID_Init(&PID_buck_I);
    //Buck电流环PID参数初始化

    PID_boost_V.Kp=PID_BOOST_V_KP;
    PID_boost_V.Ki=PID_BOOST_V_KI;
    PID_boost_V.OutMax = BOOST_DUTY_COMPARE_MAX;
    PID_boost_V.OutMin = BOOST_DUTY_COMPARE_MIN;
    PID_boost_V.ControllerDirection = REVERSE;//意思是，控制方向与变化方向相反
    //当设定输出为20V，当前输出为10V，正方向的PID提高电压的方法是增大输出，PID的输出直接作为上管PWM的占空比，当PID输出（上管占空比）变大时，电压变小；故控制反向
    PID_Init(&PID_boost_V);
    //Boost电压环PID参数初始化

    PID_boost_I.Kp=PID_BOOST_I_KP;
    PID_boost_I.Ki=PID_BOOST_I_KI;
    PID_boost_I.OutMax = BOOST_DUTY_COMPARE_MAX;
    PID_boost_I.OutMin = BOOST_DUTY_COMPARE_MIN;
    PID_boost_I.ControllerDirection = REVERSE;//意思是，控制方向与变化方向相反
    //当设定输出为10A，当前输出为5A，提高输出电流的方法是增大输出电压，故与上面相同，控制反向
    PID_Init(&PID_boost_I);
    //Boost电流环PID参数初始化

    PID_discharge_bat_Plimit.Kp = PID_DISCHARGE_BTA_PLIMIT_KP;
    PID_discharge_bat_Plimit.Ki = PID_DISCHARGE_BTA_PLIMIT_KI;
    PID_discharge_bat_Plimit.OutMax = BOOST_DUTY_COMPARE_MAX;
    PID_discharge_bat_Plimit.OutMin = BOOST_DUTY_COMPARE_MIN;
    PID_discharge_bat_Plimit.ControllerDirection = DIRECT;//意思是，控制方向与变化方向同向
    //当电池功率限制50W，当前输出30W，提高电池输出趋近功率限制，就需要将Boost的输出功率降低，降低功率的方法是降低Boost电压，
    //想要降低Boost的输出电压就需要增大上管的占空比，故输出需要增大；故控制方向正向
    Test_Loop_Teference_Init();
    StateBit.SoftStart_bit =1;
    StateBit.OCP_bit =0;
    StateBit.Charge_bit =1;
    StateBit.Discharge_bit =0;
    StateBit.Enable_bit =0;
    StateBit.UVP_bit =0;
}

uint32_t TimingCNT=0;
void A_Timing_Ranking_Idea(void){
    // TEST_OUT_HIGH;
    if(TEST_MODE)TIM1_PWM2_BREAK;
    if(!TEST_MODE)TIM1_PWM2_DISBREAK;

    // PMOS_ON;
    TimingCNT++;
    ADC_Convert_To_Interim();
    ADC_Convert_To_Reality();

    // State_Change();
    // Power_Loop();
        
    // Soft_Start_Loop();
    Charge_Loop();
    // Discharge_Loop();

    // Power_On_Short_Current();
    // Buck_Voltage_Loop();
    // Buck_Current_Loop_Without_OVP();
    // Buck_Current_Loop_With_OVP();
    // Boost_Voltage_Loop();
    // Boost_Current_Loop_Without_OVP();
    // Boost_Current_Loop_With_OVP();

    ADC_Value_AVG_Compute();
    
    // if((TimingCNT & DIV16) == 3)ADC_Convert_To_Reality();
    // if((TimingCNT & DIV1024) == 0)Can_SendMess(&CAN_TX_data);
    // if((TimingCNT & DIV8192) == 2)LED_Refresh();

    // TEST_OUT_LOW;
}

//这个函数直接将ADC的值保存到某个变量中，直接作为PID计算的反馈值，可以减少将其转换为真实值的运算时间，提高环路速度
//缺点显而易见，想要知道此时的真实反馈数值需要进行额外运算
//因为ADC的原始值范围是0-4095，而PWM占空比可用的值为0-1699所以会导致PI参数特别小（0.001的程度）
//但是！在开启PWM抖动模式下，PWM占空比可用的值为0*16-1700*16=0-27200，比ADC的原始值大了不少，这个时候PI就可以稍微大一点了
void ADC_Convert_To_Interim(void){
    FB_Vbat = (float)ADC2Value[0];
    FB_Vcap = (float)ADC2Value[1];
    FB_Vboost = (float)ADC2Value[2];
    FB_Ibat = (float)ADC1Value[0];
    FB_Icap = (float)ADC1Value[1];
    // FB_Pbat = (float)FB_Vbat*FB_Ibat;
}


void Power_Calculations(void){

}
uint16_t PMOS_OffDelay =0;
uint16_t UVP_delay =0;
uint8_t UVP_Lock =0;
void State_Change(){

    if(Vcap < 8 && (!StateBit.UVP_bit)){
        UVP_delay++;
        CAN_TX_data.SuperCapReady =0;
        CAN_TX_data.SuperCapState =0;
        if(UVP_delay >= 1000){
            StateBit.Discharge_bit =0;
            StateBit.Enable_bit =0;
            StateBit.UVP_bit =1;
            TIM1_PWM2_BREAK;
            PMOS_OFF;
            UVP_delay =0;
        }
    }else if(Vcap >10 &&(!StateBit.UVP_bit)){
        UVP_delay =0;
        CAN_TX_data.SuperCapReady =1;
    }


    if(CAN_TX_data.SuperCapReady && CAN_RX_data.Flag &&(!StateBit.UVP_bit)){
        StateBit.Discharge_bit = 1;
        if(CAN_RX_data.Move){
            StateBit.Enable_bit =1;
        }else {
            StateBit.Enable_bit =0;
        }
    }else {
        StateBit.Discharge_bit =0;
    }

    if(!StateBit.Discharge_bit && (!StateBit.UVP_bit)){
        StateBit.Charge_bit = 1;
        if(CAN_RX_data.Move){
            StateBit.Enable_bit =0;
        }else {
            StateBit.Enable_bit =1;
        }
    }else {
        StateBit.Charge_bit =0;
    }

    if(Ibat <0.5 || PMOS_OffDelay){
        PMOS_OffDelay ++;
        if(PMOS_OffDelay >=200){
            PMOS_OFF;
            PMOS_OffDelay =0;
        }
    }else if(Ibat >=1){
        PMOS_ON;
        PMOS_OffDelay =0;
    }  

}


void Power_Loop(){
    // State_Judgment();
    // if(StateBit.TEST_bit){
    //     TEST_Loop();//测试模式
    // }else 
    if(StateBit.SoftStart_bit){
        Soft_Start_Loop();//开机软启动
    }else if(StateBit.OCP_bit){
        OCP_Loop();//过流保护
    }else if(StateBit.UVP_bit){
        CAP_UVP_Loop();//超级电容低电压保护，设定值为电容组额定电压的30%
    }else if(StateBit.Charge_bit && StateBit.Enable_bit){
        Charge_Loop();//【充电】：未开启超电，且不移动的时候才会使能超电的充电
    }else if(StateBit.Discharge_bit && StateBit.Enable_bit){
        Discharge_Loop();//【放电】：开启超电，且移动的时候才会使能超电放电
    }else if((StateBit.Charge_bit || StateBit.Discharge_bit) && (!StateBit.Enable_bit)){
        Wait_Loop();//【等待】：未开启超电，且不移动；防止超电充电会跟底盘抢功率
                    //         开启超电，且不移动；防止超电无脑将电池功率拉至功率限制，但是车子又不走，此时所有功率都会往超级电容走，可能会导致超级电容被充爆
    }
}

uint16_t OCP_Delay_CNT = 0;
void State_Judgment(void){
    if((FB_Ibat >= ADC_MAX_IBAT) || (FB_Icap < ADC_MAX_ICAP_CHARGE) || OCP_Delay_CNT){
        OCP_Delay_CNT ++;
        if (OCP_Delay_CNT > 50)
        {
            StateBit.OCP_bit = 1;
        }else if((FB_Ibat < 3600) || (FB_Icap > 500)){
            StateBit.OCP_bit = 0;
            OCP_Delay_CNT = 0;
        }
        
    }
}

void TEST_Loop(void){

}

uint8_t Buck_voltage_Rising = 0;
uint8_t Buck_voltage_Falling = 0;
uint16_t SS_Delay_CNT =0;

void Soft_Start_Loop(void){
    if(!CAN_RX_data.Move){
        PowerLimitAfterOffset = CAN_RX_data.PowerLimint - PBAT_ERROR_OFFSET;
        if(PowerLimitAfterOffset <0)PowerLimitAfterOffset = 0;
        ADC_Charge_ICAP = 2502 - (312500 *PowerLimitAfterOffset)/(18.975f * FB_Vcap + 2599);//此处需要自己换算出公式
        // ADC_Charge_ICAP = ((100/Vcap) - ICAP_B)/ICAP_A;
        if(ADC_Charge_ICAP < ADC_SAFE_CHARGE_ICAP)ADC_Charge_ICAP = ADC_SAFE_CHARGE_ICAP;//取大的值是因为，电流越大ADC值越小，需要限定不超过这个电流，所以取大的ADC值

        PID_Compute(&PID_buck_I,FB_Icap,ADC_Charge_ICAP,&PID_CurrentLoopOut);
        PID_Compute(&PID_buck_V,FB_Vcap,ADC_MAX_VCAP,&PID_VoltageLoopOut);
        Loop_Competition_Buck(PID_CurrentLoopOut,PID_VoltageLoopOut,&TIM1_PWM2_Comapre);

        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,TIM1_PWM2_Comapre);
        uint16_t TIM1_OC1_Compare = (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 3);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,TIM1_OC1_Compare);
    }
    if(Vcap > SAFE_CHARGE_VCAP){
        StateBit.SoftStart_bit =0;
    }
}

uint16_t ShortCurrentProtect_CNT =0;
void Power_On_Short_Current(void){
    if((FB_Ibat >= 2000) || (FB_Icap < 100) || ShortCurrentProtect_CNT){
        ShortCurrentProtect_CNT ++;
        if (ShortCurrentProtect_CNT > 50)
        {
            TIM1_PWM2_BREAK;
            PMOS_OFF;
            CAN_TX_data.SuperCapReady = 0;
            CAN_TX_data.SuperCapState = 0;
            CAN_TX_data.SuperCapEnergy =0;
            CAN_TX_data.SuperCapProtect =1;
            PID_Clear_Integral(&PID_buck_V);
            PID_Clear_Integral(&PID_buck_I);
            PID_Clear_Integral(&PID_boost_V);
            PID_Clear_Integral(&PID_boost_I);
            PID_Clear_Integral(&PID_discharge_bat_Plimit);
            StateBit.OCP_bit = 1;
        }else if((FB_Ibat < 1600) && (FB_Icap > 500)){
            StateBit.OCP_bit = 0;
            ShortCurrentProtect_CNT = 0;
            StateBit.SoftStart_bit =1;
        }
        
    }
}

void OCP_Loop(void){
    TIM1_PWM2_BREAK;
    PMOS_OFF;
    CAN_TX_data.SuperCapReady = 0;
    CAN_TX_data.SuperCapState = 0;
    PID_Clear_Integral(&PID_buck_V);
    PID_Clear_Integral(&PID_buck_I);
    PID_Clear_Integral(&PID_boost_V);
    PID_Clear_Integral(&PID_boost_I);
    PID_Clear_Integral(&PID_discharge_bat_Plimit);
}


// uint16_t VcapRising =0;
void CAP_UVP_Loop(void){
    // TIM1_PWM2_BREAK;
    // PMOS_OFF;
    CAN_TX_data.SuperCapReady = 0;
    CAN_TX_data.SuperCapState = 0;

    
    if(!CAN_RX_data.Move){
        PMOS_ON;
        PowerLimitAfterOffset = CAN_RX_data.PowerLimint - PBAT_ERROR_OFFSET;
        if(PowerLimitAfterOffset <0)PowerLimitAfterOffset = 0;

        ADC_Charge_ICAP = 2580.116f - (PowerLimitAfterOffset)/(0.00006708*(FB_Vcap) - 0.00165808f);
        if(ADC_Charge_ICAP < ADC_SAFE_CHARGE_ICAP)ADC_Charge_ICAP = ADC_SAFE_CHARGE_ICAP;

        PID_Compute(&PID_buck_I,FB_Icap,ADC_Charge_ICAP,&PID_CurrentLoopOut);
        PID_Compute(&PID_buck_V,FB_Vcap,ADC_MAX_VCAP,&PID_VoltageLoopOut);
        Loop_Competition_Buck(PID_CurrentLoopOut,PID_VoltageLoopOut,&TIM1_PWM2_Comapre);

        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,TIM1_PWM2_Comapre);
        uint16_t TIM1_OC1_Compare = (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 3);
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,TIM1_OC1_Compare);

        TIM1_PWM2_DISBREAK;
    }else {
        TIM1_PWM2_BREAK;
        PMOS_OFF;
    }

    if(Vcap >= SAFE_CHARGE_VCAP && (!CAN_RX_data.Flag) ){
        StateBit.UVP_bit =0;
        StateBit.Charge_bit =1;
        CAN_TX_data.SuperCapReady = 1;
    }

}

// void Charge_Loop(void){
    
//     PowerLimitAfterOffset = CAN_RX_data.PowerLimint - PBAT_ERROR_OFFSET;
//     if(PowerLimitAfterOffset <0)PowerLimitAfterOffset = 0;

//     ADC_Charge_ICAP = 2580.116f - (PowerLimitAfterOffset)/(0.00006708*(FB_Vcap) - 0.00165808f);
//     if(ADC_Charge_ICAP < ADC_MAX_ICAP_CHARGE)ADC_Charge_ICAP = ADC_MAX_ICAP_CHARGE;
//     if(ADC_Charge_ICAP > HARDWARE_ZERO_CURRENT_ADC_VALUE -10) ADC_Charge_ICAP = HARDWARE_ZERO_CURRENT_ADC_VALUE ;//充电的时候，ADC的值应该小于零电流值，否则电流会反向。

//     PID_Compute(&PID_buck_I,FB_Icap,ADC_Charge_ICAP,&PID_CurrentLoopOut);
//     PID_Compute(&PID_buck_V,FB_Vcap,ADC_MAX_VCAP,&PID_VoltageLoopOut);
//     Loop_Competition_Buck(PID_CurrentLoopOut,PID_VoltageLoopOut,&TIM1_PWM2_Comapre);

//     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,TIM1_PWM2_Comapre);
//     uint16_t TIM1_OC1_Compare = (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 3);
//     __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,TIM1_OC1_Compare);

// }


void Charge_Loop(void){
    // CAN_TX_data.SuperCapReady =1;
    CAN_TX_data.SuperCapState =0;
    PowerLimitAfterOffset = CAN_RX_data.PowerLimint - PBAT_ERROR_OFFSET;
    if(PowerLimitAfterOffset <0)PowerLimitAfterOffset = 0;

     // float PowerLimitCurrent = 75.56 + 100/(0.00001675f * FB_Vbat + 0.00234925f);
    PowerLimitCurrent = 110.962f +(PowerLimitAfterOffset) /(0.00001898 * (ADC_Value_AVG.Vbat)) + 0.00094536;
    if(PowerLimitCurrent > ADC_MAX_IBAT)PowerLimitCurrent = ADC_MAX_IBAT;
    if(PowerLimitCurrent < 10) PowerLimitCurrent = 10;//放电的时候，ADC的值应该大于零电流值，否则电流会反向。


    PID_Compute(&PID_discharge_bat_Plimit,FB_Ibat,PowerLimitCurrent,&PID_CurrentLoopOut);//开启超电之后，超电对电池电流进行闭环，将其限制在功率限制/电池电压的电流大小
    PID_Compute(&PID_buck_V,FB_Vcap,ADC_MAX_VCAP,&PID_VoltageLoopOut);
    Loop_Competition_Buck(PID_CurrentLoopOut,PID_VoltageLoopOut,&TIM1_PWM2_Comapre);

    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,TIM1_PWM2_Comapre);
    uint16_t TIM1_OC1_Compare = (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 3);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,TIM1_OC1_Compare);

}

void Discharge_Loop(void){
    // PMOS_ON;
    CAN_TX_data.SuperCapState =1;
    // CAN_TX_data.SuperCapReady =1;
    PowerLimitAfterOffset = CAN_RX_data.PowerLimint - PBAT_ERROR_OFFSET;
    if(PowerLimitAfterOffset <0)PowerLimitAfterOffset = 0;

    // float PowerLimitCurrent = 75.56 + 100/(0.00001675f * FB_Vbat + 0.00234925f);
    PowerLimitCurrent = 110.962f +(PowerLimitAfterOffset) /(0.00001898 * (ADC_Value_AVG.Vbat)) + 0.00094536;
    if(PowerLimitCurrent > ADC_MAX_IBAT)PowerLimitCurrent = ADC_MAX_IBAT;
    // if(PowerLimitCurrent < HARDWARE_ZERO_CURRENT_ADC_VALUE +10) PowerLimitCurrent = HARDWARE_ZERO_CURRENT_ADC_VALUE;//放电的时候，ADC的值应该大于零电流值，否则电流会反向。


    PID_Compute(&PID_discharge_bat_Plimit,FB_Ibat,PowerLimitCurrent,&PID_CurrentLoopOut);//开启超电之后，超电对电池电流进行闭环，将其限制在功率限制/电池电压的电流大小
    PID_Compute(&PID_boost_V,FB_Vboost,ADC_MAX_VBOOST,&PID_VoltageLoopOut);
    Loop_Competition_Boost(PID_CurrentLoopOut,PID_VoltageLoopOut,&TIM1_PWM2_Comapre);

    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,TIM1_PWM2_Comapre);
    uint16_t TIM1_OC1_Compare = (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 3);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,TIM1_OC1_Compare);

}

//关闭半桥，并且关闭PMOS，将超级电容对底盘功率消耗降到最低。
void Wait_Loop(void){
    TIM1_PWM2_BREAK;
    PMOS_OFF;
    CAN_TX_data.SuperCapState = 0;
    // CAN_TX_data.SuperCapReady = 1;
    PID_Clear_Integral(&PID_buck_V);
    PID_Clear_Integral(&PID_buck_I);
    PID_Clear_Integral(&PID_boost_V);
    PID_Clear_Integral(&PID_boost_I);
    PID_Clear_Integral(&PID_discharge_bat_Plimit);
}

void LED_Refresh(void){
    if(StateBit.OCP_bit || StateBit.UVP_bit){
        LED_CAP_BLINK;
        LED_CHASSIS_BLINK;
    }else if(StateBit.Charge_bit){
        LED_CAP_BLINK;
        LED_CHASSIS_OFF;
    }else if(StateBit.Discharge_bit){
        LED_CAP_OFF;
        LED_CHASSIS_BLINK;
    }else if(CAN_TX_data.SuperCapEnergy >=95){
        LED_CAP_ON;
        LED_CHASSIS_OFF;
    }else if(!StateBit.Enable_bit){
        LED_CAP_OFF;
        LED_CHASSIS_ON;
    }
}



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    uint8_t CAN_RX_BUFF[8]={0};
    HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&FDCAN1_RxHeader,CAN_RX_BUFF);//接收数据
    if(FDCAN1_RxHeader.Identifier ==CAN_C_BOARD_ID){
        CAN_RX_data.Flag = (uint8_t)CAN_RX_BUFF[0];
        CAN_RX_data.PowerLimint = (uint8_t)CAN_RX_BUFF[1]; 
        CAN_RX_data.Dead = (uint8_t)CAN_RX_BUFF[2]; 
        CAN_RX_data.Move = (uint8_t)CAN_RX_BUFF[3];
        CAN_RX_data.SuperCapCAN_Receive_T = (uint8_t)CAN_RX_BUFF[6];
    }

}


void Can_SendMess(CAN_TransmitData *TX_temp)
{		
    
    FDCAN1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    FDCAN1_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    FDCAN1_TxHeader.Identifier = CAN_SUPERCAP_ID;
    FDCAN1_TxHeader.IdType = FDCAN_STANDARD_ID;
    FDCAN1_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    FDCAN1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    uint8_t  CAN_TX_BUFF[8]={0};
    // CAN_TX_BUFF[3] = TX_temp->SuperCapProtect;
    CAN_TX_BUFF[6] = TX_temp->SuperCapState;
    CAN_TX_BUFF[5] = TX_temp->ChassisPower;
    CAN_TX_BUFF[4] = TX_temp->SuperCapReady;
    CAN_TX_BUFF[7] = TX_temp->SuperCapEnergy;
    
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN1_TxHeader, CAN_TX_BUFF);

}



uint32_t ADC_SUM_CNT =0;
void ADC_Value_AVG_Compute(void){
    ADC_SUM_CNT++;
    // for (uint8_t i = 0; i < ADC_AVG_TIMES; i ++){
        uint16_t Offset =0;
        // Offset = i << 2;
        ADC_Value_SUM.Ibat += (uint32_t)ADC1Value[Offset];
        ADC_Value_SUM.Icap += (uint32_t)ADC1Value[Offset+1];
        ADC_Value_SUM.Tcore += (uint32_t)ADC1Value[Offset+2];
        ADC_Value_SUM.Tmos += (uint32_t)ADC1Value[Offset+3];

        ADC_Value_SUM.Vbat += (uint32_t)ADC2Value[Offset];
        ADC_Value_SUM.Vcap += (uint32_t)ADC2Value[Offset + 1];
        ADC_Value_SUM.Vboost += (uint32_t)ADC2Value[Offset + 2];
        ADC_Value_SUM.Tcap += (uint32_t)ADC2Value[Offset + 3];
    // }
    if((ADC_SUM_CNT & DIV64) == 1){
        ADC_Value_AVG.Ibat = ADC_Value_SUM.Ibat >> 6;
        ADC_Value_AVG.Icap = ADC_Value_SUM.Icap >> 6;
        ADC_Value_AVG.Vbat = ADC_Value_SUM.Vbat >> 6;
        ADC_Value_AVG.Vcap = ADC_Value_SUM.Vcap >> 6;
        ADC_Value_AVG.Vboost = ADC_Value_SUM.Vboost >> 6;
        ADC_Value_SUM.Ibat =0;
        ADC_Value_SUM.Icap =0;
        ADC_Value_SUM.Tcore =0;
        ADC_Value_SUM.Tmos =0;
        ADC_Value_SUM.Vbat =0;
        ADC_Value_SUM.Vcap =0;
        ADC_Value_SUM.Vboost =0;
        ADC_Value_SUM.Tcap =0;
    }
    
}


//这个函数的作用是，将ADC的值转化为其对应的各个量的真实值，提供调试使用，但是不参与环路反馈计算。
void ADC_Convert_To_Reality(void){

    Vbat = ADC_Value_AVG.Vbat * VBAT_A + VBAT_B;
    Vcap = ADC_Value_AVG.Vcap * VCAP_A + VCAP_B;
    Vboost = ADC_Value_AVG.Vboost * VBOOST_A + VBOOST_B;

    Ibat = ADC_Value_AVG.Ibat *IBAT_A + IBAT_B;
    Icap = ADC_Value_AVG.Icap *ICAP_A + ICAP_B;
    
    BatPower = Ibat * Vbat;
    SuperCapPower = Vcap * Icap;
    if (StateBit.Charge_bit){
        ChassisPower = BatPower - SuperCapPower;//充电的时候，电容充电功率不计入底盘电机消耗功率中，所以要减掉（虽然说减掉之后基本就没了）
    }else if(StateBit.Discharge_bit){
        DischargePower = Vcap * Icap;//放电的时候，电流是负的，所以他算出来的放电功率也是负的
        ChassisPower = BatPower - SuperCapPower;//电池功率 - 放电功率（负的）就是电容放电的时候，电机的总底盘功率
    }
    CAN_TX_data.SuperCapEnergy = (Vcap -7)*100/12;
}

void Test_Loop_Teference_Init(void){
    test_loop_reference.V_Buck =2000;
    test_loop_reference.I_Buck =2000;
    test_loop_reference.OVP_Buck = 3500;
    test_loop_reference.V_Boost = 2500;
    test_loop_reference.I_Boost = 2000;
    test_loop_reference.OVP_Boost = 3000;
}

void Buck_Voltage_Loop(){
    PMOS_ON;
    PID_Compute(&PID_buck_V,FB_Vcap,test_loop_reference.V_Buck,&PID_VoltageLoopOut);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PID_VoltageLoopOut);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,((PID_VoltageLoopOut >> 3) + (PID_VoltageLoopOut >> 3) + (PID_VoltageLoopOut >> 3)));
}

void Buck_Current_Loop_Without_OVP(){
    PID_Compute(&PID_buck_I,FB_Icap,test_loop_reference.I_Buck,&PID_CurrentLoopOut);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PID_CurrentLoopOut);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,((PID_CurrentLoopOut >> 3) + (PID_CurrentLoopOut >> 3) + (PID_CurrentLoopOut >> 3)));
}

void Buck_Current_Loop_With_OVP(){

    PID_Compute(&PID_buck_I,FB_Icap,test_loop_reference.I_Buck,&PID_CurrentLoopOut);
    PID_Compute(&PID_buck_V,FB_Vcap,test_loop_reference.OVP_Buck,&PID_VoltageLoopOut);
    Loop_Competition_Buck(PID_CurrentLoopOut,PID_VoltageLoopOut,&TIM1_PWM2_Comapre);

    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,TIM1_PWM2_Comapre);
    uint16_t TIM1_PWM1_Comapre = (TIM1_PWM2_Comapre >>3) +(TIM1_PWM2_Comapre >>3) + (TIM1_PWM2_Comapre >>3);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,TIM1_PWM1_Comapre);
}

void Boost_Voltage_Loop(){
    PMOS_ON;
    PID_Compute(&PID_boost_V,FB_Vbat,test_loop_reference.V_Boost,&PID_VoltageLoopOut);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PID_VoltageLoopOut);
    uint16_t TIM1_PWM1_Comapre = (PID_VoltageLoopOut >>3) +(PID_VoltageLoopOut >>3) + (PID_VoltageLoopOut >>3);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,TIM1_PWM1_Comapre);
}

void Boost_Current_Loop_Without_OVP(){
    PID_Compute(&PID_boost_I,FB_Icap,test_loop_reference.I_Boost,&PID_CurrentLoopOut);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,PID_CurrentLoopOut);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,((PID_CurrentLoopOut >> 3) + (PID_CurrentLoopOut >> 3) + (PID_CurrentLoopOut >> 3)));
}

void Boost_Current_Loop_With_OVP(){
    PID_Compute(&PID_boost_I,FB_Icap,test_loop_reference.I_Boost,&PID_CurrentLoopOut);
    
    PID_Compute(&PID_boost_V,FB_Vboost,test_loop_reference.OVP_Boost,&PID_VoltageLoopOut);
    Loop_Competition_Boost(PID_CurrentLoopOut,PID_VoltageLoopOut,&TIM1_PWM2_Comapre);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,TIM1_PWM2_Comapre);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,((TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 3) + (TIM1_PWM2_Comapre >> 3)));
}