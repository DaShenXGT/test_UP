#ifndef	_VEHICLE_H
#define	_VEHICLE_H

#define  MAX_SPEED              3000

//报警阈值
//水温高报警阈值
#define  WATER_WARN_VALUE       105           //paris_T3  liu~
#define  OIL_WARN_LIMIT_VALUE   600           //防止悬空时误报警
//发动机启动转速
#define  ENGINE_STARTUP_SPEED   350

//数字量输入字各Bit掩码（通过74HC165）
#define DIGITAL_BRAKE_PRESSURE_BIT            (unsigned long)0x00000002   //制动压力
#define DIGITAL_PREHEAT_BIT                   (unsigned long)0x00000008   //预热
#define DIGITAL_MUTE_BIT                      (unsigned long)0x00000040   //静音
#define DIGITAL_BACKLIGHT_BIT                 (unsigned long)0x00000080   //背光
#define DIGITAL_TRANSMISSION_OIL_PRESSURE_BIT (unsigned long)0x00000400   //变速油压（此处预留，目前用模拟电压采样测量油压）
#define DIGITAL_PARKING_BRAKE_BIT             (unsigned long)0x00001000   //驻车制动
#define DIGITAL_LANGUAGE_SWITCH_BIT           (unsigned long)0x00004000   //语言切换
#define DIGITAL_DISTANT_LIGHT_BIT             (unsigned long)0x00010000   //远光
#define DIGITAL_LEFT_TURN_BIT                 (unsigned long)0x00020000   //左转向
#define DIGITAL_RIGHT_TURN_BIT                (unsigned long)0x00040000   //右转向
#define DIGITAL_CHARGING_INDICATOR_BIT        (unsigned long)0x00200000   //充电指示
  //传动比设置 By周文熙20200731
#define SPEED_RATIO_K0_BIT                    (unsigned long)0x00000100   //车速比控制K0
#define SPEED_RATIO_K1_BIT                    (unsigned long)0x00000800   //车速比控制K1
#define SPEED_RATIO_K2_BIT                    (unsigned long)0x00008000   //车速比控制K2
#define SPEED_RATIO_K3_BIT                    (unsigned long)0x00000004   //车速比控制K3
  
  
  
  
//逻辑掩码
#define  IGN_ON                               (unsigned long)0x80000000
//#define LOGICAL_ELOCK_BIT                     (unsigned long)0x80000000   //电锁
#define LOGICAL_OIL_PRESSURE_LOW_BIT          (unsigned long)0x40000000   //机油压力低
#define LOGICAL_TRANS_OIL_PRESSURE_LOW_BIT    (unsigned long)0x20000000   //变速油压低
#define LOGICAL_FUEL_CONTAIN_WATER_BIT        (unsigned long)0x10000000   //燃油含水

//电锁状态
typedef enum
{
  ELOCK_OFF,
  ELOCK_JUST_TURN_OFF,
  ELOCK_ON,
  ELOCK_JUST_TURN_ON,
  
} Elock_State_t;

//发动机状态
typedef enum
{
  ENGINE_STOPPED,
  ENGINE_STOPPING,
  ENGINE_STARTED,
  ENGINE_STARTING,
         
} Engine_State_t;


#define ENGINE_STARTSTOP_DELAY 3000



extern  engineSpeedParams_t           engineSpeedParams;
extern  vehicle_motor           munich_motor[5];
extern  vehicle_warning         systemState;
extern  Uint32_t                LED_OnOffBitsMap;  
extern  Uint16_t             park_delay_time;

extern double Vehicle_TransOilPressure;
extern bool Vehicle_TransOilPressureLowFlag;
extern unsigned int Vehicle_TransOilPressureLowCnt;
extern bool Vehicle_TransOilPressureHighFlag;
extern unsigned int Vehicle_TransOilPressureHighCnt;

extern unsigned char    slow_flicher_flag;

void System_SelfCheckWhenElockOn(void);
void systemTasksProc(void);

void ATD_TransOilPressureProcess(void);
void Engine_CoolingLiquidTempProcess(void);
void Engine_UreaLevelProcess(void);
void voltageOutputInit(void);
void voltageOutputEnable(void);
void voltageOutputDisable(void);
void Led_Handle(void);
void Engine_CheckState(void);

#endif
