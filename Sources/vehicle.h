#ifndef	_VEHICLE_H
#define	_VEHICLE_H

#define  MAX_SPEED              3000

//������ֵ
//ˮ�¸߱�����ֵ
#define  WATER_WARN_VALUE       105           //paris_T3  liu~
#define  OIL_WARN_LIMIT_VALUE   600           //��ֹ����ʱ�󱨾�
//����������ת��
#define  ENGINE_STARTUP_SPEED   350

//�����������ָ�Bit���루ͨ��74HC165��
#define DIGITAL_BRAKE_PRESSURE_BIT            (unsigned long)0x00000002   //�ƶ�ѹ��
#define DIGITAL_PREHEAT_BIT                   (unsigned long)0x00000008   //Ԥ��
#define DIGITAL_MUTE_BIT                      (unsigned long)0x00000040   //����
#define DIGITAL_BACKLIGHT_BIT                 (unsigned long)0x00000080   //����
#define DIGITAL_TRANSMISSION_OIL_PRESSURE_BIT (unsigned long)0x00000400   //������ѹ���˴�Ԥ����Ŀǰ��ģ���ѹ����������ѹ��
#define DIGITAL_PARKING_BRAKE_BIT             (unsigned long)0x00001000   //פ���ƶ�
#define DIGITAL_LANGUAGE_SWITCH_BIT           (unsigned long)0x00004000   //�����л�
#define DIGITAL_DISTANT_LIGHT_BIT             (unsigned long)0x00010000   //Զ��
#define DIGITAL_LEFT_TURN_BIT                 (unsigned long)0x00020000   //��ת��
#define DIGITAL_RIGHT_TURN_BIT                (unsigned long)0x00040000   //��ת��
#define DIGITAL_CHARGING_INDICATOR_BIT        (unsigned long)0x00200000   //���ָʾ
  //���������� By������20200731
#define SPEED_RATIO_K0_BIT                    (unsigned long)0x00000100   //���ٱȿ���K0
#define SPEED_RATIO_K1_BIT                    (unsigned long)0x00000800   //���ٱȿ���K1
#define SPEED_RATIO_K2_BIT                    (unsigned long)0x00008000   //���ٱȿ���K2
#define SPEED_RATIO_K3_BIT                    (unsigned long)0x00000004   //���ٱȿ���K3
  
  
  
  
//�߼�����
#define  IGN_ON                               (unsigned long)0x80000000
//#define LOGICAL_ELOCK_BIT                     (unsigned long)0x80000000   //����
#define LOGICAL_OIL_PRESSURE_LOW_BIT          (unsigned long)0x40000000   //����ѹ����
#define LOGICAL_TRANS_OIL_PRESSURE_LOW_BIT    (unsigned long)0x20000000   //������ѹ��
#define LOGICAL_FUEL_CONTAIN_WATER_BIT        (unsigned long)0x10000000   //ȼ�ͺ�ˮ

//����״̬
typedef enum
{
  ELOCK_OFF,
  ELOCK_JUST_TURN_OFF,
  ELOCK_ON,
  ELOCK_JUST_TURN_ON,
  
} Elock_State_t;

//������״̬
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
