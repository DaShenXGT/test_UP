#ifndef __CAN_H__
#define __CAN_H__

#include <mc9s12hy64.h>
#include "board.h"

#ifndef NULL
#define NULL    (0)
#endif

#define CAN_OVTM  1000

//CAN ID接收
#define CAN0IDAR0V 0xff
#define CAN0IDAR1V 0xff
#define CAN0IDAR2V 0xff
#define CAN0IDAR3V 0xff
//CAN ID接收屏蔽
#define CAN0IDMR0V 0xff
#define CAN0IDMR1V 0xf7  //只接收扩展帧
#define CAN0IDMR2V 0xff
#define CAN0IDMR3V 0xff


//CAN ID定义
#define CAN_ID_AMB                            0x18fef500  //环境条件
#define CAN_ID_LFE                            0x18fef200  //燃油消耗率
#define CAN_ID_UREA_LEVEL_1                   0x18fe5600  //尿素液位ID
//#define CAN_ID_UREA_LEVEL_2                   0x18FE56A3  //玉柴尿素液位ID
//#define CAN_ID_UREA_LEVEL_2                   0x18fe563d
#define CAN_ID_DM1                            0x18FECA00
#define CAN_ID_DM1_DM2_BAM                    0x18ecff00
#define CAN_ID_DM1_DM2_PACKET                 0x18EBFF00
#define CAN_ID_ELECTRONIC_ENGINE_CONTROLLER1  0x0CF00400
#define CAN_ID_ENGINE_TEMPERATURE             0x18FEEE00
#define CAN_ID_ENGINE_HOURS                   0x18FEE500
#define CAN_ID_ENGINE_FLUID_LEVEL_PRESSURE    0x18FEEF00
#define CAN_ID_WATER_IN_FUEL_INDICATOR        0x18FEFF00
#define CAN_ID_GPS_INFO                       0x18FA05EE
#define CAN_ID_EECU_INFO                      0x18FB0800
#define CAN_ID_PREHEAT                        0X18FEE400
#define CAN_ID_HANDBRAKE                      0x18FDB200
#define CAN_ID_REQUEST                        0x18EA0052  
#define CAN_ID_DPFC1                          0x18FD7C00  //潍柴玉柴都是使用这一个ID来控制再生指示灯
#define CAN_ID_ENGINE_OIL_USAGE               0x18FEE900

#define CAN_ID_UPDATE                         0x18FF8052
#define CAN_ID_HOST_UPDATE_CMD                0x18FF7552    //上位机发送升级命令，数据为设备类型

#define CAN_ID_ENGINE_TYPE                    0X18FFDF00    //识别发动机类型的ID(只有玉柴发动机发送)


//CAN数据接收缓冲区大小（以12 Bytes为单位，4个字节CAN ID + 8个字节数据）
#define REC_DATA_NUM                  50

//CAN故障缓冲区大小（每个条目包括SPN、FMI等，设置为3的倍数，因为一屏会显示3个DTC）
#define CAN_FAULT_RECORD_MAX_NUM      33

//CAN故障持续时间（当CAN故障信息停止发送时，该故障存在的最大时间，超时则清除）
#define CAN_FAULT_TTL                 5

#define CAN_SPEED_PAR        8

#define CAN_RECV_TIMEOUT 3000

//GPS状态
#define GPS_STATE_WORK_ABNORMAL       0
#define GPS_STATE_51_DAYS_NO_SIGNAL   10
#define GPS_STATE_58_DAYS_NO_SIGNAL   20
#define GPS_STATE_61_DAYS_NO_SIGNAL   30
#define GPS_STATE_WORK_WELL           40
#define GPS_STATE_ATENNA_BROKEN       70
#define GPS_STATE_SHELL_OPENED        80
#define GPS_STATE_WEB_LOCK_CMD        90
#define GPS_STATE_LOST                100

//GPS报警状态位图
#define GPS_ALARM_WORK_ABNORMAL         0x0001
#define GPS_ALARM_51_DAYS_NO_SIGNAL     0x0002
#define GPS_ALARM_58_DAYS_NO_SIGNAL     0x0004
#define GPS_ALARM_61_DAYS_NO_SIGNAL     0x0008
#define GPS_ALARM_ATENNA_BROKEN         0x0010
#define GPS_ALARM_SHELL_OPENED          0x0020
#define GPS_ALARM_WEB_LOCK_CMD          0x0040
#define GPS_ALARM_LOST                  0x0080

#define EECU_ALARM_KEY_INCORRECT        (unsigned int)0x0100
#define EECU_ALARM_KEY_INCORRECT_LOCKED (unsigned int)0x0200

/*******************************************************************************
*  Select CAN version
*******************************************************************************/
#define CAN2A	0
#define CAN2B	1
#define CAN_V	CAN2B			//Only edit this line to be CAN2A or CAN2B

#if CAN_V==CAN2A 
//#define	CAN_FRAME_MSG_LENGTH	3         
#define	CAN_IDE					0         
#elif CAN_V==CAN2B 
//#define	CAN_FRAME_MSG_LENGTH	5         
#define	CAN_IDE					1      
#define	CAN_PRT					0
#endif

#define PARIS_T3       1                    //liu~
#define ENGING_TYPE_CAN   0

#define FOSC 8000000
#define MSCAN_BAND_RATE 250000		/*-可放置在config.h中[5/28/2013 rookie li]-*/
#define polling 0

#if  MSCAN_BAND_RATE == 250000

#define CAN0BTR0V  0x01       
#define CAN0BTR1V  0x49

#elif MSCAN_BAND_RATE == 100000

#define CAN0BTR0V  0x07       
#define CAN0BTR1V  0x25       

#elif  MSCAN_BAND_RATE == 50000

#define CAN0BTR0V  0x0f       
#define CAN0BTR1V  0x25       

#elif  MSCAN_BAND_RATE == 20000

#define CAN0BTR0V  0x28       
#define CAN0BTR1V  0x25       

#else  MSCAN_BAND_RATE == 10000

#define CAN0BTR0V  0x4f       
#define CAN0BTR1V  0x25       


#error This MSCAN_BAND_RATE value is not in "mscanh.h" file

#endif

//!  config can bandrate register
#define Can_conf_bandrate( )     {CANBTR0 = CAN0BTR0V;CANBTR1 = CAN0BTR1V; }

/*******************************************************************************
*  Select CAN Receive Mode
*******************************************************************************/
#define CAN_POLLING_RECEIVE		1
#define CAN_INTR_RECEIVE		0
//#define CAN_RECEIVE_MODE	CAN_POLLING_RECEIVE


#define DataFrm    1    //数据帧类型

#define RemoteFrm  2    //远程帧类型

#define CANE 7          //MSCAN12模块使能位

#define INITRQ 0        //MSCAN12模块初始化请求位

#define INITACK 0       //MSCAN12模块初始化应答位

#define CLKSRC 6        //MSCAN12时钟选择位

#define LOOPB 5         //自测位

#define LISTEN 4        //只听模式位,只听模式时不能发送

#define RXF  0          //接收满标志位

#define TXE0 0          //发送缓冲区0空标志位

#define TXE1 1          //发送缓冲区1空标志位

#define TXE2 2          //发送缓冲区2空标志位

#define TXS0 0          //发送缓冲区0被选中

#define TXS1 1          //发送缓冲区1被选中

#define TXS2 2          //发送缓冲区2被选中

#define TXF0 0          //发送缓冲区0满

#define TXF1 1          //发送缓冲区1满

#define TXF2 2          //发送缓冲区2满   

//#if Freescale_mcu

#define IDE  3          //IDE位

//#endif

#define RTR  4          //RTR位

#define ERTR 0          //扩展帧中的RTR位

#define CANTXF CANTXIDR0
#define CANRXF CANRXIDR0

#define EN_MSCAN_WAKEUP_INTERRUPT           0X80
#define EN_MSCAN_STATA_CHANGE_INTERRUPT     0X40
#define EN_MSCAN_RX_OFF_INTERRUPT           0X10
#define EN_MSCAN_RX_ERR_OFF_INTERRUPT       0X20
#define EN_MSCAN_RX_ALL_INTERRUPT           0X30 
#define EN_MSCAN_TX_OFF_INTERRUPT           0X04
#define EN_MSCAN_TX_ERR_OFF_INTERRUPT       0X08
#define EN_MSCAN_TX_ALL_INTERRUPT           0X0c
#define EN_MSCAN_OVER_RUN_INTERRUPT         0X02
#define EN_MSCAN_RX_FULL_INTERRUPT          0X01

//CAN通信用到的设置量定义
//bit.7-6=00，同步跳转宽度为1，bit.5-0=000100,预分频因子为4
//	#define CAN0BTR0V  (FOSC/MSCAN_BAND_RATE/10-1)

//bit.7=0，单次采样，bit.6-4=010,时间段2为3，bit.3-0=0101,
//时间段1为6，0b00100101
//	#define CAN0BTR1V  0x25 
//两个32位验收滤波器,滤波器0命中
#define CAN0IDACV  0x00


//不允许接收中断
#define CAN0TIERV  0x00

//不允许发送缓冲区空中断
#define CAN0TIERV   0x00


#define MESSAGE_BASIC_FORMAT      0
#define MESSAGE_EXTENDED_FORMAT   1


  
#define CAN_COM_ERR		0X01
#define	CAN_MARNING1	0X02
#define	CAN_MARNING2	0X04
#define	CAN_LOCK			0X08
#define	LOCK_ON				0X80

//处理错误方式：置位/清除相应错误标志
typedef enum
{
  CAN_FAULT_SET_FLAG,
  CAN_FAULT_CLEAR_FLAG,
  
} CAN_FaultTreatment_t;

typedef struct
{
    unsigned char   can_cmd;
    unsigned char   can_id;
    unsigned int    can_time;
    unsigned int    can_time0;
    unsigned int    can_time1;
    unsigned int    can_time2;
    unsigned int    can_time3;
    unsigned long   can_time4;
    unsigned char   can_wait_time;
    unsigned int    can_star_time;   //控制CAN任务在预定时间段之后开始
    unsigned int    can_status;
    unsigned int    can_st_buff;
} can_contral;

//CAN接收数据结构体
typedef struct
{
    unsigned char fuel_water;
    
    //新增
    unsigned char handbrake;
    
    double ambientAirTemp;
    double ambientAirTempOld;
    
    double engineFuelRate;
    double engineFuelRateOld;
    
    double fuelConsumption;
    double fuelConsumptionOld;
       
    unsigned int currWaterTemp;
    unsigned int oldWaterTemp;
    
    unsigned int    oil_temp;
    
    unsigned int    can_speed;
    
    unsigned long engineWorkingTime;
    unsigned long engineWorkingTimeOld;
    
    unsigned char   can_preheat;
    unsigned char currUreaLevel;
    unsigned char oldUreaLevel;
    
    unsigned char sourceAddr;
    unsigned int    multi_bag_len;
    unsigned char   multi_bag_num;
    unsigned char   multi_bag_time;
    unsigned char   multi_bag_data[80];
    unsigned long   pgn_id;
    unsigned char   can_total_num;
    
    unsigned char   warning_level;  //GPS 状态
    unsigned char   warning_ecu;    //EECU 状态
    unsigned char   warning_time;
    unsigned char durationFor51DaysWarning;
    unsigned char durationFor58DaysWarning;
    
} CAN_RecvData_t;

//CAN报警标志
typedef struct
{
  bool engineFaultFlag;
  bool postProcessingFaultFlag;
  bool engineOilPressureLowFlag;
  bool ureaFaultFlag;
  
} CAN_AlarmFlag_t;

//CAN故障结构体
typedef struct 
{
  unsigned char sourceAddr; //发出CAN故障信息的设备地址
  Uint8_t   lamp_status;
  Uint8_t   flash_lamp_status; 
  Uint8_t   can_fmi;
  Uint32_t  can_spn;
  Uint8_t   error_count;
  Uint8_t   error_time;
    
} CAN_Fault_t;

//故障处理方式
typedef enum _CAN_FaultProcMethod_t
{
  CAN_SINGLE_FAULT_PROC = 0,
  CAN_ECU_MULTI_FAULT_PROC = 2,
  CAN_FAULT_CLEAR = 1,
  
} CAN_FaultProcMethod_t;

//后处理NCD驾驶性能限制系统状态
typedef enum _NCD_PerformanceState_t
{                                        
  NCD_STATE_INACTIVE = 0,                     //驾驶性能限制未激活
  NCD_STATE_ACTIVE = 1,                       //驾驶员报警系统激活
  NCD_STATE_WILL_LIMIT_TORQUE_SLIGHTLY = 2,   //初级驾驶性能限值即将激活
  NCD_STATE_TORQUE_LIMITED_SLIGHTLY = 3,      //初级驾驶性能限制激活
  NCD_STATE_WILL_LIMIT_TORQUE_SEVERELY = 4,   //严重驾驶性能限制即将激
  NCD_STATE_TORQUE_LIMITED_SEVERELY = 5,      //严重驾驶性能限制激活
  NCD_STATE_INTERRUPT_TEMPORARILY = 6,        //暂时中断
  
} NCD_PerformanceState_t;

//后处理NCD驾驶员报警系统状态
typedef enum _NCD_DriverAlarmState_t
{
  NCD_DRIVER_ALARM_INACTIVE = 0,              //报警系统未激活
  NCD_DRIVER_ALARM_ACTIVE = 1,                //相关故障导致报警系统激活
  NCD_DRIVER_ALARM_UREA_LEVEL_LOW = 4,        //尿素液位低
  
} NCD_DriverAlarmState_t;

//再生状态
typedef enum _Regenerate_State_t
{
  REGENERATE_STATE_NONE = 0,                  //不再生（不亮灯）
  REGENERATE_STATE_IN_PROCESS = 1,            //再生过程中（黄色，常亮）
  REGENERATE_STATE_PARKING_PROMPT = 4,        //驻车再生提示（黄色，闪亮）
  REGENERATE_STATE_SERVICE_PROMPT = 2,        //服务再生提示（红色，常亮）
  
} Regenerate_State_t;

//再生抑制状态
typedef enum _RegenerateInhibit_State_t
{
  REGENERATE_INHIBIT_STATE_NONE = 0,          //非禁止状态（不亮灯）
  REGENERATE_INHIBIT_STATE_IN_PROCESS = 1,    //再生禁止中（红色，常亮）
};

//后处理状态
typedef struct _CAN_PostprocessingState_t
{
  unsigned char ncdState;                 //NCD状态
  unsigned char pcdState;                 //PCD状态
  unsigned char regenerateState;          //再生状态
  unsigned char regenerateInhibitState;   //再生抑制状态
  unsigned char regenerateTemp;           //再生温度
    
} CAN_PostprocessingState_t;

//(玉柴)后处理状态
typedef struct  _CAN_PostprocessingState_YC_t
{
  unsigned char regenerateRemind;          //(玉柴)再生提醒
  unsigned char regenerateProcess;         //(玉柴)再生过程  
  unsigned char regenerateInhibitState;    //(玉柴)再生抑制状态
  unsigned char exhaust_high_temperature;  //(玉柴)排气高温警告
}CAN_PostprocessingState_YC_t;

//发动机类型
typedef enum _TYPE_of_Engine_t
{
  Weichai_engine,                            //潍柴发动机
  Yuchai_engine,                             //玉柴发动机                                             
}TYPE_of_Engine_t;

#define  YC_EGR_MODE   0x01     //玉柴发动机EGR模式，即废气循环，不使用尿素
#define  YC_SCR_MODE   0x02     //玉柴发动机SCR模式，使用尿素进行废弃处理

extern unsigned char NoconversionUreaLevel;      
extern unsigned char EN_YC_Urea_Function;        //使能玉柴发动机上尿素功能

extern  can_contral	can_task;
extern CAN_Fault_t CAN_FaultRecordBuffer[CAN_FAULT_RECORD_MAX_NUM];
extern CAN_RecvData_t  can_data;

//Newly added by liuyongguang @ 2018/3/27
extern unsigned int CAN_ReqWorkingTimeCnt;
extern CAN_AlarmFlag_t CAN_AlarmFlag;
extern bool CAN_RecvEngineHourReqAckFlag;
extern CAN_PostprocessingState_t CAN_PostprocessingState;

extern CAN_PostprocessingState_YC_t CAN_PostprocessingState_YC;

extern unsigned int CAN_RecvEngineHourReqAckTimeout;
//extern unsigned char TYPE_of_Engine;

extern TYPE_of_Engine_t TYPE_of_Engine; 

//新增发动机总油耗请求
extern unsigned int CAN_ReqOilCnt;

void Mscan_Initial(void);

unsigned char Can_Tx_Frame(
                           unsigned char ucNoBytes,     //发送数据个数
                           unsigned char ucXtdFormat,   //格式：extended/standard
                           unsigned long ulCANid,       //CANID
                           unsigned char* pDatapointer);//(unsigned char TXBUF)//
                           
unsigned char  Get_Idle_Sendbuf_Num(void);

void	Can_Par_Init(void);
void CAN_TaskProcess(void);
int CAN_SearchSpecificFault(unsigned long spn, unsigned char fmi);
unsigned char CAN_FaultNumStats(void);
void CAN_FaultProcess(CAN_FaultProcMethod_t method);

#endif
