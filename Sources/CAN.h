#ifndef __CAN_H__
#define __CAN_H__

#include <mc9s12hy64.h>
#include "board.h"

#ifndef NULL
#define NULL    (0)
#endif

#define CAN_OVTM  1000

//CAN ID����
#define CAN0IDAR0V 0xff
#define CAN0IDAR1V 0xff
#define CAN0IDAR2V 0xff
#define CAN0IDAR3V 0xff
//CAN ID��������
#define CAN0IDMR0V 0xff
#define CAN0IDMR1V 0xf7  //ֻ������չ֡
#define CAN0IDMR2V 0xff
#define CAN0IDMR3V 0xff


//CAN ID����
#define CAN_ID_AMB                            0x18fef500  //��������
#define CAN_ID_LFE                            0x18fef200  //ȼ��������
#define CAN_ID_UREA_LEVEL_1                   0x18fe5600  //����ҺλID
//#define CAN_ID_UREA_LEVEL_2                   0x18FE56A3  //�������ҺλID
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
#define CAN_ID_DPFC1                          0x18FD7C00  //Ϋ�������ʹ����һ��ID����������ָʾ��
#define CAN_ID_ENGINE_OIL_USAGE               0x18FEE900

#define CAN_ID_UPDATE                         0x18FF8052
#define CAN_ID_HOST_UPDATE_CMD                0x18FF7552    //��λ�����������������Ϊ�豸����

#define CAN_ID_ENGINE_TYPE                    0X18FFDF00    //ʶ�𷢶������͵�ID(ֻ����񷢶�������)


//CAN���ݽ��ջ�������С����12 BytesΪ��λ��4���ֽ�CAN ID + 8���ֽ����ݣ�
#define REC_DATA_NUM                  50

//CAN���ϻ�������С��ÿ����Ŀ����SPN��FMI�ȣ�����Ϊ3�ı�������Ϊһ������ʾ3��DTC��
#define CAN_FAULT_RECORD_MAX_NUM      33

//CAN���ϳ���ʱ�䣨��CAN������Ϣֹͣ����ʱ���ù��ϴ��ڵ����ʱ�䣬��ʱ�������
#define CAN_FAULT_TTL                 5

#define CAN_SPEED_PAR        8

#define CAN_RECV_TIMEOUT 3000

//GPS״̬
#define GPS_STATE_WORK_ABNORMAL       0
#define GPS_STATE_51_DAYS_NO_SIGNAL   10
#define GPS_STATE_58_DAYS_NO_SIGNAL   20
#define GPS_STATE_61_DAYS_NO_SIGNAL   30
#define GPS_STATE_WORK_WELL           40
#define GPS_STATE_ATENNA_BROKEN       70
#define GPS_STATE_SHELL_OPENED        80
#define GPS_STATE_WEB_LOCK_CMD        90
#define GPS_STATE_LOST                100

//GPS����״̬λͼ
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
#define MSCAN_BAND_RATE 250000		/*-�ɷ�����config.h��[5/28/2013 rookie li]-*/
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


#define DataFrm    1    //����֡����

#define RemoteFrm  2    //Զ��֡����

#define CANE 7          //MSCAN12ģ��ʹ��λ

#define INITRQ 0        //MSCAN12ģ���ʼ������λ

#define INITACK 0       //MSCAN12ģ���ʼ��Ӧ��λ

#define CLKSRC 6        //MSCAN12ʱ��ѡ��λ

#define LOOPB 5         //�Բ�λ

#define LISTEN 4        //ֻ��ģʽλ,ֻ��ģʽʱ���ܷ���

#define RXF  0          //��������־λ

#define TXE0 0          //���ͻ�����0�ձ�־λ

#define TXE1 1          //���ͻ�����1�ձ�־λ

#define TXE2 2          //���ͻ�����2�ձ�־λ

#define TXS0 0          //���ͻ�����0��ѡ��

#define TXS1 1          //���ͻ�����1��ѡ��

#define TXS2 2          //���ͻ�����2��ѡ��

#define TXF0 0          //���ͻ�����0��

#define TXF1 1          //���ͻ�����1��

#define TXF2 2          //���ͻ�����2��   

//#if Freescale_mcu

#define IDE  3          //IDEλ

//#endif

#define RTR  4          //RTRλ

#define ERTR 0          //��չ֡�е�RTRλ

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

//CANͨ���õ�������������
//bit.7-6=00��ͬ����ת���Ϊ1��bit.5-0=000100,Ԥ��Ƶ����Ϊ4
//	#define CAN0BTR0V  (FOSC/MSCAN_BAND_RATE/10-1)

//bit.7=0�����β�����bit.6-4=010,ʱ���2Ϊ3��bit.3-0=0101,
//ʱ���1Ϊ6��0b00100101
//	#define CAN0BTR1V  0x25 
//����32λ�����˲���,�˲���0����
#define CAN0IDACV  0x00


//����������ж�
#define CAN0TIERV  0x00

//�������ͻ��������ж�
#define CAN0TIERV   0x00


#define MESSAGE_BASIC_FORMAT      0
#define MESSAGE_EXTENDED_FORMAT   1


  
#define CAN_COM_ERR		0X01
#define	CAN_MARNING1	0X02
#define	CAN_MARNING2	0X04
#define	CAN_LOCK			0X08
#define	LOCK_ON				0X80

//�������ʽ����λ/�����Ӧ�����־
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
    unsigned int    can_star_time;   //����CAN������Ԥ��ʱ���֮��ʼ
    unsigned int    can_status;
    unsigned int    can_st_buff;
} can_contral;

//CAN�������ݽṹ��
typedef struct
{
    unsigned char fuel_water;
    
    //����
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
    
    unsigned char   warning_level;  //GPS ״̬
    unsigned char   warning_ecu;    //EECU ״̬
    unsigned char   warning_time;
    unsigned char durationFor51DaysWarning;
    unsigned char durationFor58DaysWarning;
    
} CAN_RecvData_t;

//CAN������־
typedef struct
{
  bool engineFaultFlag;
  bool postProcessingFaultFlag;
  bool engineOilPressureLowFlag;
  bool ureaFaultFlag;
  
} CAN_AlarmFlag_t;

//CAN���Ͻṹ��
typedef struct 
{
  unsigned char sourceAddr; //����CAN������Ϣ���豸��ַ
  Uint8_t   lamp_status;
  Uint8_t   flash_lamp_status; 
  Uint8_t   can_fmi;
  Uint32_t  can_spn;
  Uint8_t   error_count;
  Uint8_t   error_time;
    
} CAN_Fault_t;

//���ϴ���ʽ
typedef enum _CAN_FaultProcMethod_t
{
  CAN_SINGLE_FAULT_PROC = 0,
  CAN_ECU_MULTI_FAULT_PROC = 2,
  CAN_FAULT_CLEAR = 1,
  
} CAN_FaultProcMethod_t;

//����NCD��ʻ��������ϵͳ״̬
typedef enum _NCD_PerformanceState_t
{                                        
  NCD_STATE_INACTIVE = 0,                     //��ʻ��������δ����
  NCD_STATE_ACTIVE = 1,                       //��ʻԱ����ϵͳ����
  NCD_STATE_WILL_LIMIT_TORQUE_SLIGHTLY = 2,   //������ʻ������ֵ��������
  NCD_STATE_TORQUE_LIMITED_SLIGHTLY = 3,      //������ʻ�������Ƽ���
  NCD_STATE_WILL_LIMIT_TORQUE_SEVERELY = 4,   //���ؼ�ʻ�������Ƽ�����
  NCD_STATE_TORQUE_LIMITED_SEVERELY = 5,      //���ؼ�ʻ�������Ƽ���
  NCD_STATE_INTERRUPT_TEMPORARILY = 6,        //��ʱ�ж�
  
} NCD_PerformanceState_t;

//����NCD��ʻԱ����ϵͳ״̬
typedef enum _NCD_DriverAlarmState_t
{
  NCD_DRIVER_ALARM_INACTIVE = 0,              //����ϵͳδ����
  NCD_DRIVER_ALARM_ACTIVE = 1,                //��ع��ϵ��±���ϵͳ����
  NCD_DRIVER_ALARM_UREA_LEVEL_LOW = 4,        //����Һλ��
  
} NCD_DriverAlarmState_t;

//����״̬
typedef enum _Regenerate_State_t
{
  REGENERATE_STATE_NONE = 0,                  //�������������ƣ�
  REGENERATE_STATE_IN_PROCESS = 1,            //���������У���ɫ��������
  REGENERATE_STATE_PARKING_PROMPT = 4,        //פ��������ʾ����ɫ��������
  REGENERATE_STATE_SERVICE_PROMPT = 2,        //����������ʾ����ɫ��������
  
} Regenerate_State_t;

//��������״̬
typedef enum _RegenerateInhibit_State_t
{
  REGENERATE_INHIBIT_STATE_NONE = 0,          //�ǽ�ֹ״̬�������ƣ�
  REGENERATE_INHIBIT_STATE_IN_PROCESS = 1,    //������ֹ�У���ɫ��������
};

//����״̬
typedef struct _CAN_PostprocessingState_t
{
  unsigned char ncdState;                 //NCD״̬
  unsigned char pcdState;                 //PCD״̬
  unsigned char regenerateState;          //����״̬
  unsigned char regenerateInhibitState;   //��������״̬
  unsigned char regenerateTemp;           //�����¶�
    
} CAN_PostprocessingState_t;

//(���)����״̬
typedef struct  _CAN_PostprocessingState_YC_t
{
  unsigned char regenerateRemind;          //(���)��������
  unsigned char regenerateProcess;         //(���)��������  
  unsigned char regenerateInhibitState;    //(���)��������״̬
  unsigned char exhaust_high_temperature;  //(���)�������¾���
}CAN_PostprocessingState_YC_t;

//����������
typedef enum _TYPE_of_Engine_t
{
  Weichai_engine,                            //Ϋ�񷢶���
  Yuchai_engine,                             //��񷢶���                                             
}TYPE_of_Engine_t;

#define  YC_EGR_MODE   0x01     //��񷢶���EGRģʽ��������ѭ������ʹ������
#define  YC_SCR_MODE   0x02     //��񷢶���SCRģʽ��ʹ�����ؽ��з�������

extern unsigned char NoconversionUreaLevel;      
extern unsigned char EN_YC_Urea_Function;        //ʹ����񷢶��������ع���

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

//�������������ͺ�����
extern unsigned int CAN_ReqOilCnt;

void Mscan_Initial(void);

unsigned char Can_Tx_Frame(
                           unsigned char ucNoBytes,     //�������ݸ���
                           unsigned char ucXtdFormat,   //��ʽ��extended/standard
                           unsigned long ulCANid,       //CANID
                           unsigned char* pDatapointer);//(unsigned char TXBUF)//
                           
unsigned char  Get_Idle_Sendbuf_Num(void);

void	Can_Par_Init(void);
void CAN_TaskProcess(void);
int CAN_SearchSpecificFault(unsigned long spn, unsigned char fmi);
unsigned char CAN_FaultNumStats(void);
void CAN_FaultProcess(CAN_FaultProcMethod_t method);

#endif
