#include <hidef.h>
#include <mc9s12hy64.h>
#include "app.h"
#include "board.h"
#include "CAN.h"
#include "atd.h"
#include "lcd.h"
#include "vehicle.h"
#include "digital.h"
#include "time.h"
#include "update.h"
#include "led.h"

//CAN接收数据存储及处理相关
unsigned char ucCanRece[REC_DATA_NUM][12];                      //CAN接收缓存
unsigned char data_rec_point;                                   //CAN接收缓存数据存储索引
unsigned char data_deal_point;                                  //CAN接收缓存数据处理索引

//CAN故障相关
CAN_Fault_t CAN_FaultRecordBuffer[CAN_FAULT_RECORD_MAX_NUM];    //CAN故障记录缓存
Uint32_t CAN_FaultBitMap;                                       //对应存储故障条目的CAN接收缓存位图 
CAN_Fault_t CAN_SingleFault;                                    //当前故障信息

//存储水温、尿素液位等CAN信息
CAN_RecvData_t  can_data;

//潍柴发动机定义后处理
CAN_PostprocessingState_t CAN_PostprocessingState =
{
  NCD_STATE_INACTIVE,
  0,
  REGENERATE_STATE_NONE,
  REGENERATE_INHIBIT_STATE_NONE,
  0,
};
//玉柴发动机定义后处理
CAN_PostprocessingState_YC_t CAN_PostprocessingState_YC =
{
  REGENERATE_STATE_NONE,
  REGENERATE_STATE_NONE,
  REGENERATE_INHIBIT_STATE_NONE,
  0x00,
};
//发动机时间
unsigned int CAN_ReqWorkingTimeCnt = 0;                         //请求时间计数器
const unsigned char CAN_WorkingTimePGN[3]= {0xE5, 0xFE, 0x00};  //发动机时间，PGN 0x00FEE5，Sent least significant byte first 
bool CAN_RecvEngineHourReqAckFlag = false;                      //ECU返回时间标志
unsigned int CAN_RecvEngineHourReqAckTimeout = 0;               //时间请求超时计数器


//发动机总油耗
unsigned int CAN_ReqOilCnt = 0;                                 //请求燃油总量计数器
const unsigned char Can_OilUsagePGN[3] = {0xE9,0xFE,0x00};      //发动机总油耗，PGN 0x00FEE9

//发动机转速接收超时计数器
unsigned int CAN_EngineSpeedRecvTimeout = CAN_RECV_TIMEOUT; 
unsigned int CAN_CoolingLiquidTempRecvTimeout = CAN_RECV_TIMEOUT;
unsigned int CAN_UreaLevelRecvTimeout = CAN_RECV_TIMEOUT;

//CAN报警标志
CAN_AlarmFlag_t CAN_AlarmFlag =
{
  false,     //发动机故障标志
  false,     //后处理故障标志
  false,     //机油压力低故障标志
  false,     //尿素相关后处理故障
};

//发送数据缓冲区
unsigned char ucCanSend0[8];                    //0x1fff5252   第1~4字节是小时计  第5~8   LED状态       //liu~                     //0x1bf00417，第1字节油位：转速;4低字节，5高字节
unsigned char ucCanSend1[8];                    //0x1fff5352   1 设备类型	2 气压值	3 油位	4 水温	5 油温	6~7 转速            //0x03F00517 报警量+累计工作时间
const unsigned char CAN_DeviceInfoBuffer[8] = {0x02, 0x06, 0x11, 0x02, 0x01, 0x10, 0x00, 0x00};
unsigned char CAN_UreaLevelBuffer[8];

can_contral	can_task;

//External Vars
extern queue_list systemTask; 
extern Uint32_t LED_OnOffBitsMap;
extern Uint16_t CAN_GPSWarningBitsMap;
extern Engine_State_t Engine_State;
extern unsigned long LCD_OldCanFaultSpnFmi;
extern bool Meter_UreaLevelFirstProcessFlag;

//发动机类型 （首先默认设置为潍柴发动机）
TYPE_of_Engine_t TYPE_of_Engine = Weichai_engine;
unsigned char NoconversionUreaLevel=0;
unsigned char EN_YC_Urea_Function = true ;

/*******************************************************************************
@ Func: CAN模块初始化
*******************************************************************************/
void Mscan_Initial(void) 
{
    //STB 置低,TJA1040正常工作
    DDRP  |= (DDRP_DDRP1_MASK);
    PTP   &= (~PTP_PTP1_MASK); 
    
    CANCTL1 |= (1 << CANE);                 //mscan module enable   

    CANCTL0 |= (1 << INITRQ);               //MSCAN in initialization mode

    while ((CANCTL1 & (1<<INITACK)) == 0)   //-等待进入初始化模式[5/28/2013 rookie li]
    {
    }

    CANCTL0 |= (CANCTL0_TIME_MASK);	      	//-Enable internal MSCAN time[5/28/2013 rookie li]

//	CANCTL1 |= CLKSRC;
    CANCTL1 &= ~(1 << CLKSRC);              //external crystal for mscan clock   

    //set can bus bandrate
    Can_conf_bandrate();     

    //!  self loop for test mscan module  
    //CANCTL1 |= (1 << LOOPB);              //回环自测模式

    CANCTL1 &= ~(1 << LISTEN);              //normal operation mode

    //!  accept filter register    
    CANIDAC = CAN0IDACV;                    //两个32位验收过滤器?

    CANIDAR0 = CAN0IDAR0V; 
    CANIDAR1 = CAN0IDAR1V;
    CANIDAR2 = CAN0IDAR2V; 
    CANIDAR3 = CAN0IDAR3V;                  //为什么只是用前4个？[5/28/2013 rookie li]
    CANIDAR4 = 0xFF; 
    CANIDAR5 = 0xFF;
    CANIDAR6 = 0xFF; 
    CANIDAR7 = 0xFF;

    //CANIDMR0 = CAN0IDMR0V; 
    //CANIDMR1 = CAN0IDMR1V;
    //CANIDMR2 = CAN0IDMR2V; 
    //CANIDMR3 = CAN0IDMR3V;
    CANIDMR0 = 0xFF; 
    CANIDMR1 = 0xFF;
    CANIDMR2 = 0xFF; 
    CANIDMR3 = 0xFF;
    CANIDMR4 = 0xFF; 
    CANIDMR5 = 0xFF;
    CANIDMR6 = 0xFF; 
    CANIDMR7 = 0xFF;      

    //!  set can module work module 
    CANTIER = CAN0TIERV;  //set interrupt modle：No interrupt request is generated from this event

    CANCTL0 &= ~(1<<INITRQ);   //INITRQ=0，exit initial

    //这里应该用&
    while ((CANCTL1 & (1<<INITACK)) == 1)//wait for ack
    {
    }
    
    while(!(CANCTL0_SYNCH));		// wait for CAN module to synch 

    CANRFLG = 0xFF;			//-清零寄存器[5/28/2013 rookie li]

    //!  can receiver stata change interrupt enable
    //CANRIER |= EN_MSCAN_STATA_CHANGE_INTERRUPT;

    //!  tx and rx leave error or bus off stata interrupt    
    //CANRIER |= EN_MSCAN_TX_ALL_INTERRUPT;

    //CANRIER |= EN_MSCAN_RX_ALL_INTERRUPT;

    //!  overun interrupt
    //CANRIER |= EN_MSCAN_OVER_RUN_INTERRUPT;

//#if  CAN_RX_INTERRUPT
    CANRIER |= EN_MSCAN_RX_FULL_INTERRUPT;  //turn on receive interrupt
//#endif   
}

/*******************************************************************************
@ Func: CAN通信参数初始化
*******************************************************************************/
void	Can_Par_Init(void)
{
    Uint8_t can_par_mi,can_par_mj; 
    
    can_task.can_cmd = 0;
    can_task.can_id = 0;
    can_task.can_time0 = 100;
    can_task.can_time1 = 200;
    can_task.can_time2 = 300;
    can_task.can_time3 = 400;
    can_task.can_time4 = 500;
    can_task.can_star_time = 500;
    
    data_rec_point = 0;
    data_deal_point = 0;
    CAN_FaultBitMap = 0;
    
    can_data.multi_bag_len = 0;
    can_data.multi_bag_num = 0;
    can_data.multi_bag_time = 0;
    //can_data.can_oil_press_warn = 0;
    can_data.currWaterTemp = 0;
    CAN_GPSWarningBitsMap = 0;
    //can_data.warning_ecu = 0xff;
    can_data.warning_ecu = 0x00;
    can_data.warning_level = 0xff;
    
    for(can_par_mi = 0;can_par_mi < 80;can_par_mi++)
       can_data.multi_bag_data[can_par_mi] = 0;
       
    for(can_par_mi = 0;can_par_mi < REC_DATA_NUM;can_par_mi++)
    {
        for(can_par_mj = 0;can_par_mj < 12;can_par_mj++)
        {
            ucCanRece[can_par_mi][can_par_mj] = 0;
        }
    }
    
    for(can_par_mi = 0;can_par_mi < CAN_FAULT_RECORD_MAX_NUM;can_par_mi++)
    {
    	  CAN_FaultRecordBuffer[can_par_mi].lamp_status = 0;
        CAN_FaultRecordBuffer[can_par_mi].flash_lamp_status = 0;
    	  CAN_FaultRecordBuffer[can_par_mi].can_fmi = 0;
    	  CAN_FaultRecordBuffer[can_par_mi].can_spn = 0;
    	  CAN_FaultRecordBuffer[can_par_mi].error_count = 0;
    	  CAN_FaultRecordBuffer[can_par_mi].error_time = 0;
    }
    
    
//	get_can_status();
}

/****************************************************************************
*
* Function name :Get_Idle_Sendbuf_Num    Function: find idle transmite buffer
*
* Input : None                           Output : Idle buffer num or oxff
*
*****************************************************************************/
unsigned char  Get_Idle_Sendbuf_Num(void)
{   
    unsigned char Get_Tmp;
    unsigned char bufbit = 0xFF;         
    Get_Tmp =  CANTFLG;

    //!  find can transmit buffer number
    if ((Get_Tmp & (1 << TXE0)) != 0)
    {  
        //!  transmit buffer 0 empty      
        bufbit = 1;  
    }
    else if ((Get_Tmp & (1 << TXE1)) != 0) 
    {  
        //!  transmit buffer 1 empty       
        bufbit = 2;
    }
    else if ((Get_Tmp & (1 << TXE2)) != 0) 
    {  
        //!  transmit buffer 2 empty 
        bufbit = 4;
    }
    return bufbit;//return empty  buffer number
}


/*******************************************************************************
* Function		: Can_Tx_Frame
* Description	: Initialises a send data box，自动寻找空闲MOB;
* Called By		: main()
* Input	 :	
*			ucNoBytes        Number of data bytes. Permissible values 0 .. 8
*			ucXtdFormat		   MESSAGE_EXTENDED_FORMAT or MESSAGE_BASIC_FORMAT
*			iCANId           CAN message identifier 
*     pDatapointer     指向发送数据
* Output :  
* Return : 发送成功返回1，没有空闲buffer返回0
* Others :
*******************************************************************************/
unsigned char Can_Tx_Frame(
              unsigned char ucNoBytes,
						  unsigned char ucXtdFormat,
						  unsigned long ulCANid,
						  unsigned char* pDatapointer)//(unsigned char TXBUF)//
{
    unsigned char ucTemp0,ucTemp1,ucTemp2;
    unsigned char ucIndex;
    //unsigned char Can_Temp,Can_Temp1;

    //!  Transmit buffer address
    //unsigned char * addr; 

    //!  Idle transmit buffer number
    unsigned char BufNum = 0xFF;        

    //Can_Tx_Data_Frames =  Can_Tx_Buf;

    //!  find idle transmit buffer
    BufNum = Get_Idle_Sendbuf_Num();       

    //!  return 0x00,no idle transmit buffer
    if (BufNum == 0xFF)   
    {
        return 0;  
    }

    //!  select idle transmit buffer
    CANTBSEL |= BufNum;

    /*
    *	by rookie_li
    */
    if(ucXtdFormat == MESSAGE_EXTENDED_FORMAT)
    {
        ulCANid <<= 3;
        CANTXIDR0	= (ulCANid>>24);
        ucTemp0		= (ulCANid>>16);
        ucTemp1		= ((ucTemp0>>2)&0x07);	//保证低三位为ID17-ID15
        ucTemp2		= (ucTemp0 & 0xE0);		//保证高三位为ID20-ID18
        CANTXIDR1	= (ucTemp1|ucTemp2|CANTXIDR1_SRR_MASK|CANTXIDR1_IDE_MASK);
        ulCANid		>>= 2;
        CANTXIDR2	= (ulCANid>>8);
        CANTXIDR3	= ulCANid;				/*-数据帧[5/29/2013 rookie li]-*/
    }
    else
    {
        /*-标准格式[5/29/2013 rookie li]-*/
        ulCANid <<= 5;
        CANTXIDR0	= (ulCANid>>8);
        CANTXIDR1	= (ulCANid);
        /*-bit3=0:0 Standard format[5/29/2013 rookie li]-*/
        /*-bit4=0:0 Data frame[5/29/2013 rookie li]-*/
        CANTXIDR1	&= 0xE0;
    }

    /*-发送字节个数可控制[5/29/2013 rookie li]-*/
    for(ucIndex=0;ucIndex<ucNoBytes;ucIndex++) 
    {
        *(&CANTXDSR0+ucIndex) = pDatapointer[ucIndex];	  
    }
    can_task.can_wait_time = 400;
    CANTXDLR	= ucNoBytes;

    CANTFLG = BufNum; //发送缓冲区0准备就绪  write of 1 clears flag
//	while((CANTFLG&BufNum) != BufNum);/*-Wait for Transmission completion-*/
    while(((CANTFLG&BufNum) != BufNum) && can_task.can_wait_time);/*-Wait for Transmission completion-*/
    

    //Delay_N_Nop(800) ;

    //return (Can_Tx_Data_Frames->Tx_Flag);  
    return(1);        
}

/************************************************************************************************
@ Func:  搜索特定故障所在故障数组索引
@ Param: spn <--> DTC SPN
         fmi <--> DTC FMI
@ Ret:   -1   <--> 未找到该故障
         正数 <--> 该故障索引
************************************************************************************************/
int CAN_SearchSpecificFault(unsigned long spn, unsigned char fmi/*, unsigned char lamp*/)
{
  unsigned int i = 0;

  for(i = 0; i < CAN_FAULT_RECORD_MAX_NUM; ++i)
  {
    if((CAN_FaultRecordBuffer[i].can_spn == spn) && (CAN_FaultRecordBuffer[i].can_fmi == fmi) /*&& (CAN_FaultRecordBuffer[i])*/)
    {
      return i;
    }
  }

  return -1;
}

/************************************************************************************************
@ Func:  清除特定故障信息
@ Param: spn <--> DTC SPN
         fmi <--> DTC FMI
************************************************************************************************/
void CAN_ClearSpecificFault(unsigned long spn, unsigned char fmi)
{
  unsigned int i = 0;
  
  for(i = 0; i < CAN_FAULT_RECORD_MAX_NUM; ++i)
  {
    if((CAN_FaultRecordBuffer[i].can_spn == spn) && (CAN_FaultRecordBuffer[i].can_fmi == fmi))
    {
      CAN_FaultRecordBuffer[i].sourceAddr = 0;
      CAN_FaultRecordBuffer[i].can_spn = 0;
      CAN_FaultRecordBuffer[i].can_fmi = 0;
      CAN_FaultRecordBuffer[i].lamp_status = 0;
      CAN_FaultRecordBuffer[i].flash_lamp_status=0;
      CAN_FaultRecordBuffer[i].error_count = 0;
      
      CAN_FaultBitMap &= ~((unsigned long)1 << i);
      
      if(can_data.can_total_num)
      {    
        can_data.can_total_num--;
      }
      
      break;
    }
  }
}

/**********************************************************************************
 * Func：  统计故障总数
**********************************************************************************/
unsigned char CAN_FaultNumStats(void)
{
	unsigned char num = 0;
	unsigned char i = 0;
	
	for(i = 0; i < CAN_FAULT_RECORD_MAX_NUM; ++i)
	{
		if(CAN_FaultRecordBuffer[i].can_spn)
		{
			++num;
		}
	}
	
	return num;
}


/************************************************************************************************
@ Func:  CAN故障处理
@ Param: method <-> 故障处理方式，单帧故障、ECU多帧故障、DCU多帧故障、故障清除
************************************************************************************************/
void CAN_FaultProcess(CAN_FaultProcMethod_t method)
{
    Uint32_t  can_data_spn;
    Uint32_t  can_spn_buff;
    Uint8_t   can_data_fmi;
    unsigned char lampStatus = 0;
    unsigned char flashlampStatus = 0;
    unsigned char i = 0;
    unsigned char j = 0;
    int pos = 0;
    
    switch(method)
    {
      //单帧故障处理
      case CAN_SINGLE_FAULT_PROC:
        //查找该故障是否已存在
        pos = CAN_SearchSpecificFault(CAN_SingleFault.can_spn, CAN_SingleFault.can_fmi);
        //若已存在则只刷新存在时间
        if(pos >= 0)
        {
          CAN_FaultRecordBuffer[pos].error_time = CAN_FAULT_TTL;
          //指示灯状态改变则更新
          if(CAN_FaultRecordBuffer[pos].lamp_status != CAN_SingleFault.lamp_status)
          {
            CAN_FaultRecordBuffer[pos].lamp_status = CAN_SingleFault.lamp_status;
          }
          //闪烁指示灯状态
          if(CAN_FaultRecordBuffer[pos].flash_lamp_status != CAN_SingleFault.flash_lamp_status)
          {
            CAN_FaultRecordBuffer[pos].flash_lamp_status = CAN_SingleFault.flash_lamp_status;
          }
        }
        //为新发生故障则记录之
        else if(-1 == pos)
        {
          pos = CAN_SearchSpecificFault(0, 0);
          if(pos >= 0)
          {
            CAN_FaultRecordBuffer[pos].sourceAddr = CAN_SingleFault.sourceAddr;
            CAN_FaultRecordBuffer[pos].lamp_status = CAN_SingleFault.lamp_status;
            CAN_FaultRecordBuffer[pos].flash_lamp_status = CAN_SingleFault.flash_lamp_status;
            CAN_FaultRecordBuffer[pos].can_spn = CAN_SingleFault.can_spn;
            CAN_FaultRecordBuffer[pos].can_fmi = CAN_SingleFault.can_fmi;
            
            CAN_FaultRecordBuffer[pos].error_time = CAN_FAULT_TTL;
            
            CAN_FaultBitMap |= ((unsigned long)1 << pos);
            
            if(can_data.can_total_num < CAN_FAULT_RECORD_MAX_NUM)
            {
              can_data.can_total_num += 1;
            }
          }
          else if(-1 == pos)
          {
            //NOP
          }
        }
      break;
      
      //ECU多帧故障处理
      case CAN_ECU_MULTI_FAULT_PROC:
        lampStatus = can_data.multi_bag_data[0];
        flashlampStatus = can_data.multi_bag_data[1];
        for(i = 2; i < can_data.multi_bag_len; i++)
        {
          //提取SPN
          can_data_spn = can_data.multi_bag_data[i++];
          can_spn_buff = can_data.multi_bag_data[i++];
          can_spn_buff <<= 8;
          can_data_spn |= can_spn_buff;
          can_spn_buff = can_data.multi_bag_data[i] >> 5;
          can_spn_buff <<= 16;
          can_data_spn |= can_spn_buff;
          
          //提取FMI
          can_data_fmi = can_data.multi_bag_data[i] & 0x1f;
          
          //跳过[CM OC]字节
          ++i;
            
          pos = CAN_SearchSpecificFault(can_data_spn, can_data_fmi);
          if(pos >= 0)
          {
            CAN_FaultRecordBuffer[pos].error_time = CAN_FAULT_TTL;
            //指示灯状态改变则更新
            if(CAN_FaultRecordBuffer[pos].lamp_status != lampStatus/*CAN_SingleFault.lamp_status*/)
            {
              CAN_FaultRecordBuffer[pos].lamp_status = lampStatus/*CAN_SingleFault.lamp_status*/;
            }
            //闪烁指示灯状态 
            if(CAN_FaultRecordBuffer[pos].flash_lamp_status != flashlampStatus)
            {
              CAN_FaultRecordBuffer[pos].flash_lamp_status = flashlampStatus;
            }
          }
          else if(-1 == pos)
          {
            pos = CAN_SearchSpecificFault(0, 0);
            if(pos >= 0)
            {
              CAN_FaultRecordBuffer[pos].lamp_status = lampStatus;
              CAN_FaultRecordBuffer[pos].flash_lamp_status = flashlampStatus;
              CAN_FaultRecordBuffer[pos].sourceAddr = can_data.sourceAddr;
              CAN_FaultRecordBuffer[pos].can_spn = can_data_spn;
              CAN_FaultRecordBuffer[pos].can_fmi = can_data_fmi;
              
              CAN_FaultRecordBuffer[pos].error_time = CAN_FAULT_TTL;
              
              CAN_FaultBitMap |= ((unsigned long)1 << pos);
              
              if(can_data.can_total_num < CAN_FAULT_RECORD_MAX_NUM)
              {
                can_data.can_total_num += 1;
              }
            }
            else if(-1 == pos)
            {
              //NOP
            }
          }            
        }
      break;
      
      //故障超时清除
      case CAN_FAULT_CLEAR:
        if(CAN_FaultBitMap != 0)
        {	
          for(i = 0; i < CAN_FAULT_RECORD_MAX_NUM; i++)
          {
            if((CAN_FaultRecordBuffer[i].error_time == 0) && ((CAN_FaultRecordBuffer[i].can_spn != 0) || (CAN_FaultRecordBuffer[i].can_fmi != 0)))
            {
              CAN_FaultRecordBuffer[i].sourceAddr = 0;
              CAN_FaultRecordBuffer[i].lamp_status = 0;
              CAN_FaultRecordBuffer[i].flash_lamp_status = 0;
              CAN_FaultRecordBuffer[i].can_spn = 0;
              CAN_FaultRecordBuffer[i].can_fmi = 0;

              CAN_FaultBitMap &= ~((unsigned long)1 << i);
              
              if(can_data.can_total_num != 0)
              {    
                can_data.can_total_num--;
              }
            }
          }
        }
      break;
      
      default:
      break;
    }
}

/************************************************************************************************
@ Func:  GPS报警解析
@ Brief: 除了GPS发送一切正常状态清零CAN_GPSWarningBitsMap外，发送其他信息只置位
         CAN_GPSWarningBitsMap相关位,但是不清零，从而警告信息会一直显示（51和58天报警除外，它们只
         持续1min，即使再次重复发送, 由于第一次收到时CAN_GPSWarningBitsMap相关位已经置位，从而不
         会再次显示报警信息）
************************************************************************************************/
void CAN_GPSWarningProc(void)
{
  //定义为static类型，相当于CAN_GPSWarningBitsMap的前一个状态
  static Uint16_t warning_buff = 0;
  static bool oneMinuteDurationFlag = false;
  static bool firstRecv51DaysWarningFlag = true;
  static bool firstRecv58DaysWarningFlag = true;
  
  CAN_GPSWarningBitsMap = 0;
  
  //判断ECU状态   
  //GPS功能激活，KEY值验证不通过，未锁车
  if((can_data.warning_ecu & 0x07) == 0x01)
  {
    CAN_GPSWarningBitsMap |= EECU_ALARM_KEY_INCORRECT;
  }
  //GPS功能激活，KEY值验证不通过，已锁车
  else if((can_data.warning_ecu & 0x07) == 0x03)
  {
    CAN_GPSWarningBitsMap |= EECU_ALARM_KEY_INCORRECT_LOCKED;
  }
  
  switch(can_data.warning_level)
  {
    case GPS_STATE_WORK_ABNORMAL:
      CAN_GPSWarningBitsMap |= GPS_ALARM_WORK_ABNORMAL;
    break;
    
    case GPS_STATE_51_DAYS_NO_SIGNAL:
      if(firstRecv51DaysWarningFlag)
      {
        firstRecv51DaysWarningFlag = false;
        can_data.durationFor51DaysWarning = 60;
      }
      
      if(can_data.durationFor51DaysWarning)
      {
        CAN_GPSWarningBitsMap |= GPS_ALARM_51_DAYS_NO_SIGNAL;
      }
    break;
    
    case GPS_STATE_58_DAYS_NO_SIGNAL:
      if(firstRecv58DaysWarningFlag)
      {
        firstRecv58DaysWarningFlag = false;
        can_data.durationFor58DaysWarning = 60;
      }
      
      if(can_data.durationFor58DaysWarning)
      {
        CAN_GPSWarningBitsMap |= GPS_ALARM_58_DAYS_NO_SIGNAL;
      }
    break;
    
    case GPS_STATE_61_DAYS_NO_SIGNAL:
      CAN_GPSWarningBitsMap |= GPS_ALARM_61_DAYS_NO_SIGNAL;
    break;
    
    //状态复位
    case GPS_STATE_WORK_WELL:
      firstRecv51DaysWarningFlag = true;
      firstRecv58DaysWarningFlag = true;
    break;
        
    case GPS_STATE_WEB_LOCK_CMD:
      CAN_GPSWarningBitsMap |= GPS_ALARM_WEB_LOCK_CMD;
    break;
    
    default:
    break;
  }
}

/************************************************************************************************
@ Func:  CAN接收数据处理
@ Brief: 在CAN接收中断中将数据存入接收缓存，该函数处理接收缓存中的数据
************************************************************************************************/
unsigned long fuel_test = 0;
void CAN_RecvDataProcess(void)
{
  unsigned char ucTemp0,ucTemp1,ucTemp2,ucTemp3;
  unsigned long ulCANID;
  unsigned char can_mi;
  unsigned char pack_id;
  unsigned long time = 0;
  unsigned long pgn = 0;
  unsigned char i = 0;
  unsigned long total_fuel = 0;
  
  //若CAN接收缓冲区数据接收索引和数据处理索引不一致，则有待处理的数据  
  while(data_rec_point != data_deal_point)
  {
    //超出缓存范围则回卷
    if(data_deal_point >= REC_DATA_NUM)
    {
      data_deal_point = 0;
    }

    ucTemp0 = ucCanRece[data_deal_point][0];
    ucTemp1 = (ucCanRece[data_deal_point][1]&0xE0)|((ucCanRece[data_deal_point][1]&0x07)<<2)|(ucCanRece[data_deal_point][2]>>6);
    ucTemp2 = (ucCanRece[data_deal_point][2]<<2)|(ucCanRece[data_deal_point][3]>>6);
    ucTemp3 = (ucCanRece[data_deal_point][3]<<2)&0xFE;
    ulCANID = ucTemp0;
    ulCANID <<= 8;
    ulCANID |= ucTemp1;
    ulCANID <<= 8;
    ulCANID |= ucTemp2;
    ulCANID <<= 8;
    ulCANID |= ucTemp3;
    ulCANID >>= 3;
    
    switch(ulCANID)
    {
      //发动机转速
      case CAN_ID_ELECTRONIC_ENGINE_CONTROLLER1:
        can_data.can_speed = ucCanRece[data_deal_point][8];
        can_data.can_speed <<= 8;
        can_data.can_speed |= ucCanRece[data_deal_point][7];
        can_data.can_speed = can_data.can_speed / CAN_SPEED_PAR;
        Engine_SpeedSampleBuffer[Engine_SpeedSampleCnt] = can_data.can_speed;
        Engine_SpeedSampleCnt++;
        if(Engine_SpeedSampleCnt >= ENGINE_SPEED_SAMPLE_NUM)
        {
            Engine_SpeedSampleCnt = 0;
            systemTask.b.Pulse_active = TRUE;
        }
        CAN_EngineSpeedRecvTimeout = CAN_RECV_TIMEOUT;
      break;
      
      //预热
      case CAN_ID_PREHEAT:
        can_data.can_preheat = ucCanRece[data_deal_point][7] & 0x03;
      break;
      
      //手刹
      /*case CAN_ID_HANDBRAKE:
        can_data.handbrake = ucCanRece[data_deal_point][4] & 0x01;
      break;*/
      
      case CAN_ID_ENGINE_TEMPERATURE:
        //水温
        can_data.currWaterTemp = ucCanRece[data_deal_point][4];
        if(can_data.currWaterTemp >= 80)
        {
          can_data.currWaterTemp -= 40;
        }
        else
        {
          can_data.currWaterTemp = 40;
        }
        
        if(can_data.currWaterTemp != can_data.oldWaterTemp)
        {
          can_data.oldWaterTemp = can_data.currWaterTemp;
          systemTask.b.coolingLiquidTempProcessFlag = true;
        }
        
        CAN_CoolingLiquidTempRecvTimeout = CAN_RECV_TIMEOUT;
        
        //机油温度
        can_data.oil_temp = ucCanRece[data_deal_point][7];
        can_data.oil_temp <<= 8;
        can_data.oil_temp |= ucCanRece[data_deal_point][6];
        can_data.oil_temp = can_data.oil_temp / 32;
        if(can_data.oil_temp >= 273)
        {
            can_data.oil_temp -= 273;
        } 
        else
        {
            can_data.oil_temp = 0;
        }
      break;
      
      //发动机时间
      case CAN_ID_ENGINE_HOURS:
    		//0.05hr/bit，即3min/bit
    		time = (ucCanRece[data_deal_point][4] + ((unsigned long)ucCanRece[data_deal_point][5] << 8) 
    		       + ((unsigned long)ucCanRece[data_deal_point][6] << 16) + ((unsigned long)ucCanRece[data_deal_point][7] << 24));
    		//换算为以6min为单位
    		time /= 2;
    		can_data.engineWorkingTime = time;
    		if(can_data.engineWorkingTime > 999999)
    		{
    		  can_data.engineWorkingTime = 999999;
    		}
    		working_time = time;
    		working_time_h = (unsigned int)((time >> 16) & 0xffff);
    		working_time_l = (unsigned int)(time & 0xffff);
    		
    		if(!CAN_RecvEngineHourReqAckFlag)
    		{
      		CAN_RecvEngineHourReqAckFlag = true;
    		}
      break;
      
      //发动机总油耗
      case CAN_ID_ENGINE_OIL_USAGE:
        total_fuel =  ucCanRece[data_deal_point][11];
			  total_fuel <<= 8;
		    total_fuel |= ucCanRece[data_deal_point][10];
			  total_fuel <<= 8;
		    total_fuel |= ucCanRece[data_deal_point][9];
	      total_fuel <<= 8;
		    total_fuel |= ucCanRece[data_deal_point][8];			    			    
		    total_fuel /=  2;
		    fuel_test = total_fuel;
		    if(working_time == 0)
		      can_data.fuelConsumption = 0;			    
		    else 
		      can_data.fuelConsumption = ( ((total_fuel*10*100 / working_time)+5)/10 ) ;       //total_fuel *10 / working_time
			    break;
      break;
    
      //燃油含水指示、NCD
      case CAN_ID_WATER_IN_FUEL_INDICATOR:
        can_data.fuel_water = ucCanRece[data_deal_point][4] & 0x03;
        if (TYPE_of_Engine == Weichai_engine)
        {
          CAN_PostprocessingState.ncdState = ucCanRece[data_deal_point][5];
        }
      break;

      //再生 & 再生抑制（潍柴和玉柴都是用这一个CAN id）进行兼容潍柴和玉柴by徐光同-2021/4/16
      case CAN_ID_DPFC1:                       
      if (TYPE_of_Engine == Weichai_engine)
        {
          CAN_PostprocessingState.regenerateState = ucCanRece[data_deal_point][4] & 0x07;
          CAN_PostprocessingState.regenerateInhibitState = ucCanRece[data_deal_point][6] & 0x03;
        }
      else
        {
          CAN_PostprocessingState_YC.regenerateRemind= ucCanRece[data_deal_point][4]& 0x07;
          CAN_PostprocessingState_YC.regenerateProcess=ucCanRece[data_deal_point][5]& 0x0C;
          CAN_PostprocessingState_YC.regenerateInhibitState=ucCanRece[data_deal_point][6]& 0x0C; 
          CAN_PostprocessingState_YC.exhaust_high_temperature=ucCanRece[data_deal_point][10]& 0x1C;              
        }
      break;
      
      //尿素液位（潍柴发动机）
      case CAN_ID_UREA_LEVEL_1:
        can_data.currUreaLevel = ucCanRece[data_deal_point][4] * 0.4;
        NoconversionUreaLevel  = ucCanRece[data_deal_point][4];
        if((can_data.currUreaLevel != can_data.oldUreaLevel) || Meter_UreaLevelFirstProcessFlag)
        {
          can_data.oldUreaLevel = can_data.currUreaLevel;
          systemTask.b.ureaLevelProcessFlag = true;
        }
        //此处为如果识别到玉柴发动机采用EGR模式，即使有尿素信号也强制清零，使指针强制保持在零刻度。
        if (TYPE_of_Engine == Yuchai_engine)
        {
          if (EN_YC_Urea_Function == false)
          {
            can_data.currUreaLevel = 0X00;
            can_data.oldUreaLevel = 0X00;
          }
        }        
        CAN_UreaLevelRecvTimeout = CAN_RECV_TIMEOUT;
      break;
      /*
       //尿素液位（玉柴发动机）   因为一个发动机只对应一个尿素液位的CANid，故尿素液位处理的信息是一样的
      case CAN_ID_UREA_LEVEL_2:
        can_data.currUreaLevel = ucCanRece[data_deal_point][4] * 0.4;
        if ((can_data.currUreaLevel != can_data.oldUreaLevel) || Meter_UreaLevelFirstProcessFlag)
        {
          can_data.oldUreaLevel = can_data.currUreaLevel;
          systemTask.b.ureaLevelProcessFlag = true;
        }
        CAN_UreaLevelRecvTimeout = CAN_RECV_TIMEOUT;
      break;
      */
      //大气温度
      case CAN_ID_AMB:
        can_data.ambientAirTemp = (ucCanRece[data_deal_point][7] + ((unsigned int)ucCanRece[data_deal_point][8] << 8)) * 0.03125 - 273;
      break;
      
      //燃油消耗
      case CAN_ID_LFE:
        can_data.engineFuelRate = (ucCanRece[data_deal_point][4] + ((unsigned int)ucCanRece[data_deal_point][5] << 8)) * 0.05;
      break;
      
      //GPS状态
      case CAN_ID_GPS_INFO:
        can_data.warning_level = ucCanRece[data_deal_point][4];
      break;
      
      //EECU
      case CAN_ID_EECU_INFO:
        can_data.warning_ecu = ucCanRece[data_deal_point][6];
      break;
      
      //单帧故障
      case CAN_ID_DM1:
        CAN_SingleFault.sourceAddr = ulCANID & 0xff;
        
        CAN_SingleFault.lamp_status = ucCanRece[data_deal_point][4];        
        CAN_SingleFault.flash_lamp_status = ucCanRece[data_deal_point][5];

        CAN_SingleFault.can_fmi = ucCanRece[data_deal_point][8] & 0x1f;
        
        CAN_SingleFault.can_spn = ucCanRece[data_deal_point][8] >> 5;
        CAN_SingleFault.can_spn <<= 8;
        CAN_SingleFault.can_spn |= ucCanRece[data_deal_point][7];
        CAN_SingleFault.can_spn <<= 8;
        CAN_SingleFault.can_spn |= ucCanRece[data_deal_point][6];
        
        //单包故障信息处理
        CAN_FaultProcess(CAN_SINGLE_FAULT_PROC);      
      break;
      
      //多帧故障广播
      case CAN_ID_DM1_DM2_BAM:
        //获取PGN
        can_data.sourceAddr = ulCANID & 0xff;
    	  can_data.pgn_id = 0;
    	  can_data.pgn_id = ucCanRece[data_deal_point][9];
    	  can_data.pgn_id <<= 8;
    	  can_data.pgn_id |= ucCanRece[data_deal_point][10];
    	  can_data.pgn_id <<= 8;
    	  can_data.pgn_id |= ucCanRece[data_deal_point][11];
    	  
    	  if(can_data.pgn_id == 0x00CAFE00)
    	  {
  	      //总字节数
  	      can_data.multi_bag_len = ucCanRece[data_deal_point][6];
  	      can_data.multi_bag_len <<= 8;
  	      can_data.multi_bag_len |= ucCanRece[data_deal_point][5];

          //总包数
  	      can_data.multi_bag_num = ucCanRece[data_deal_point][7];
  	      
  	      //接收字节数置零，准备接收
  	      can_data.multi_bag_time = 0;
    	  }
    	  else
    	  {
  	      can_data.multi_bag_len = 0;
  	      can_data.multi_bag_num = 0;
    	  }
      break;
      
      //多帧故障数据
      case CAN_ID_DM1_DM2_PACKET:
    	  if(can_data.multi_bag_len != 0)
    	  {
  	      pack_id = ucCanRece[data_deal_point][4];
  	      for(can_mi = 0; can_mi < 7; can_mi++)
  	      {
	           can_data.multi_bag_data[(pack_id - 1) * 7 + can_mi] = ucCanRece[data_deal_point][5 + can_mi];
	           
	           //已接收字节数，考虑到一包数据可能被重复发送，不能采用“++can_data.multi_bag_time”的方式
	           can_data.multi_bag_time = (pack_id - 1) * 7 + can_mi + 1;
	           
	           if(can_data.multi_bag_time >= can_data.multi_bag_len)
	           {
	              //多包故障信息处理
	               CAN_FaultProcess(CAN_ECU_MULTI_FAULT_PROC);      
	               can_data.multi_bag_len = 0;
	               can_data.multi_bag_time = 0;
	               break;
	           }
  	      }
    	  }
      break;
      
      //GPS升级命令
      case CAN_ID_HOST_UPDATE_CMD:
        for(can_mi = 0; can_mi < 8; ++can_mi)
        {
          Update_CmdStoreUnit.inforByteMap.ucBuffer[can_mi] = ucCanRece[data_deal_point][4 + can_mi]; 
        }
        
        if(0x02061102 == *(unsigned long *)Update_CmdStoreUnit.inforByteMap.ucBuffer)
        {
          updateCmdFlag = true;
        } 
        
      break;
        //发动机类型判断,只有当判断到玉柴发动机是采用EGR、还是SCR的模式后，才改为玉柴发动机模式，否则一直默认为潍柴发动机
        //by徐光同-2021/11/13
        //在接收到该ID后，对接收到的再生信息只能在下个循环周期起作用
      case CAN_ID_ENGINE_TYPE:
        {
          if (TYPE_of_Engine == Weichai_engine)
          {
            if( YC_SCR_MODE  ==  ucCanRece[data_deal_point][4])
            {
              TYPE_of_Engine = Yuchai_engine;
              CAN_PostprocessingState.regenerateState = 0x00;            //清除潍柴状态下缓存的数据
              CAN_PostprocessingState.regenerateInhibitState = 0x00;     //清除潍柴状态下缓存的数据
              CAN_PostprocessingState.ncdState=0x00;                     //清除潍柴状态下缓存的数据
            }
            else if (YC_EGR_MODE ==  ucCanRece[data_deal_point][4])
            {
              TYPE_of_Engine = Yuchai_engine;
              CAN_PostprocessingState.regenerateState = 0x00;            //清除潍柴状态下缓存的数据
              CAN_PostprocessingState.regenerateInhibitState = 0x00;     //清除潍柴状态下缓存的数据
              CAN_PostprocessingState.ncdState=0x00;                     //清除潍柴状态下缓存的数据
              EN_YC_Urea_Function = false;                               //不使能玉柴的尿素功能，指针为0，NCD灯不闪烁
              can_data.oldUreaLevel = 0 ;
              can_data.currUreaLevel = 0;
            }
          }
        }
      break;
      default:
      break;
    }
 
      data_deal_point++;
  }
}

/************************************************************************************************
@ Func:  CAN相关任务处理
************************************************************************************************/
void CAN_TaskProcess(void)
{
  int pos = 0;
  unsigned int i = 0;
  
  //CAN接收数据处理
  CAN_RecvDataProcess();
  
  //执行频率较低的任务
  if(0 == can_task.can_time4)
  {
    can_task.can_time4 = 1500;
    
		CAN_FaultProcess(CAN_FAULT_CLEAR);              //故障消除处理
	  CAN_GPSWarningProc();                           //GPS报警处理
	  if(CAN_FaultBitMap)                             //查找是否有机油压力低故障
    {
      //查找机油压力低故障条目
      pos = CAN_SearchSpecificFault(100, 17); 
      if(pos >= 0)
      {
        CAN_AlarmFlag.engineOilPressureLowFlag = true;
      }
      else if(-1 == pos)
      {
        CAN_AlarmFlag.engineOilPressureLowFlag = false;
      }
    }
    else
    {
      CAN_AlarmFlag.engineOilPressureLowFlag = false;
    }
  }
  
  //请求发动机时间
	if(0 == CAN_ReqWorkingTimeCnt)
	{
	  CAN_ReqWorkingTimeCnt = 5000;
	  
	  Can_Tx_Frame(3, MESSAGE_EXTENDED_FORMAT, 0x18EA0021, CAN_WorkingTimePGN);
	}
	
  //新增发动机总油耗请求 
	if(0 ==  CAN_ReqOilCnt) 
	{
	  CAN_ReqOilCnt = 1000;
	  Can_Tx_Frame(3, MESSAGE_EXTENDED_FORMAT, 0x18EA0021, Can_OilUsagePGN);
	}
  //發送小時計與LED狀態
  if(0 == can_task.can_time0)
  {
    can_task.can_time0 = 1000;
    
    //小时计 
    ucCanSend0[0] = working_time_h >> 8;          
    ucCanSend0[1] = working_time_h;
    ucCanSend0[2] = working_time_l >> 8;
    ucCanSend0[3] = working_time_l;

    //LED状态
    ucCanSend0[4] = (LED_OnOffBitsMap >> 24) & 0xff;             
    ucCanSend0[5] = (LED_OnOffBitsMap >>16) & 0xff;
    ucCanSend0[6] = (LED_OnOffBitsMap >>8) & 0xff;
    ucCanSend0[7] = LED_OnOffBitsMap & 0xff;
    
    //左右转向灯额外处理
    if(digital_status.new_status & DIGITAL_LEFT_TURN_BIT)
    {
      ucCanSend0[7] |= LED_LEFT;
    }
    else
    {
      ucCanSend0[7] &= ~LED_LEFT;
    }
    
    if(digital_status.new_status & DIGITAL_RIGHT_TURN_BIT)
    {
      ucCanSend0[7] |= LED_RIGHT;
    }
    else
    {
      ucCanSend0[7] &= ~LED_RIGHT;
    }

    
    Can_Tx_Frame(8,MESSAGE_EXTENDED_FORMAT,0x1fff5252,ucCanSend0);
  }
  
  //發送油位、水溫、油溫、轉速
  if(0 == can_task.can_time1)
  {
    can_task.can_time1 = 1000;
    
    ucCanSend1[0] = 0;                                                          //设备類型，T4不再使用該字節
    ucCanSend1[1] = 0;                                                          //无气压值
    ucCanSend1[2] = mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_convert_value / 10;  //油位值
    ucCanSend1[3] = can_data.currWaterTemp;                                     //水温  
    ucCanSend1[4] = mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_convert_value;   //油温 
    ucCanSend1[5] = (engineSpeedParams.new_speed >> 8) & 0xff;                  //转速高字节
    ucCanSend1[6] = engineSpeedParams.new_speed & 0xff;                         //转速低字节
    ucCanSend1[7] = 0;                                                          //气压表2，无
    
    Can_Tx_Frame(8,MESSAGE_EXTENDED_FORMAT,0x1fff5352,ucCanSend1);
  }
  
  //發送設備信息
  if(0 == can_task.can_time3)
  {
    can_task.can_time3 = 10000;
    
    Can_Tx_Frame(8, MESSAGE_EXTENDED_FORMAT, 0X18FF5652, CAN_DeviceInfoBuffer);
  }
  
  //發送尿素液位
  if(0 == can_task.can_time2)
  {
    can_task.can_time2 = 1000;
    
    CAN_UreaLevelBuffer[0] = can_data.currUreaLevel / 0.4 + 0.5;
    //CAN_UreaLevelBuffer[4] = can_data.can_speed & 0xff;
    //CAN_UreaLevelBuffer[5] = can_data.can_speed >> 8;
    
    Can_Tx_Frame(8, MESSAGE_EXTENDED_FORMAT, 0x18FF5752, CAN_UreaLevelBuffer);
  }
}
  
/*******************************************************************************
* Function		: mscan_rx_isr
* Description	: 中断服务程序：将接收数据存储到数组中
*******************************************************************************/
#pragma CODE_SEG NON_BANKED
void interrupt 38 mscan_rx_isr(void) 
{
  unsigned char ucI = 0;
  unsigned char *addr = NULL;    

  //关闭中断
  asm("sei");  

  addr = (unsigned char *)(&CANRXF); 
  
  if(data_rec_point >= REC_DATA_NUM)
  {
    data_rec_point = 0;
  }

  for(ucI = 0; ucI < 12; ucI++) 
  {
    ucCanRece[data_rec_point][ucI] = addr[ucI];
  }

  data_rec_point++;

  //clear receive interrupt flag
  CANRFLG |= 0x01; 

  //打开中断
  asm("cli");
}
#pragma CODE_SEG DEFAULT
