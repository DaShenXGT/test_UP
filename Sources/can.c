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

//CAN�������ݴ洢���������
unsigned char ucCanRece[REC_DATA_NUM][12];                      //CAN���ջ���
unsigned char data_rec_point;                                   //CAN���ջ������ݴ洢����
unsigned char data_deal_point;                                  //CAN���ջ������ݴ�������

//CAN�������
CAN_Fault_t CAN_FaultRecordBuffer[CAN_FAULT_RECORD_MAX_NUM];    //CAN���ϼ�¼����
Uint32_t CAN_FaultBitMap;                                       //��Ӧ�洢������Ŀ��CAN���ջ���λͼ 
CAN_Fault_t CAN_SingleFault;                                    //��ǰ������Ϣ

//�洢ˮ�¡�����Һλ��CAN��Ϣ
CAN_RecvData_t  can_data;

//Ϋ�񷢶����������
CAN_PostprocessingState_t CAN_PostprocessingState =
{
  NCD_STATE_INACTIVE,
  0,
  REGENERATE_STATE_NONE,
  REGENERATE_INHIBIT_STATE_NONE,
  0,
};
//��񷢶����������
CAN_PostprocessingState_YC_t CAN_PostprocessingState_YC =
{
  REGENERATE_STATE_NONE,
  REGENERATE_STATE_NONE,
  REGENERATE_INHIBIT_STATE_NONE,
  0x00,
};
//������ʱ��
unsigned int CAN_ReqWorkingTimeCnt = 0;                         //����ʱ�������
const unsigned char CAN_WorkingTimePGN[3]= {0xE5, 0xFE, 0x00};  //������ʱ�䣬PGN 0x00FEE5��Sent least significant byte first 
bool CAN_RecvEngineHourReqAckFlag = false;                      //ECU����ʱ���־
unsigned int CAN_RecvEngineHourReqAckTimeout = 0;               //ʱ������ʱ������


//���������ͺ�
unsigned int CAN_ReqOilCnt = 0;                                 //����ȼ������������
const unsigned char Can_OilUsagePGN[3] = {0xE9,0xFE,0x00};      //���������ͺģ�PGN 0x00FEE9

//������ת�ٽ��ճ�ʱ������
unsigned int CAN_EngineSpeedRecvTimeout = CAN_RECV_TIMEOUT; 
unsigned int CAN_CoolingLiquidTempRecvTimeout = CAN_RECV_TIMEOUT;
unsigned int CAN_UreaLevelRecvTimeout = CAN_RECV_TIMEOUT;

//CAN������־
CAN_AlarmFlag_t CAN_AlarmFlag =
{
  false,     //���������ϱ�־
  false,     //������ϱ�־
  false,     //����ѹ���͹��ϱ�־
  false,     //������غ������
};

//�������ݻ�����
unsigned char ucCanSend0[8];                    //0x1fff5252   ��1~4�ֽ���Сʱ��  ��5~8   LED״̬       //liu~                     //0x1bf00417����1�ֽ���λ��ת��;4���ֽڣ�5���ֽ�
unsigned char ucCanSend1[8];                    //0x1fff5352   1 �豸����	2 ��ѹֵ	3 ��λ	4 ˮ��	5 ����	6~7 ת��            //0x03F00517 ������+�ۼƹ���ʱ��
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

//���������� ������Ĭ������ΪΫ�񷢶�����
TYPE_of_Engine_t TYPE_of_Engine = Weichai_engine;
unsigned char NoconversionUreaLevel=0;
unsigned char EN_YC_Urea_Function = true ;

/*******************************************************************************
@ Func: CANģ���ʼ��
*******************************************************************************/
void Mscan_Initial(void) 
{
    //STB �õ�,TJA1040��������
    DDRP  |= (DDRP_DDRP1_MASK);
    PTP   &= (~PTP_PTP1_MASK); 
    
    CANCTL1 |= (1 << CANE);                 //mscan module enable   

    CANCTL0 |= (1 << INITRQ);               //MSCAN in initialization mode

    while ((CANCTL1 & (1<<INITACK)) == 0)   //-�ȴ������ʼ��ģʽ[5/28/2013 rookie li]
    {
    }

    CANCTL0 |= (CANCTL0_TIME_MASK);	      	//-Enable internal MSCAN time[5/28/2013 rookie li]

//	CANCTL1 |= CLKSRC;
    CANCTL1 &= ~(1 << CLKSRC);              //external crystal for mscan clock   

    //set can bus bandrate
    Can_conf_bandrate();     

    //!  self loop for test mscan module  
    //CANCTL1 |= (1 << LOOPB);              //�ػ��Բ�ģʽ

    CANCTL1 &= ~(1 << LISTEN);              //normal operation mode

    //!  accept filter register    
    CANIDAC = CAN0IDACV;                    //����32λ���չ�����?

    CANIDAR0 = CAN0IDAR0V; 
    CANIDAR1 = CAN0IDAR1V;
    CANIDAR2 = CAN0IDAR2V; 
    CANIDAR3 = CAN0IDAR3V;                  //Ϊʲôֻ����ǰ4����[5/28/2013 rookie li]
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
    CANTIER = CAN0TIERV;  //set interrupt modle��No interrupt request is generated from this event

    CANCTL0 &= ~(1<<INITRQ);   //INITRQ=0��exit initial

    //����Ӧ����&
    while ((CANCTL1 & (1<<INITACK)) == 1)//wait for ack
    {
    }
    
    while(!(CANCTL0_SYNCH));		// wait for CAN module to synch 

    CANRFLG = 0xFF;			//-����Ĵ���[5/28/2013 rookie li]

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
@ Func: CANͨ�Ų�����ʼ��
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
* Description	: Initialises a send data box���Զ�Ѱ�ҿ���MOB;
* Called By		: main()
* Input	 :	
*			ucNoBytes        Number of data bytes. Permissible values 0 .. 8
*			ucXtdFormat		   MESSAGE_EXTENDED_FORMAT or MESSAGE_BASIC_FORMAT
*			iCANId           CAN message identifier 
*     pDatapointer     ָ��������
* Output :  
* Return : ���ͳɹ�����1��û�п���buffer����0
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
        ucTemp1		= ((ucTemp0>>2)&0x07);	//��֤����λΪID17-ID15
        ucTemp2		= (ucTemp0 & 0xE0);		//��֤����λΪID20-ID18
        CANTXIDR1	= (ucTemp1|ucTemp2|CANTXIDR1_SRR_MASK|CANTXIDR1_IDE_MASK);
        ulCANid		>>= 2;
        CANTXIDR2	= (ulCANid>>8);
        CANTXIDR3	= ulCANid;				/*-����֡[5/29/2013 rookie li]-*/
    }
    else
    {
        /*-��׼��ʽ[5/29/2013 rookie li]-*/
        ulCANid <<= 5;
        CANTXIDR0	= (ulCANid>>8);
        CANTXIDR1	= (ulCANid);
        /*-bit3=0:0 Standard format[5/29/2013 rookie li]-*/
        /*-bit4=0:0 Data frame[5/29/2013 rookie li]-*/
        CANTXIDR1	&= 0xE0;
    }

    /*-�����ֽڸ����ɿ���[5/29/2013 rookie li]-*/
    for(ucIndex=0;ucIndex<ucNoBytes;ucIndex++) 
    {
        *(&CANTXDSR0+ucIndex) = pDatapointer[ucIndex];	  
    }
    can_task.can_wait_time = 400;
    CANTXDLR	= ucNoBytes;

    CANTFLG = BufNum; //���ͻ�����0׼������  write of 1 clears flag
//	while((CANTFLG&BufNum) != BufNum);/*-Wait for Transmission completion-*/
    while(((CANTFLG&BufNum) != BufNum) && can_task.can_wait_time);/*-Wait for Transmission completion-*/
    

    //Delay_N_Nop(800) ;

    //return (Can_Tx_Data_Frames->Tx_Flag);  
    return(1);        
}

/************************************************************************************************
@ Func:  �����ض��������ڹ�����������
@ Param: spn <--> DTC SPN
         fmi <--> DTC FMI
@ Ret:   -1   <--> δ�ҵ��ù���
         ���� <--> �ù�������
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
@ Func:  ����ض�������Ϣ
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
 * Func��  ͳ�ƹ�������
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
@ Func:  CAN���ϴ���
@ Param: method <-> ���ϴ���ʽ����֡���ϡ�ECU��֡���ϡ�DCU��֡���ϡ��������
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
      //��֡���ϴ���
      case CAN_SINGLE_FAULT_PROC:
        //���Ҹù����Ƿ��Ѵ���
        pos = CAN_SearchSpecificFault(CAN_SingleFault.can_spn, CAN_SingleFault.can_fmi);
        //���Ѵ�����ֻˢ�´���ʱ��
        if(pos >= 0)
        {
          CAN_FaultRecordBuffer[pos].error_time = CAN_FAULT_TTL;
          //ָʾ��״̬�ı������
          if(CAN_FaultRecordBuffer[pos].lamp_status != CAN_SingleFault.lamp_status)
          {
            CAN_FaultRecordBuffer[pos].lamp_status = CAN_SingleFault.lamp_status;
          }
          //��˸ָʾ��״̬
          if(CAN_FaultRecordBuffer[pos].flash_lamp_status != CAN_SingleFault.flash_lamp_status)
          {
            CAN_FaultRecordBuffer[pos].flash_lamp_status = CAN_SingleFault.flash_lamp_status;
          }
        }
        //Ϊ�·����������¼֮
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
      
      //ECU��֡���ϴ���
      case CAN_ECU_MULTI_FAULT_PROC:
        lampStatus = can_data.multi_bag_data[0];
        flashlampStatus = can_data.multi_bag_data[1];
        for(i = 2; i < can_data.multi_bag_len; i++)
        {
          //��ȡSPN
          can_data_spn = can_data.multi_bag_data[i++];
          can_spn_buff = can_data.multi_bag_data[i++];
          can_spn_buff <<= 8;
          can_data_spn |= can_spn_buff;
          can_spn_buff = can_data.multi_bag_data[i] >> 5;
          can_spn_buff <<= 16;
          can_data_spn |= can_spn_buff;
          
          //��ȡFMI
          can_data_fmi = can_data.multi_bag_data[i] & 0x1f;
          
          //����[CM OC]�ֽ�
          ++i;
            
          pos = CAN_SearchSpecificFault(can_data_spn, can_data_fmi);
          if(pos >= 0)
          {
            CAN_FaultRecordBuffer[pos].error_time = CAN_FAULT_TTL;
            //ָʾ��״̬�ı������
            if(CAN_FaultRecordBuffer[pos].lamp_status != lampStatus/*CAN_SingleFault.lamp_status*/)
            {
              CAN_FaultRecordBuffer[pos].lamp_status = lampStatus/*CAN_SingleFault.lamp_status*/;
            }
            //��˸ָʾ��״̬ 
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
      
      //���ϳ�ʱ���
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
@ Func:  GPS��������
@ Brief: ����GPS����һ������״̬����CAN_GPSWarningBitsMap�⣬����������Ϣֻ��λ
         CAN_GPSWarningBitsMap���λ,���ǲ����㣬�Ӷ�������Ϣ��һֱ��ʾ��51��58�챨�����⣬����ֻ
         ����1min����ʹ�ٴ��ظ�����, ���ڵ�һ���յ�ʱCAN_GPSWarningBitsMap���λ�Ѿ���λ���Ӷ���
         ���ٴ���ʾ������Ϣ��
************************************************************************************************/
void CAN_GPSWarningProc(void)
{
  //����Ϊstatic���ͣ��൱��CAN_GPSWarningBitsMap��ǰһ��״̬
  static Uint16_t warning_buff = 0;
  static bool oneMinuteDurationFlag = false;
  static bool firstRecv51DaysWarningFlag = true;
  static bool firstRecv58DaysWarningFlag = true;
  
  CAN_GPSWarningBitsMap = 0;
  
  //�ж�ECU״̬   
  //GPS���ܼ��KEYֵ��֤��ͨ����δ����
  if((can_data.warning_ecu & 0x07) == 0x01)
  {
    CAN_GPSWarningBitsMap |= EECU_ALARM_KEY_INCORRECT;
  }
  //GPS���ܼ��KEYֵ��֤��ͨ����������
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
    
    //״̬��λ
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
@ Func:  CAN�������ݴ���
@ Brief: ��CAN�����ж��н����ݴ�����ջ��棬�ú���������ջ����е�����
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
  
  //��CAN���ջ��������ݽ������������ݴ���������һ�£����д����������  
  while(data_rec_point != data_deal_point)
  {
    //�������淶Χ��ؾ�
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
      //������ת��
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
      
      //Ԥ��
      case CAN_ID_PREHEAT:
        can_data.can_preheat = ucCanRece[data_deal_point][7] & 0x03;
      break;
      
      //��ɲ
      /*case CAN_ID_HANDBRAKE:
        can_data.handbrake = ucCanRece[data_deal_point][4] & 0x01;
      break;*/
      
      case CAN_ID_ENGINE_TEMPERATURE:
        //ˮ��
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
        
        //�����¶�
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
      
      //������ʱ��
      case CAN_ID_ENGINE_HOURS:
    		//0.05hr/bit����3min/bit
    		time = (ucCanRece[data_deal_point][4] + ((unsigned long)ucCanRece[data_deal_point][5] << 8) 
    		       + ((unsigned long)ucCanRece[data_deal_point][6] << 16) + ((unsigned long)ucCanRece[data_deal_point][7] << 24));
    		//����Ϊ��6minΪ��λ
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
      
      //���������ͺ�
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
    
      //ȼ�ͺ�ˮָʾ��NCD
      case CAN_ID_WATER_IN_FUEL_INDICATOR:
        can_data.fuel_water = ucCanRece[data_deal_point][4] & 0x03;
        if (TYPE_of_Engine == Weichai_engine)
        {
          CAN_PostprocessingState.ncdState = ucCanRece[data_deal_point][5];
        }
      break;

      //���� & �������ƣ�Ϋ������������һ��CAN id�����м���Ϋ������by���ͬ-2021/4/16
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
      
      //����Һλ��Ϋ�񷢶�����
      case CAN_ID_UREA_LEVEL_1:
        can_data.currUreaLevel = ucCanRece[data_deal_point][4] * 0.4;
        NoconversionUreaLevel  = ucCanRece[data_deal_point][4];
        if((can_data.currUreaLevel != can_data.oldUreaLevel) || Meter_UreaLevelFirstProcessFlag)
        {
          can_data.oldUreaLevel = can_data.currUreaLevel;
          systemTask.b.ureaLevelProcessFlag = true;
        }
        //�˴�Ϊ���ʶ����񷢶�������EGRģʽ����ʹ�������ź�Ҳǿ�����㣬ʹָ��ǿ�Ʊ�������̶ȡ�
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
       //����Һλ����񷢶�����   ��Ϊһ��������ֻ��Ӧһ������Һλ��CANid��������Һλ�������Ϣ��һ����
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
      //�����¶�
      case CAN_ID_AMB:
        can_data.ambientAirTemp = (ucCanRece[data_deal_point][7] + ((unsigned int)ucCanRece[data_deal_point][8] << 8)) * 0.03125 - 273;
      break;
      
      //ȼ������
      case CAN_ID_LFE:
        can_data.engineFuelRate = (ucCanRece[data_deal_point][4] + ((unsigned int)ucCanRece[data_deal_point][5] << 8)) * 0.05;
      break;
      
      //GPS״̬
      case CAN_ID_GPS_INFO:
        can_data.warning_level = ucCanRece[data_deal_point][4];
      break;
      
      //EECU
      case CAN_ID_EECU_INFO:
        can_data.warning_ecu = ucCanRece[data_deal_point][6];
      break;
      
      //��֡����
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
        
        //����������Ϣ����
        CAN_FaultProcess(CAN_SINGLE_FAULT_PROC);      
      break;
      
      //��֡���Ϲ㲥
      case CAN_ID_DM1_DM2_BAM:
        //��ȡPGN
        can_data.sourceAddr = ulCANID & 0xff;
    	  can_data.pgn_id = 0;
    	  can_data.pgn_id = ucCanRece[data_deal_point][9];
    	  can_data.pgn_id <<= 8;
    	  can_data.pgn_id |= ucCanRece[data_deal_point][10];
    	  can_data.pgn_id <<= 8;
    	  can_data.pgn_id |= ucCanRece[data_deal_point][11];
    	  
    	  if(can_data.pgn_id == 0x00CAFE00)
    	  {
  	      //���ֽ���
  	      can_data.multi_bag_len = ucCanRece[data_deal_point][6];
  	      can_data.multi_bag_len <<= 8;
  	      can_data.multi_bag_len |= ucCanRece[data_deal_point][5];

          //�ܰ���
  	      can_data.multi_bag_num = ucCanRece[data_deal_point][7];
  	      
  	      //�����ֽ������㣬׼������
  	      can_data.multi_bag_time = 0;
    	  }
    	  else
    	  {
  	      can_data.multi_bag_len = 0;
  	      can_data.multi_bag_num = 0;
    	  }
      break;
      
      //��֡��������
      case CAN_ID_DM1_DM2_PACKET:
    	  if(can_data.multi_bag_len != 0)
    	  {
  	      pack_id = ucCanRece[data_deal_point][4];
  	      for(can_mi = 0; can_mi < 7; can_mi++)
  	      {
	           can_data.multi_bag_data[(pack_id - 1) * 7 + can_mi] = ucCanRece[data_deal_point][5 + can_mi];
	           
	           //�ѽ����ֽ��������ǵ�һ�����ݿ��ܱ��ظ����ͣ����ܲ��á�++can_data.multi_bag_time���ķ�ʽ
	           can_data.multi_bag_time = (pack_id - 1) * 7 + can_mi + 1;
	           
	           if(can_data.multi_bag_time >= can_data.multi_bag_len)
	           {
	              //���������Ϣ����
	               CAN_FaultProcess(CAN_ECU_MULTI_FAULT_PROC);      
	               can_data.multi_bag_len = 0;
	               can_data.multi_bag_time = 0;
	               break;
	           }
  	      }
    	  }
      break;
      
      //GPS��������
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
        //�����������ж�,ֻ�е��жϵ���񷢶����ǲ���EGR������SCR��ģʽ�󣬲Ÿ�Ϊ��񷢶���ģʽ������һֱĬ��ΪΫ�񷢶���
        //by���ͬ-2021/11/13
        //�ڽ��յ���ID�󣬶Խ��յ���������Ϣֻ�����¸�ѭ������������
      case CAN_ID_ENGINE_TYPE:
        {
          if (TYPE_of_Engine == Weichai_engine)
          {
            if( YC_SCR_MODE  ==  ucCanRece[data_deal_point][4])
            {
              TYPE_of_Engine = Yuchai_engine;
              CAN_PostprocessingState.regenerateState = 0x00;            //���Ϋ��״̬�»��������
              CAN_PostprocessingState.regenerateInhibitState = 0x00;     //���Ϋ��״̬�»��������
              CAN_PostprocessingState.ncdState=0x00;                     //���Ϋ��״̬�»��������
            }
            else if (YC_EGR_MODE ==  ucCanRece[data_deal_point][4])
            {
              TYPE_of_Engine = Yuchai_engine;
              CAN_PostprocessingState.regenerateState = 0x00;            //���Ϋ��״̬�»��������
              CAN_PostprocessingState.regenerateInhibitState = 0x00;     //���Ϋ��״̬�»��������
              CAN_PostprocessingState.ncdState=0x00;                     //���Ϋ��״̬�»��������
              EN_YC_Urea_Function = false;                               //��ʹ���������ع��ܣ�ָ��Ϊ0��NCD�Ʋ���˸
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
@ Func:  CAN���������
************************************************************************************************/
void CAN_TaskProcess(void)
{
  int pos = 0;
  unsigned int i = 0;
  
  //CAN�������ݴ���
  CAN_RecvDataProcess();
  
  //ִ��Ƶ�ʽϵ͵�����
  if(0 == can_task.can_time4)
  {
    can_task.can_time4 = 1500;
    
		CAN_FaultProcess(CAN_FAULT_CLEAR);              //������������
	  CAN_GPSWarningProc();                           //GPS��������
	  if(CAN_FaultBitMap)                             //�����Ƿ��л���ѹ���͹���
    {
      //���һ���ѹ���͹�����Ŀ
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
  
  //���󷢶���ʱ��
	if(0 == CAN_ReqWorkingTimeCnt)
	{
	  CAN_ReqWorkingTimeCnt = 5000;
	  
	  Can_Tx_Frame(3, MESSAGE_EXTENDED_FORMAT, 0x18EA0021, CAN_WorkingTimePGN);
	}
	
  //�������������ͺ����� 
	if(0 ==  CAN_ReqOilCnt) 
	{
	  CAN_ReqOilCnt = 1000;
	  Can_Tx_Frame(3, MESSAGE_EXTENDED_FORMAT, 0x18EA0021, Can_OilUsagePGN);
	}
  //�l��С�rӋ�cLED��B
  if(0 == can_task.can_time0)
  {
    can_task.can_time0 = 1000;
    
    //Сʱ�� 
    ucCanSend0[0] = working_time_h >> 8;          
    ucCanSend0[1] = working_time_h;
    ucCanSend0[2] = working_time_l >> 8;
    ucCanSend0[3] = working_time_l;

    //LED״̬
    ucCanSend0[4] = (LED_OnOffBitsMap >> 24) & 0xff;             
    ucCanSend0[5] = (LED_OnOffBitsMap >>16) & 0xff;
    ucCanSend0[6] = (LED_OnOffBitsMap >>8) & 0xff;
    ucCanSend0[7] = LED_OnOffBitsMap & 0xff;
    
    //����ת��ƶ��⴦��
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
  
  //�l����λ��ˮ�ء��͜ء��D��
  if(0 == can_task.can_time1)
  {
    can_task.can_time1 = 1000;
    
    ucCanSend1[0] = 0;                                                          //�豸��ͣ�T4����ʹ��ԓ�ֹ�
    ucCanSend1[1] = 0;                                                          //����ѹֵ
    ucCanSend1[2] = mnuich_atd[ATD_CHANNEL_FUEL_LEVEL].new_convert_value / 10;  //��λֵ
    ucCanSend1[3] = can_data.currWaterTemp;                                     //ˮ��  
    ucCanSend1[4] = mnuich_atd[ATD_CHANNEL_TRANS_OIL_TEMP].new_convert_value;   //���� 
    ucCanSend1[5] = (engineSpeedParams.new_speed >> 8) & 0xff;                  //ת�ٸ��ֽ�
    ucCanSend1[6] = engineSpeedParams.new_speed & 0xff;                         //ת�ٵ��ֽ�
    ucCanSend1[7] = 0;                                                          //��ѹ��2����
    
    Can_Tx_Frame(8,MESSAGE_EXTENDED_FORMAT,0x1fff5352,ucCanSend1);
  }
  
  //�l���O����Ϣ
  if(0 == can_task.can_time3)
  {
    can_task.can_time3 = 10000;
    
    Can_Tx_Frame(8, MESSAGE_EXTENDED_FORMAT, 0X18FF5652, CAN_DeviceInfoBuffer);
  }
  
  //�l������Һλ
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
* Description	: �жϷ�����򣺽��������ݴ洢��������
*******************************************************************************/
#pragma CODE_SEG NON_BANKED
void interrupt 38 mscan_rx_isr(void) 
{
  unsigned char ucI = 0;
  unsigned char *addr = NULL;    

  //�ر��ж�
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

  //���ж�
  asm("cli");
}
#pragma CODE_SEG DEFAULT
