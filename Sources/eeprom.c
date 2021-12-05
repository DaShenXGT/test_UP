#include "eeprom.h"
#include	<mc9s12hy64.h>


/*********************************************�ۺ�����********************************************/
#ifndef nop
  #define nop asm NOP
#endif

unsigned char IIC_Flag; //����Ӧ���־λ��0��ʾ���յ�Ӧ��1��ʾδ���ܵ�
//SCL-PH0 SDA-PH3 WC-PP5
//�����˿�
#define	SCL          	 	PTR_PTR6
#define	SDA         		PTR_PTR5
#define	WC          		PTH_PTH0





/*******************************************�����ʱ**********************************************/
//�������ʱ
static void delay_us(unsigned int num)	
{
	unsigned int i,j;
	for(j=0;j<num;j++) 
	{	  
    for(i=0;i<5;i++);
	}
}

static void nops(unsigned char i) {
  while(i--) {
    nop;
    nop;
    nop;
    nop;
    nop;
  }
}

/**************************************************ģ��IIC����*****************************************/
void EEPROM_IIC_Init(void)
{
  DDRR |= 0X60;
  PTR &= 0X9F;
  DDRH |= 0X01;
  PTH |= 0X01;
}

//��������
void iic_start(void) 
{
	SDA = 1;  //�Ȳ���SDA,�ڲ���SCL
	SCL = 1;
  nops(1);
	SDA = 0;
  nops(1);
  SCL = 0;   //ǯס����
}

void iic_stop(void) 
{
  DDRR |= 0x20;
  SCL = 0;
  SDA = 0;
  SCL = 1;
  nops(1); 
  SDA = 1; 
  nops(1);    
}

void Wait_Ack(void) 
{
  unsigned char waittime = 255;
  IIC_Flag = 0;
  
  SDA = 1;
  DDRR &= 0xDF;
  nops(1);        //�����ʱ��᲻��̫����200���������ڣ�����
  SCL = 1;
  nops(1);
  
  //�ȴ�Ӧ��λ
  while(SDA) 
  {
    if(!waittime --) 
    {
      iic_stop();
      IIC_Flag = 1;
      break;
    }
  }
  if(SDA == 0)
    IIC_Flag = 0;
  else
    IIC_Flag = 1;  
  
  SCL = 0;
  nops(1);
  DDRR |= 0x20;
}
  

void iic_ack()        //��Ӧ�ź�
{
  SDA = 0; //������������
  nops(1);
  SCL = 1; //ʱ����������
  nops(1);
  SCL = 0; //ǯס����
  nops(1);
  SDA = 1;
}

void iic_noack()      //����Ӧ�ź�
{
  SDA = 1;					//����ʱ������
  nops(1);
	SCL = 1;					//�ͷ���������	
  nops(1);
	SCL = 0;         //ǯס����
}

/*********************************************EEPROM����ʵ��***********************************************/
//ָ��ѡ��
typedef enum {
  Write_Byte,
  Read_Byte,
}EEPROM_COMMAND_T;

unsigned char Get_Device_Select_Code(EEPROM_COMMAND_T com,unsigned long address) 
{
    switch(com) 
    {
      case Write_Byte:
        if(address > 0xffff)
          return 0xa2;
        else
          return 0xa0;
        break;
      case Read_Byte:
        if(address > 0xffff)
          return 0xa3;
        else
          return 0xa1;
        break;
      default:break;
    }
}
          

void iic_send_byte(unsigned char send_byte) 
{
  unsigned char i;
  unsigned char send_buff = send_byte;
  
  for(i=0;i<8;i++) 
  {
    nops(1);
		if(send_buff & 0x80)   //����Ϊ��0ʱ��SDA = 1; ����Ϊ0ʱ��SDA = 0;
		  SDA = 1;
		else
		  SDA = 0;
    nops(1);
		SCL = 1;
		nops(1);
	  SCL = 0;
	  send_buff <<= 1; 
  }	
}
  
  
unsigned char iic_rcv_byte(void) 
{
  unsigned char i,temp = 0,a;
  SDA = 1;
  DDRR &= 0XDF;
  for (i = 0; i < 8; i++)
  {
    nops(1);
    SCL = 1;
    nops(1);
    if (SDA)
      a = 0x01;
    else
      a = 0;
    temp |= (a << (7 - i));
    SCL = 0;
  }
  DDRR |= 0x20;
  return temp;
}

//������EEPROM�д洢����
unsigned char eeprom_send_byte(unsigned long romaddr, unsigned char *s, unsigned int num)     //�����ַ���
{
		unsigned char addr_l,addr_h;
		unsigned int i = 0;
		
		//�����ַ
		addr_l = romaddr & 0xff;
		addr_h = (romaddr>>8) & 0xff;
		//����д�벢��������
		WC = 0;
		iic_start(); 
		
		//д���豸ѡ����                     
	  iic_send_byte(Get_Device_Select_Code(Write_Byte,romaddr));	//Get_Device_Select_Code(Write_Byte,romaddr)				
	  Wait_Ack();
	  if(IIC_Flag == 0) 
	  {
	    //д��ʮ��λ�����ַ
	    iic_send_byte(addr_h);
      Wait_Ack();
      if(IIC_Flag == 1)
        return 1;
      iic_send_byte(addr_l);
      Wait_Ack();
      if(IIC_Flag == 1)
        return 1;
     
	  	//����num����
	  	for(i = 0; i < num; i++)    
	  	{
	  		iic_send_byte(*s);    
			  Wait_Ack();
			  if(IIC_Flag == 1)
			    return 1;
        nops(1);
			  s++;
	  	}
	  }	
	 	iic_stop();	
	  //�ȴ�д������ //����Ҫ�ȴ�5ms
	 	DelayMS(7);		  
	  WC =1;
	  return 1;
}

//������ȡEEPROM���洢������
unsigned char eeprom_rcv_byte(unsigned char devaddr, unsigned long romaddr, unsigned char *s, unsigned int num)
{
		unsigned char addr_l,addr_h;
		unsigned int i = 0;
		
		//�����ַ
		addr_l = romaddr & 0xff;
		addr_h = (romaddr>>8) & 0xff;
		
		//д���豸ѡ����
		iic_start();	
		iic_send_byte(Get_Device_Select_Code(Write_Byte,romaddr));           
		Wait_Ack();
		if(IIC_Flag == 1)
        return 1;
		
		//д��ʮ��λ��ַ
	  iic_send_byte(addr_h);         
	  Wait_Ack();
	  if(IIC_Flag == 1)
        return 1;
	  iic_send_byte(addr_l);
	  Wait_Ack();
	  if(IIC_Flag == 1)
        return 1;
	  
	  //�����漴��ַ��ȡģʽ
	  iic_start();
	  iic_send_byte(Get_Device_Select_Code(Read_Byte,romaddr));             
		Wait_Ack();
		if(IIC_Flag == 1)
        return 1;
		for(i = 0; i < num ; i++)     //��ʼ������
		{
				*s = iic_rcv_byte();
				iic_ack();
				s++;
		}
		iic_noack();
    
    //������ȡ
    nops(1);
		iic_stop();
		return 0;		
}


/*************************************�ⲿ���ýӿ�**********************************/

/*
unsigned char test[10] = {0x01,0x02,0x03,0xF1,0xF2,0x03,0x0F,0x0F,0x0F,0x0F};
unsigned char temp[11];
void EEPROM_Test(void) 
{  
		eeprom_send_byte(0x122,test,10);
			
		eeprom_rcv_byte(0xa0,0x122,temp,10);
}
*/






