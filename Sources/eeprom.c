#include "eeprom.h"
#include	<mc9s12hy64.h>


/*********************************************综合设置********************************************/
#ifndef nop
  #define nop asm NOP
#endif

unsigned char IIC_Flag; //主机应答标志位，0表示接收到应答，1表示未接受到
//SCL-PH0 SDA-PH3 WC-PP5
//操作端口
#define	SCL          	 	PTR_PTR6
#define	SDA         		PTR_PTR5
#define	WC          		PTH_PTH0





/*******************************************软件延时**********************************************/
//纯软件延时
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

/**************************************************模拟IIC总线*****************************************/
void EEPROM_IIC_Init(void)
{
  DDRR |= 0X60;
  PTR &= 0X9F;
  DDRH |= 0X01;
  PTH |= 0X01;
}

//操作总线
void iic_start(void) 
{
	SDA = 1;  //先操作SDA,在操作SCL
	SCL = 1;
  nops(1);
	SDA = 0;
  nops(1);
  SCL = 0;   //钳住总线
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
  nops(1);        //这里的时间会不会太长了200个机器周期？？？
  SCL = 1;
  nops(1);
  
  //等待应答位
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
  

void iic_ack()        //响应信号
{
  SDA = 0; //拉低数据总线
  nops(1);
  SCL = 1; //时钟总线拉高
  nops(1);
  SCL = 0; //钳住总线
  nops(1);
  SDA = 1;
}

void iic_noack()      //非响应信号
{
  SDA = 1;					//拉高时钟总线
  nops(1);
	SCL = 1;					//释放数据总线	
  nops(1);
	SCL = 0;         //钳住总线
}

/*********************************************EEPROM功能实现***********************************************/
//指令选择
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
		if(send_buff & 0x80)   //与上为非0时，SDA = 1; 与上为0时，SDA = 0;
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

//用来向EEPROM中存储数据
unsigned char eeprom_send_byte(unsigned long romaddr, unsigned char *s, unsigned int num)     //发送字符串
{
		unsigned char addr_l,addr_h;
		unsigned int i = 0;
		
		//计算地址
		addr_l = romaddr & 0xff;
		addr_h = (romaddr>>8) & 0xff;
		//允许写入并开启总线
		WC = 0;
		iic_start(); 
		
		//写入设备选择码                     
	  iic_send_byte(Get_Device_Select_Code(Write_Byte,romaddr));	//Get_Device_Select_Code(Write_Byte,romaddr)				
	  Wait_Ack();
	  if(IIC_Flag == 0) 
	  {
	    //写入十六位物理地址
	    iic_send_byte(addr_h);
      Wait_Ack();
      if(IIC_Flag == 1)
        return 1;
      iic_send_byte(addr_l);
      Wait_Ack();
      if(IIC_Flag == 1)
        return 1;
     
	  	//发送num个字
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
	  //等待写入周期 //最少要等待5ms
	 	DelayMS(7);		  
	  WC =1;
	  return 1;
}

//用来读取EEPROM所存储的数据
unsigned char eeprom_rcv_byte(unsigned char devaddr, unsigned long romaddr, unsigned char *s, unsigned int num)
{
		unsigned char addr_l,addr_h;
		unsigned int i = 0;
		
		//计算地址
		addr_l = romaddr & 0xff;
		addr_h = (romaddr>>8) & 0xff;
		
		//写入设备选择码
		iic_start();	
		iic_send_byte(Get_Device_Select_Code(Write_Byte,romaddr));           
		Wait_Ack();
		if(IIC_Flag == 1)
        return 1;
		
		//写入十六位地址
	  iic_send_byte(addr_h);         
	  Wait_Ack();
	  if(IIC_Flag == 1)
        return 1;
	  iic_send_byte(addr_l);
	  Wait_Ack();
	  if(IIC_Flag == 1)
        return 1;
	  
	  //进入随即地址读取模式
	  iic_start();
	  iic_send_byte(Get_Device_Select_Code(Read_Byte,romaddr));             
		Wait_Ack();
		if(IIC_Flag == 1)
        return 1;
		for(i = 0; i < num ; i++)     //开始读数据
		{
				*s = iic_rcv_byte();
				iic_ack();
				s++;
		}
		iic_noack();
    
    //结束读取
    nops(1);
		iic_stop();
		return 0;		
}


/*************************************外部调用接口**********************************/

/*
unsigned char test[10] = {0x01,0x02,0x03,0xF1,0xF2,0x03,0x0F,0x0F,0x0F,0x0F};
unsigned char temp[11];
void EEPROM_Test(void) 
{  
		eeprom_send_byte(0x122,test,10);
			
		eeprom_rcv_byte(0xa0,0x122,temp,10);
}
*/






