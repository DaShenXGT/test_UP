#ifndef	_UART_H
#define	_UART_H

#define BAUD9600			0xD0//9600
#define RBUF_SIZE	  150
#define TBUF_SIZE   150
#define FRAME_START 0xAA
typedef struct{
unsigned char R_head;
unsigned char R_tail;
unsigned char R_count;
unsigned char R_overflow;
unsigned char R_buf[RBUF_SIZE];
unsigned char R_data_buf[RBUF_SIZE];
unsigned char R_step;
unsigned int  R_recvcrc;
unsigned char R_cmd;
unsigned char R_len;
unsigned char R_oldlen;
unsigned char T_cmd;//发送的命令码
unsigned char T_head;
unsigned char T_tail;
unsigned char T_count;
unsigned char T_buf[TBUF_SIZE];
unsigned char T_disabled;
unsigned int  T_recvcrc;
unsigned int  time_delay_count;//延时时间  单位是秒 
unsigned char tx_nub;//发送的次数
}siocirqueue;
extern siocirqueue RTbuf_UART0;
void  rxcrc0(unsigned char  crcbuf);
void  sendcrc0(unsigned char  crcbuf);
void Tbuf_putchar(unsigned char x,unsigned char channel,unsigned char crc);
void Rbuf_init(siocirqueue *RTbuf);
void Tbuf_init(siocirqueue *RTbuf);
void ProcRx0(void);
void Uart0_Proc(void);
void sensor_inf_tx(void);//发送传感器信息
void config_uart_gps(void);
void config_uart(void);
#endif