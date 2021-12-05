#include <hidef.h>	
#include <mc9s12hy64.h>
#include "board.h"
#include "app.h"
#include "cpu.h"
#include "time.h"
#include "atd.h"
#include "vehicle.h"
#include "smc_motor.h"
#include "print.h"

unsigned char   print_data[105];
unsigned int    print_time = 0;
unsigned char   data_buff;

void data_hex_ascii(unsigned char hex,unsigned char *data_point)
{
	unsigned char v;
	
	v = hex >> 4;
	*data_point = (v <= 9) ? 0x30 + v : 0x41 + v - 0x0A;
	v = hex & 0x0f;
	data_point++;
	*data_point = (v <= 9) ? 0x30 + v : 0x41 + v - 0x0A;
}


void print_config_uart(void)
{

	SCIBD = 0xD0;									// 19.2 kHz
	SCICR1 &= ~SCICR1_M_MASK & ~SCICR1_PE_MASK;		// 8,N,1
	SCICR2 |= SCICR2_TE_MASK;// | SCICR2_RE_MASK;		// SCI Tx/Rx enabled
//    uart_model_init();
}

/*
void TERMIO_PutChar(char C)
{ 
    while((SCISR1 & 0x80) == 0);         //keep waiting when not empty  
    SCIDRL=C;
}
*/

void print_data_setup(void)
{
    unsigned char i;
    
    print_data[0] = 'W';
    print_data[1] = 'a';
    print_data[2] = 't';
    print_data[3] = 'e';
    print_data[4] = 'r';
    print_data[5] = ':';
    data_buff = (Uint8_t)mnuich_atd[2].new_adc_value;
    data_hex_ascii(data_buff,&print_data[8]);
    data_buff = (Uint8_t)(mnuich_atd[2].new_adc_value>>8);
    data_hex_ascii(data_buff,&print_data[6]);
    print_data[10] = ',';
    data_buff = munich_motor[0].new_motor_angle;
    data_hex_ascii(data_buff,&print_data[11]);
    print_data[13] = ' ';
    print_data[14] = ' ';
    print_data[15] = ' ';
    print_data[16] = ' ';
    
    print_data[17] = 'O';
    print_data[18] = 'i';
    print_data[19] = 'l';
    print_data[20] = ':';
    data_buff = (Uint8_t)mnuich_atd[3].new_adc_value;
    data_hex_ascii(data_buff,&print_data[23]);
    data_buff = (Uint8_t)(mnuich_atd[3].new_adc_value>>8);
    data_hex_ascii(data_buff,&print_data[21]);
    print_data[25] = ',';
    data_buff = munich_motor[1].new_motor_angle;
    data_hex_ascii(data_buff,&print_data[26]);
    print_data[28] = ' ';
    print_data[29] = ' ';
    print_data[30] = ' ';
    print_data[31] = ' ';
    
    
    print_data[32] = 'P';
    print_data[33] = 'r';
    print_data[34] = 'e';
    print_data[35] = 's';
    print_data[36] = 's';
    print_data[37] = ':';
    data_buff = (Uint8_t)mnuich_atd[0].new_adc_value;
    data_hex_ascii(data_buff,&print_data[40]);
    data_buff = (Uint8_t)(mnuich_atd[0].new_adc_value>>8);
    data_hex_ascii(data_buff,&print_data[38]);
    print_data[42] = ',';
    data_buff = munich_motor[3].new_motor_angle;
    data_hex_ascii(data_buff,&print_data[43]);
    print_data[45] = ' ';
    print_data[46] = ' ';
    print_data[47] = ' ';
    print_data[48] = ' ';
    
    
    print_data[49] = 'L';
    print_data[50] = 'e';
    print_data[51] = 'v';
    print_data[52] = 'e';
    print_data[53] = 'l';
    print_data[54] = ':';
    data_buff = (Uint8_t)mnuich_atd[1].new_adc_value;
    data_hex_ascii(data_buff,&print_data[57]);
    data_buff = (Uint8_t)(mnuich_atd[1].new_adc_value>>8);
    data_hex_ascii(data_buff,&print_data[55]);
    print_data[59] = ',';
    data_buff = munich_motor[4].new_motor_angle;
    data_hex_ascii(data_buff,&print_data[60]);
    print_data[62] = ' ';
    print_data[63] = ' ';
    print_data[64] = ' ';
    print_data[65] = ' ';
    
    print_data[66] = 'S';
    print_data[67] = 'p';
    print_data[68] = 'e';
    print_data[69] = 'e';
    print_data[70] = 'd';
    print_data[71] = ':';
    data_buff = (Uint8_t)(Motor2.u32_cmdpos>>8);
    data_hex_ascii(data_buff,&print_data[72]);
    data_buff = (Uint8_t)Motor2.u32_cmdpos;
    data_hex_ascii(data_buff,&print_data[74]);
    print_data[76] = ',';
    data_buff = (Uint8_t)(Motor2.u32_pos>>8);
    data_hex_ascii(data_buff,&print_data[77]);
    data_buff = (Uint8_t)Motor2.u32_pos;
    data_hex_ascii(data_buff,&print_data[79]);
    print_data[81] = ' ';
    print_data[82] = ' ';
    print_data[83] = ' ';
    print_data[84] = ' ';
    
    print_data[85] = 'V';
    print_data[86] = 'O';
    print_data[87] = 'L';
    print_data[88] = ' ';
    print_data[89] = ' ';
    print_data[90] = ':';
    data_buff = (Uint8_t)mnuich_atd[4].new_adc_value;
    data_hex_ascii(data_buff,&print_data[93]);
    data_buff = (Uint8_t)(mnuich_atd[4].new_adc_value>>8);
    data_hex_ascii(data_buff,&print_data[91]);
    print_data[95] = ',';
    data_buff =  (Uint8_t)munich_speed.new_speed;
    data_hex_ascii(data_buff,&print_data[98]);
    data_buff = (Uint8_t)(munich_speed.new_speed>>8);
    data_hex_ascii(data_buff,&print_data[96]);


    print_data[100] = '\n';
    
    while((SCISR1 & 0x80) == 0);
    for(i = 0; i < 101; i++) 
	{
		SCIDRL  = print_data[i];
		while(!SCISR1_TC);					// wait for Tx Complete flag to set
	}
}

void print_handle(unsigned char print_id)
{
    switch(print_id)
    {
        case 0:
            print_config_uart();
            break;
        case 1:
            if(_pasti(print_time) >= PRINT_TIME_VALUE)
            {
            	print_data_setup();
            	print_time = jiffies;
            }
            break;
        default:
        	break;
    }
}
