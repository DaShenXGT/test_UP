#include <hidef.h>	
#include <mc9s12hy64.h>
#include "app.h"
#include "cpu.h"
#include "board.h"
#include "time.h"
#ifdef	CAN_COMM
#include "CAN.h"
#include "lock_dashboard.h"
#endif
#include "flash.h"
extern	queue_list	munich_task;
extern	vehicle_warning			munich_warning; 

//#define READword(address)     ((Uint16_t)(*(volatile Uint16_t *__near)(address)))

Uint32_t	work_time_addr;
Uint16_t	work_addr_addr;

Uint8_t	offset_add1;
Uint8_t	offset_add2;

Uint16_t	test_cycle;
Uint16_t	test_data;
Uint32_t	test_addr;

/************************* function prototypes *******************************/
#pragma CODE_SEG DEFAULT
Uint8_t LaunchFlashCommand(Uint8_t params, Uint8_t command, Uint8_t ccob0, Uint16_t ccob1, 
                               Uint16_t ccob2, Uint16_t ccob3, Uint16_t ccob4, Uint16_t ccob5);

/******************************************************************************
Function Name	:	FLASH_fill
Engineer		:	
Date			:	
Arguments		:	type:D-Flash or P-flash
val				: 	data pattern 
Return			: 	0/success; others/fail 
Notes			:	This function writes known data pattern to flash and then reads the data 
            		back before data checking
******************************************************************************/
static Uint16_t FLASH_Write(Uint8_t id,Uint16_t	address,Uint16_t val) 
{
  	Uint16_t 	base, addr;//, j;
  
  	base = address;
   
    if(LaunchFlashCommand(2, ERASE_D_FLASH_SECTOR, FlashStartAddrGH, base, 0, 0, 0, 0) != CCIF_MASK) /* erase the flash block again*/
      	return FLASH_ACCESS_ERROR;
	
	if(id == 0)
	{
		addr = base; 
		if(LaunchFlashCommand(5, PROGRAM_D_FLASH, FlashStartAddrGH, addr, val, val, val, 0) != CCIF_MASK) /* timing 1 word program command */
      		return FLASH_ACCESS_ERROR;
      	
  }
  else
  {
		addr = base; 
		if(LaunchFlashCommand(5, PROGRAM_D_FLASH, FlashStartAddrGH, addr, working_time_h, working_time_l, time_6m, 0) != CCIF_MASK) /* timing 1 word program command */
      		return FLASH_ACCESS_ERROR;
//    	addr += ADDR_LEN;
//		if(LaunchFlashCommand(5, PROGRAM_D_FLASH, FlashStartAddrGH, addr, working_time_h, working_time_l, time_6m, 0) != CCIF_MASK) //timing 1 word program command 
//      		return FLASH_ACCESS_ERROR;
//		addr += ADDR_LEN;
//		if(LaunchFlashCommand(5, PROGRAM_D_FLASH, FlashStartAddrGH, addr, working_time_h, working_time_l, time_6m, 0) != CCIF_MASK) // timing 1 word program command 
 //     		return FLASH_ACCESS_ERROR;
    
	} 
  return FLASH_OK;
}

/******************************************************************************
Function Name	:	LaunchFlashCommand
Engineer		:	  b06320
Date			:	    08/06/09
Arguments		:	
Return			:
Notes			:	This function does not check if the Flash is erased.
    This function does not explicitly verify that the data has been sucessfully programmed.
	This function must be located in RAM or a flash block not being programmed.
******************************************************************************/
Uint8_t LaunchFlashCommand(Uint8_t params, Uint8_t command, Uint8_t ccob0, Uint16_t ccob1, Uint16_t ccob2, Uint16_t ccob3, Uint16_t ccob4, Uint16_t ccob5)
{
  	if(FSTAT_CCIF == 1)
	{																	
		/* Clear any error flags*/	
	  	FSTAT = (FPVIOL_MASK | ACCERR_MASK); 

    /* Write the command id / ccob0 */
		FCCOBIX = 0;
		FCCOBHI = command;
		FCCOBLO = ccob0;

    	if(++FCCOBIX != params) 
    	{
      		FCCOB = ccob1; 								/* Write next data word to CCOB buffer. */ 
     	 	if(++FCCOBIX != params) 
      		{
  		  		FCCOB = ccob2; 							/* Write next data word to CCOB buffer. */
        		if(++FCCOBIX != params) 
        		{
   		   	 		FCCOB = ccob3; 							/* Write next data word to CCOB buffer. */
          			if(++FCCOBIX != params) 
          			{
            			FCCOB = ccob4; 							/* Write next data word to CCOB buffer. */
            			if(++FCCOBIX != params) 
         		  		FCCOB = ccob5; 					/* Write next data word to CCOB buffer. */
          			} 											
        		}  
	  		}
    	}
		FCCOBIX = params-1;
	  
	 	/* Clear command buffer empty flag by writing a 1 to it */
		FSTAT = CCIF_MASK;
    	while (!FSTAT_CCIF) {							/* wait for the command to complete */
      /* Return status. */
    	}
    	return(FSTAT);									/* command completed */
  	} 
	return(FLASH_BUSY);								/* state machine busy */
}


/******************************************************************************
Function Name	:	MDelay
Engineer		:	
Date			:	08/07/2009

Parameters		:	none
Returns			:	none
Notes			:	. 
******************************************************************************/

void MDelay(int delayTime)					
  {
    int x;				 //outer loop counter 
    char y;									 //inner loop counter 

    for (x=0; x<delayTime; x++){for (y=0; y<100; y++){}}
  }
  
  
Uint8_t	get_address(void)
{
	Uint16_t	time_h_temp2,time_h_temp3;
	Uint16_t  time_l_temp2,time_l_temp3;
	Uint16_t  time_6m_temp2,time_6m_temp3;
	Uint16_t	time_6m_temp0,time_h_temp0,time_l_temp0;
	Uint8_t		offset_temp = 0xFF;
	Uint8_t		cycle_time;
	
	for(cycle_time = 0;cycle_time < 16;cycle_time++)
	{
		time_h_temp0 = READword(WORK_TIME_ADDR_STAR1+FLASH_OFFSET+ADDR_LEN*cycle_time);
		time_l_temp0 = READword(WORK_TIME_ADDR_STAR1+FLASH_OFFSET+ADDR_LEN*cycle_time+2);
		time_6m_temp0 = READword(WORK_TIME_ADDR_STAR1+FLASH_OFFSET+ADDR_LEN*cycle_time+4);

		time_h_temp2 = READword(WORK_TIME_ADDR_STAR2+FLASH_OFFSET+ADDR_LEN*cycle_time);
		time_l_temp2 = READword(WORK_TIME_ADDR_STAR2+FLASH_OFFSET+ADDR_LEN*cycle_time+2);
		time_6m_temp2 = READword(WORK_TIME_ADDR_STAR2+FLASH_OFFSET+ADDR_LEN*cycle_time+4);
	
		time_h_temp3 = READword(WORK_TIME_ADDR_STAR3+FLASH_OFFSET+ADDR_LEN*cycle_time);
		time_l_temp3 = READword(WORK_TIME_ADDR_STAR3+FLASH_OFFSET+ADDR_LEN*cycle_time+2);
		time_6m_temp3 = READword(WORK_TIME_ADDR_STAR3+FLASH_OFFSET+ADDR_LEN*cycle_time+4);
		
		if((time_h_temp0 != 0xffff)||(time_l_temp0 != 0xffff)||(time_6m_temp0 != 0xffff)||
			(time_h_temp2 != 0xffff)||(time_l_temp2 != 0xffff)||(time_6m_temp2 != 0xffff)||
			(time_h_temp3 != 0xffff)||(time_l_temp3 != 0xffff)||(time_6m_temp3 != 0xffff))

			offset_temp = cycle_time;
	}
	if(offset_temp == 0xFF)
	{
		offset_temp = 0xff;
		return(offset_temp);
	}
	else
	{	
//		offset_temp -= 1;
		return(offset_temp);
	}
}


void Init_working_time(void)
{
//	Uint8_t  	cycle_mi;
//	Uint16_t  	data_init;
		
	offset_add1 = 0;
	offset_add2 = 0;
	working_time_h = 0;
	working_time_l = 0;
	time_6m = TIME_6M_CONST;
	working_time = 0;
	Flash_Handle();
//	cycle_mi = 0;
/*
	do
	{
		if(!FLASH_Write(1,work_time_addr,0))
			break;
		cycle_mi++;
	}while(cycle_mi<3);
*/
}

void	get_time(void)
{
	Uint16_t	time_h_buff2,time_h_buff3;	
	Uint16_t  	time_l_buff2,time_l_buff3;
	Uint16_t  	time_6m_buff2,time_6m_buff3;
	Uint16_t		time_h_buff1,time_l_buff1,time_6m_buff1;
	Uint32_t  	work_time1,work_time2,work_time3;
	
	DisableInterrupts;	
	time_h_buff1 = READword(WORK_TIME_ADDR_STAR1+FLASH_OFFSET+ADDR_LEN*offset_add1);
	time_l_buff1 = READword(WORK_TIME_ADDR_STAR1+FLASH_OFFSET+ADDR_LEN*offset_add1+2);
	time_6m_buff1 = READword(WORK_TIME_ADDR_STAR1+FLASH_OFFSET+ADDR_LEN*offset_add1+4);
	work_time1 = time_h_buff1;
	work_time1 <<= 16;
	work_time1 |= time_l_buff1;
	
	time_h_buff2 = READword(WORK_TIME_ADDR_STAR2+FLASH_OFFSET+ADDR_LEN*offset_add1);
	time_l_buff2 = READword(WORK_TIME_ADDR_STAR2+FLASH_OFFSET+ADDR_LEN*offset_add1+2);
	time_6m_buff2 = READword(WORK_TIME_ADDR_STAR2+FLASH_OFFSET+ADDR_LEN*offset_add1+4);
	work_time2 = time_h_buff2;
	work_time2 <<= 16;
	work_time2 |= time_l_buff2;
		
	time_h_buff3 = READword(WORK_TIME_ADDR_STAR3+FLASH_OFFSET+ADDR_LEN*offset_add1);
	time_l_buff3 = READword(WORK_TIME_ADDR_STAR3+FLASH_OFFSET+ADDR_LEN*offset_add1+2);
	time_6m_buff3 = READword(WORK_TIME_ADDR_STAR3+FLASH_OFFSET+ADDR_LEN*offset_add1+4);
	work_time3 = time_h_buff3;
	work_time3 <<= 16;
	work_time3 |= time_l_buff3;
		
	if(work_time1 == work_time2)
	{
		working_time_h = time_h_buff1;
		working_time_l = time_l_buff1;
		time_6m = (Uint8_t)time_6m_buff1;
	}
	else if(work_time2 == work_time3)
	{
		working_time_h = time_h_buff2;
		working_time_l = time_l_buff2;
		time_6m = (Uint8_t)time_6m_buff2;
	}
	else
	{
		working_time_h = time_h_buff1;
		working_time_l = time_l_buff1;
		time_6m = (Uint8_t)time_6m_buff1;
	}
	time_1m = TIME_1M_CONST;
	working_time = working_time_h;
	working_time <<= 16;
	working_time |= working_time_l;
	working_time = working_time_h;
	working_time <<= 16;
	working_time |= working_time_l;
	EnableInterrupts;
}

void	working_time_int(void)
{
//	Uint16_t		time_h_buff1,time_h_buff2,time_h_buff3;	
//	Uint16_t  	time_l_buff1,time_l_buff2,time_l_buff3;
//	Uint16_t  	time_6m_buff1,time_6m_buff2,time_6m_buff3;
//	Uint32_t  	work_time1,work_time2,work_time3;

	DisableInterrupts;
	offset_add1 = get_address();
	offset_add2 = offset_add1;
	
	if(offset_add1 == 0xff)
		Init_working_time();
	else
		get_time();
	EnableInterrupts;
}
/******************************************************************************
Function Name	: MAIN
Engineer	:	b19005, updated by b06320
Date			:	30/11/08, 08/07/09
Arguments	:	
Return		: 0/success; others/fail  
Note      : 

Test step : 
State     : Passed            
******************************************************************************/
void Init_flash(void) 
{   
  	volatile Uint16_t status = 0;

//  	CPMUCOP = 0x40;                  /* disable cop, stop COP and RTI counters when part is in active BDM mode. */
// 	ECLKCTL = 0x00;                  /* enable bus clock (ECLK) */

   /* initialise the memory controller prior to any commands being launched or EEE data access */
  	while (!FSTAT_CCIF) {					/* wait for FTM reset to complete */
  	}
  	FCLKDIV = 0x07;                  /* BUSCLK 16MHz - Vital that this is correct for porper flash operation  */
    FCLKDIV |= 0x40; 
         
  	FPROT = 0x84;	                   /* Protect Vectors and flash configuration field F800-FFFF*/    
  	DFPROT = 0x8f;                   /* Disable any protection set on DFlash */
//  	FSTAT |= 0xb0;
  	
}

void	save_work_addr(void)
{
	Uint8_t		save_add_time,cycle_mi;
	Uint16_t	addr_buff1,addr_buff2,addr_buff3;

	
	save_add_time = 0;
	work_addr_addr = FlashStartAddrGL;	
#ifdef	WDT_ON   //added by roolie_lxp
	__RESET_WATCHDOG();
#endif
	do
	{
		cycle_mi = 0;
		do
		{
			if(!FLASH_Write(0,work_addr_addr,work_time_addr))
				break;
			else
				cycle_mi++;
		}while(cycle_mi<3);
#ifdef	WDT_ON   //added by roolie_lxp
		__RESET_WATCHDOG();
#endif
		addr_buff1 = READword(work_addr_addr+FLASH_OFFSET);
		addr_buff2 = READword(work_addr_addr+FLASH_OFFSET+2);
		addr_buff3 = READword(work_addr_addr+FLASH_OFFSET+4);
		if((work_time_addr != addr_buff1)||(work_time_addr != addr_buff2)||(work_time_addr != addr_buff3))
		{
			save_add_time++;
			if((save_add_time%3) == 0)
				work_addr_addr += ADDR_LEN;
		}	
		else
			save_add_time = 0;
	}while(save_add_time);	
#ifdef	WDT_ON   //added by roolie_lxp
	__RESET_WATCHDOG();
#endif
}

void	Flash_Handle(void)
{
	Uint8_t		save_time_test;//,save_add_time,cycle_mi;
	Uint16_t	time_h_buf1,time_l_buf1;
	Uint16_t	time_h_buf2,time_l_buf2;
	Uint16_t	time_h_buf3,time_l_buf3;
	Uint16_t	w_t_6m_buf1,w_t_6m_buf2,w_t_6m_buf3;
	
	Uint32_t	working_time_temp;
	
	if(offset_add1 != offset_add2)
	{	
		offset_add1 = get_address();
		offset_add2 = offset_add1;
	}
	working_time_temp = working_time_h;
	working_time_temp <<= 16;
	working_time_temp |= working_time_l;
	if(working_time_temp != working_time)
		get_time();

	DisableInterrupts;
	save_time_test = 0;
#ifdef	WDT_ON   //added by roolie_lxp
	__RESET_WATCHDOG();
#endif
#ifdef OUT_WDT
	Feed_WatchDog();
#endif

	do
	{
		if((!FLASH_Write(1,WORK_TIME_ADDR_STAR1+ADDR_LEN*offset_add1,0))&&
			(!FLASH_Write(1,WORK_TIME_ADDR_STAR2+ADDR_LEN*offset_add1,0))&&
			(!FLASH_Write(1,WORK_TIME_ADDR_STAR3+ADDR_LEN*offset_add1,0)))
		{
			time_h_buf1 = READword(WORK_TIME_ADDR_STAR1+FLASH_OFFSET+ADDR_LEN*offset_add1);
			time_l_buf1 = READword(WORK_TIME_ADDR_STAR1+FLASH_OFFSET+ADDR_LEN*offset_add1+2);
			w_t_6m_buf1 = READword(WORK_TIME_ADDR_STAR1+FLASH_OFFSET+ADDR_LEN*offset_add1+4);
			
			time_h_buf2 = READword(WORK_TIME_ADDR_STAR2+FLASH_OFFSET+ADDR_LEN*offset_add1);
			time_l_buf2 = READword(WORK_TIME_ADDR_STAR2+FLASH_OFFSET+ADDR_LEN*offset_add1+2);
			w_t_6m_buf2 = READword(WORK_TIME_ADDR_STAR2+FLASH_OFFSET+ADDR_LEN*offset_add1+4);
			
			time_h_buf3 = READword(WORK_TIME_ADDR_STAR3+FLASH_OFFSET+ADDR_LEN*offset_add1);
			time_l_buf3 = READword(WORK_TIME_ADDR_STAR3+FLASH_OFFSET+ADDR_LEN*offset_add1+2);
			w_t_6m_buf3 = READword(WORK_TIME_ADDR_STAR3+FLASH_OFFSET+ADDR_LEN*offset_add1+4);
			if((working_time_h != time_h_buf1)||(working_time_h != time_h_buf2)||(working_time_h != time_h_buf3)
				||(working_time_l != time_l_buf1)||(working_time_l != time_l_buf2)||(working_time_l != time_l_buf3)
				||(time_6m != w_t_6m_buf1)||(time_6m != w_t_6m_buf2)||(time_6m != w_t_6m_buf3))	
			{
				save_time_test ++;
				if((save_time_test%3) == 0)
				{					
					if(offset_add1 >= 15)
					{
						offset_add1 = 0;
						offset_add2 = offset_add1;
					}
					else
					{
						offset_add1++;
						offset_add2 = offset_add1;
					}
//					if(work_time_addr < 0x5000)
//					{	
//						work_time_addr += FLASH_D_SECTSIZE; 
//						save_work_addr();
//					}
//					else
//						save_time_test = 0;
				}
			}
			else
				save_time_test = 0;
		}
		else
		{	
			save_time_test++;
			if((save_time_test%10) == 0)
			{	
				offset_add1++;
				offset_add2 = offset_add1;
//				work_time_addr += FLASH_D_SECTSIZE; 
//				save_time_test = 0;
//				save_work_addr();
			}

		}
	}while(save_time_test);
#ifdef	WDT_ON   //added by roolie_lxp
	__RESET_WATCHDOG();
#endif
#ifdef OUT_WDT
	Feed_WatchDog();
#endif
	EnableInterrupts;
	munich_task.b.Flash_active = FALSE;
}

void save_can_status(void)
{
	Uint16_t	can_temp1,can_temp2,can_temp3;
	Uint8_t		can_save_time;

	DisableInterrupts;
	can_save_time = 0;
#ifdef	WDT_ON   //added by roolie_lxp
	__RESET_WATCHDOG();
#endif
#ifdef OUT_WDT
	Feed_WatchDog();
#endif
	do
	{
		if(!FLASH_Write(0,CAN_ADDR_STAR,can_task.can_status))//&&
//			(!FLASH_Write(0,CAN_ADDR_STAR,can_task.can_status))&&
//			(!FLASH_Write(0,CAN_ADDR_STAR,can_task.can_status)))
		{
			can_temp1 = READword(CAN_ADDR_STAR+FLASH_OFFSET);
			can_temp2 = READword(CAN_ADDR_STAR+FLASH_OFFSET+2);
			can_temp3 = READword(CAN_ADDR_STAR+FLASH_OFFSET+4);
			if((can_temp1 != can_task.can_status)||(can_temp2 != can_task.can_status)||(can_temp3 != can_task.can_status))
			{
				can_save_time ++;
				if((can_save_time%3) == 0)
				{					
					can_task.can_status = 0;
					can_save_time = 0;
				}
			}
			else
				can_save_time = 0;
		}
	}while(can_save_time);
#ifdef	WDT_ON   //added by roolie_lxp
	__RESET_WATCHDOG();
#endif
#ifdef OUT_WDT
	Feed_WatchDog();
#endif
	EnableInterrupts;
}

void	get_can_status(void)
{
	Uint16_t	can_buff1,can_buff2,can_buff3;
	
	can_buff1 = READword(CAN_ADDR_STAR+FLASH_OFFSET);
	can_buff2 = READword(CAN_ADDR_STAR+FLASH_OFFSET+2);
	can_buff3 = READword(CAN_ADDR_STAR+FLASH_OFFSET+4);
	
	if((can_buff1 == 0xffff)&&(can_buff2 == 0xffff)&&(can_buff3 ==0xffff))
	{	
		can_task.can_status = 0x80;
		save_can_status();
	}
	else
	{
		if(can_buff1 == can_buff2)
			can_task.can_status = can_buff1;
		else if(can_buff2 == can_buff3)
			can_task.can_status = can_buff2;
		else
			can_task.can_status = 0x80;
	}
	if((can_task.can_status&0xfe) == 0x88)
	{	
		munich_lock.cmd_id = 3;
		munich_warning.b.buzz_bit  = TRUE;
	}
	else if((can_task.can_status&0xfe) == 0x84)
		munich_lock.cmd_id = 2;
	else if((can_task.can_status&0xfe) == 0x82)
		munich_lock.cmd_id = 1;
	else
		munich_lock.cmd_id = 0xff;
	can_task.can_st_buff = can_task.can_status;
}
