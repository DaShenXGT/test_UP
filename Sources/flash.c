#include <hidef.h>	
#include <mc9s12hy64.h>
#include "app.h"
#include "cpu.h"
#include "board.h"
#include "time.h"
#include "flash.h"

extern	queue_list	systemTask;

int DFLASH_WorkTimeSavePos;
bool DFLASH_SaveWorkTimeFlag;

#pragma CODE_SEG DEFAULT
/******************************************************************************
@ Func: Flash初始化
******************************************************************************/
void FLASH_Init(void) 
{ 
  //等待上一条Flash指令执行完成  
  while (!FSTAT_CCIF)				
    ;
    
  //BUSCLK 16MHz - Vital that this is correct for porper flash operation
  FCLKDIV = 0x0f;
  while(!FCLKDIV_FDIVLD)
    ;               
  FCNFG = 0;
  //FDIV value is locked and cannot be changed.
  //FCLKDIV |= 0x40; 
  //Protected High Range 0x3_F800–0x3_FFFF     
  //FPROT = 0x84;
  //Disable any protection set on DFlash	                      
  //DFPROT = 0x8f;  
  //  	FSTAT |= 0xb0;
}

/******************************************************************************
@ Func:  启动Flash命令
@ Param: params -- DFlash命令后参数个数
         command -- 指令
         ccob0~ccob5 -- 指令后参数
@ Ret:   执行完毕，返回FSTAT寄存器内容；没有执行（上一个操作未执行完毕），返回
         FLASH_BUSY 
******************************************************************************/
Uint8_t FLASH_LaunchCmd(Uint8_t params, Uint8_t command, Uint8_t ccob0, Uint16_t ccob1, 
                        Uint16_t ccob2, Uint16_t ccob3, Uint16_t ccob4, Uint16_t ccob5)
{
  while(!FSTAT_CCIF)
    ;
    
	if(FSTAT_CCIF == 1)
	{																	
		/* Clear any error flags*/	
  	//FSTAT = (FPVIOL_MASK | ACCERR_MASK); 
  	if(FSTAT_ACCERR)           //判断并清除标志位；  
        FSTAT_ACCERR=1;  
    if(FSTAT_FPVIOL)           //判断并清除标志位；  
        FSTAT_FPVIOL=1;  

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
	   	 		FCCOB = ccob3; 						/* Write next data word to CCOB buffer. */
    			if(++FCCOBIX != params) 
    			{
      			FCCOB = ccob4; 					/* Write next data word to CCOB buffer. */
      			if(++FCCOBIX != params)
      			{
     		  		FCCOB = ccob5; 				/* Write next data word to CCOB buffer. */
      			}
    			} 											
    		}  
  		}
  	}
  	
  	//FCCOBIX必须正确
		FCCOBIX = params - 1;
	  
	 	/* Clear command buffer empty flag by writing a 1 to it */
		FSTAT |= CCIF_MASK;
		//FSTAT_CCIF = 1;
		/* wait for the command to complete */
  	while (!FSTAT_CCIF) 							
      ;
  	
  	return(FSTAT);									/* command completed */
  }
  	 
	return(FLASH_BUSY);								/* state machine busy */
}

/******************************************************************************
@ Func: 擦除DFLASH
******************************************************************************/
void FLASH_EraseDFlash()
{
  FLASH_LaunchCmd(2, ERASE_D_FLASH_BLOCK, GLOBAL_ADDRESS_MSB(DFLASH_GLOBAL_BASE), GLOBAL_ADDRESS_LOW_WORD(DFLASH_GLOBAL_BASE),
                  0, 0, 0, 0);
}

/******************************************************************************
@ Func: 擦除DFLASH扇区
@ Param: unsigned char sectorNum -- 扇区号
******************************************************************************/
void DFlash_EraseSector(unsigned char sectorNum)
{
  unsigned long globalAddr = DFLASH_GLOBAL_BASE + DFLASH_SECTOR_SIZE * sectorNum;
  
  FLASH_LaunchCmd(2, ERASE_D_FLASH_SECTOR, GLOBAL_ADDRESS_MSB(globalAddr), GLOBAL_ADDRESS_LOW_WORD(globalAddr),
                  0, 0, 0, 0);    
}

/******************************************************************************
@ Func: 写入8字节数据
@ Param: unsigned int number -- 数据序号
         unsigned char *pData -- 数据指针
******************************************************************************/
void DFlash_Program(unsigned char sector, unsigned char nbr, unsigned int *pData)
{
  unsigned long globalAddr = DFLASH_WRITE_ADDR(sector, nbr);
  
  FLASH_LaunchCmd(6, PROGRAM_D_FLASH, GLOBAL_ADDRESS_MSB(globalAddr), GLOBAL_ADDRESS_LOW_WORD(globalAddr), 
                  pData[0], pData[1], pData[2], pData[3]);
}

/******************************************************************************
@ Func:  读出4字节数据
@ Param: number -- 数据序号
******************************************************************************/
unsigned long DFlash_ReadData(unsigned int number)
{
  return FLASH_READ_DWORD(DFLASH_DATA_READ_ADDR(number));
}

/******************************************************************************
@ Func:  读出4字节数据后的校验码
@ Param: number -- 数据序号
******************************************************************************/
unsigned long DFlash_ReadCheck(unsigned int dataNumber)
{
  return FLASH_READ_DWORD(DFLASH_DATA_READ_ADDR(dataNumber) + sizeof(unsigned long));
}
