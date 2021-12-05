#include <mc9s12hy64.h>
#include "update.h"
#include "board.h"
#include "flash.h"

bool updateCmdFlag;

//定义缓存：当前升级指令、历史升级指令、接收控制信息
Update_InforStoreUnit_t Update_CmdStoreUnit;
Update_InforStoreUnit_t Update_EncriptionKeyStoreUnit;
Update_InforStoreUnit_t Update_DevTypeProductCodeStoreUnit;
Update_InforStoreUnit_t Update_VinSoftVerStoreUnit;
Update_InforStoreUnit_t Update_ExecDataInforStoreUnit;
Update_InforStoreUnit_t Update_DataRecvCtrlStoreUnit;


//指向以上缓冲区的指针
Update_InforStoreUnit_t *Update_StoreUnitPtr[6] =
{
  &Update_CmdStoreUnit,
  &Update_EncriptionKeyStoreUnit,
  &Update_DevTypeProductCodeStoreUnit,
  &Update_VinSoftVerStoreUnit,
  &Update_ExecDataInforStoreUnit,
  &Update_DataRecvCtrlStoreUnit,
};

/*****************************************************************************************
@ Func:  升级命令检测
@ Brief: 接收到GPS发送的升级命令后，首先保存升级信息，然后使看门狗复位
*****************************************************************************************/
void Update_CmdCheck(void)
{
  unsigned char i = 0;
  unsigned char j = 0;
  //unsigned int crc = 0;
  
  if(updateCmdFlag)
  {
    updateCmdFlag = false;
    
    //此处必须从1开始，否则会覆盖已接收到的升级指令，导致无法升级
    for(i = 1; i < sizeof(Update_StoreUnitPtr) / sizeof(Update_StoreUnitPtr[0]); ++i)
    {
      for(j = 0; j < 4; ++j)
      {
        Update_StoreUnitPtr[i]->inforByteMap.uiBuffer[j] = FLASH_READ_WORD(DFLASH_READ_ADDR(UPDATE_INFOR_PAGE, UPDATE_CMD_STORED_NBR - i) + (j * 2));
      }
    }
    
    //擦除DFlash最后一页
    DFlash_EraseSector(UPDATE_INFOR_PAGE);
    
    //写入DFlash
    for(i = 0; i < sizeof(Update_StoreUnitPtr) / sizeof(Update_StoreUnitPtr[0]); ++i)
    {
      DFlash_Program(UPDATE_INFOR_PAGE, UPDATE_CMD_STORED_NBR - i, Update_StoreUnitPtr[i]->inforByteMap.uiBuffer);
    }
    
#ifdef WDT_ON
    CPMUARMCOP = 0xbb;  //写入非法值，使看门狗重启
#endif   

    //Should never get here!
    while(true)
      ;   
  }
}
