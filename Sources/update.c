#include <mc9s12hy64.h>
#include "update.h"
#include "board.h"
#include "flash.h"

bool updateCmdFlag;

//���建�棺��ǰ����ָ���ʷ����ָ����տ�����Ϣ
Update_InforStoreUnit_t Update_CmdStoreUnit;
Update_InforStoreUnit_t Update_EncriptionKeyStoreUnit;
Update_InforStoreUnit_t Update_DevTypeProductCodeStoreUnit;
Update_InforStoreUnit_t Update_VinSoftVerStoreUnit;
Update_InforStoreUnit_t Update_ExecDataInforStoreUnit;
Update_InforStoreUnit_t Update_DataRecvCtrlStoreUnit;


//ָ�����ϻ�������ָ��
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
@ Func:  ����������
@ Brief: ���յ�GPS���͵�������������ȱ���������Ϣ��Ȼ��ʹ���Ź���λ
*****************************************************************************************/
void Update_CmdCheck(void)
{
  unsigned char i = 0;
  unsigned char j = 0;
  //unsigned int crc = 0;
  
  if(updateCmdFlag)
  {
    updateCmdFlag = false;
    
    //�˴������1��ʼ������Ḳ���ѽ��յ�������ָ������޷�����
    for(i = 1; i < sizeof(Update_StoreUnitPtr) / sizeof(Update_StoreUnitPtr[0]); ++i)
    {
      for(j = 0; j < 4; ++j)
      {
        Update_StoreUnitPtr[i]->inforByteMap.uiBuffer[j] = FLASH_READ_WORD(DFLASH_READ_ADDR(UPDATE_INFOR_PAGE, UPDATE_CMD_STORED_NBR - i) + (j * 2));
      }
    }
    
    //����DFlash���һҳ
    DFlash_EraseSector(UPDATE_INFOR_PAGE);
    
    //д��DFlash
    for(i = 0; i < sizeof(Update_StoreUnitPtr) / sizeof(Update_StoreUnitPtr[0]); ++i)
    {
      DFlash_Program(UPDATE_INFOR_PAGE, UPDATE_CMD_STORED_NBR - i, Update_StoreUnitPtr[i]->inforByteMap.uiBuffer);
    }
    
#ifdef WDT_ON
    CPMUARMCOP = 0xbb;  //д��Ƿ�ֵ��ʹ���Ź�����
#endif   

    //Should never get here!
    while(true)
      ;   
  }
}
