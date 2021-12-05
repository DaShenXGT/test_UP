#ifndef __UPDATE_H__
#define __UPDATE_H__

#include "board.h"

//������Ϣ�洢ҳ
#define UPDATE_INFOR_PAGE 15
//������Ϣ��ҳ�е���Ŀ����8�ֽ�Ϊ��λ��
#define UPDATE_CMD_STORED_NBR                           31      //��������洢��Ŀ
#define UPDATE_ENCRIPTION_KEY_STORED_NBR                30      //��Կ�洢��Ŀ
#define UPDATE_DEV_TYPE_PRODUCT_CODE_STORED_NBR         29      //�豸����&��������洢��Ŀ
#define UPDATE_VIN_SOFT_VER_NBR                         28      //���ܺ�&����汾�洢��Ŀ
#define UPDATE_PROGRAM_INFOR_STORED_NBR                 27      //��д��Ϣ�洢��Ŀ
#define UPDATE_DATA_TRANS_CTRL_INFOR_STORED_NBR         26      //���ݴ��������Ϣ�洢��Ŀ


//��Ϣ��֯��ʽ��
//Byte0         - Byte1         - Byte2       - Byte3       - Byte4       - Byte5       - Byte6             - Byte7
//��Ŀ��        - �豸����      - �����豸    - ������ʽ    - �����СMSB - �����СLSB - ����������汾MSB - ����������汾MSB
//�ѽ��հ���MSB - �ѽ��հ���LSB - �ۻ�CRC MSB - �ۻ�CRC LSB - �ܰ���MSB   - �ܰ���LSB   - �����ɹ���־MSB   - �����ɹ���־LSB
//CRC MSB       - CRC LSB
typedef union
{
  unsigned int uiBuffer[4];
  unsigned char ucBuffer[8];
  
} Update_FlashByteMap_t;

typedef struct
{
  Update_FlashByteMap_t inforByteMap;
  //Update_FlashByteMap_t crcByteMap;
  
} Update_InforStoreUnit_t;


void Update_CmdCheck(void);


extern bool updateCmdFlag;
extern Update_InforStoreUnit_t Update_CmdStoreUnit;
extern Update_InforStoreUnit_t Update_DataRecvCtrlStoreUnit;
extern Update_InforStoreUnit_t *Update_StoreUnitPtr[6];

#endif