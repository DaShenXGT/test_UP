#ifndef __UPDATE_H__
#define __UPDATE_H__

#include "board.h"

//升级信息存储页
#define UPDATE_INFOR_PAGE 15
//升级信息在页中的条目（以8字节为单位）
#define UPDATE_CMD_STORED_NBR                           31      //升级命令存储条目
#define UPDATE_ENCRIPTION_KEY_STORED_NBR                30      //密钥存储条目
#define UPDATE_DEV_TYPE_PRODUCT_CODE_STORED_NBR         29      //设备类型&生产编码存储条目
#define UPDATE_VIN_SOFT_VER_NBR                         28      //车架号&软件版本存储条目
#define UPDATE_PROGRAM_INFOR_STORED_NBR                 27      //烧写信息存储条目
#define UPDATE_DATA_TRANS_CTRL_INFOR_STORED_NBR         26      //数据传输控制信息存储条目


//信息组织形式：
//Byte0         - Byte1         - Byte2       - Byte3       - Byte4       - Byte5       - Byte6             - Byte7
//项目组        - 设备大类      - 具体设备    - 升级方式    - 程序大小MSB - 程序大小LSB - 待更新软件版本MSB - 待更新软件版本MSB
//已接收包数MSB - 已接收包数LSB - 累积CRC MSB - 累积CRC LSB - 总包数MSB   - 总包数LSB   - 升级成功标志MSB   - 升级成功标志LSB
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