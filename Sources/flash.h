#ifndef	_FLASH_H
#define	_FLASH_H

#include "app.h"

//Flash寄存器状态
#define CCIF_MASK               0x80
#define ACCERR_MASK             0x20
#define FPVIOL_MASK             0x10

#define FLASH_OK                0
#define FLASH_BUSY              1
#define FLASH_PROGRAM_ERROR     2
#define FLASH_PROTECT_ERROR     4  
#define FLASH_COMMAND_ERROR     8
#define FLASH_ACCESS_ERROR      16 

//Flash读一个字
#define FLASH_READ_WORD(address) ((Uint16_t)(*(volatile Uint16_t *__near)(address)))
//Flash读双字
#define FLASH_READ_DWORD(addr) ((unsigned long)(*(volatile unsigned long *)(addr)))

//DFlash全局地址
#define DFLASH_GLOBAL_BASE                  0x4400  //DFlash基址
#define DFLASH_GLOBAL_END                   0x53ff  //DFlash结束地址
//DFlash局部地址
#define DFLASH_LOCAL_BASE                   0x0400
#define DFLASH_LOCAL_END                    0x13ff

#define DFLASH_SECTOR_NUM                   16      //DFlash扇区数目
#define DFLASH_SECTOR_SIZE                  256     //DFlash扇区大小
#define DFLASH_PROGRAM_DATA_SIZE            8       //每次写入的字节数
#define DFLASH_PROGRAM_DATA_NUM_PER_SECTOR  32      //每个扇区能够写入以8个字节为单位的数据数目

//第x个扇区地址
#define DFLASH_SECTOR_GLOBAL_ADDR(x) (DFLASH_GLOBAL_BASE + (DFLASH_SECTOR_SIZE * (x)))
#define DFLASH_SECTOR_LOCAL_ADDR(x) (DFLASH_LOCAL_BASE + (DFLASH_SECTOR_SIZE * (x)))

//占用的扇区共能写入的数据数目（一共16个扇区，占用8个）
//#define DFLASH_PROGRAM_DATA_MAX_NUM (DFLASH_PROGRAM_DATA_NUM_PER_SECTOR * 8)

//工作时间数据存储
#define DFLASH_WRITE_ADDR(sector, nbr) (DFLASH_GLOBAL_BASE + ((unsigned long)DFLASH_SECTOR_SIZE * (sector)) + ((unsigned long)(nbr) * 8))
#define DFLASH_READ_ADDR(sector, nbr) (DFLASH_LOCAL_BASE + ((unsigned long)DFLASH_SECTOR_SIZE * (sector)) + ((unsigned long)(nbr) * 8))

//第x个数据的地址   
//#define DFLASH_DATA_WRITE_ADDR(x) (DFLASH_GLOBAL_BASE + (x * DFLASH_PROGRAM_DATA_SIZE))
//#define DFLASH_DATA_READ_ADDR(x)   (DFLASH_LOCAL_BASE + (x * DFLASH_PROGRAM_DATA_SIZE))

//第x个数据所在扇区号0~7
//#define DFLASH_DATA_SECTOR(x) (unsigned char)((x / DFLASH_PROGRAM_DATA_NUM_PER_SECTOR))

//第x个数据在扇区内的偏移
//#define DFLASH_DATA_SECTOR_OFFSET(x) (x % DFLASH_PROGRAM_DATA_NUM_PER_SECTOR)

//全局地址最高2位和低字
#define GLOBAL_ADDRESS_MSB(globalAddr)      ((unsigned char)((globalAddr & 0xffff0000) >> 16) & 0x03)
#define GLOBAL_ADDRESS_LOW_WORD(globalAddr) ((unsigned int)(globalAddr & 0xffff))



Uint8_t FLASH_LaunchCmd(Uint8_t params, Uint8_t command, Uint8_t ccob0, Uint16_t ccob1, 
                               Uint16_t ccob2, Uint16_t ccob3, Uint16_t ccob4, Uint16_t ccob5);
void FLASH_EraseDFlash(void);
void DFlash_EraseSector(unsigned char sectorNum);
void DFlash_Program(unsigned char sector, unsigned char nbr, unsigned int *pData);
unsigned long DFlash_ReadData(unsigned int number);
unsigned long DFlash_ReadCheck(unsigned int dataNumber);
int DFlash_SearchLastWorkTime(void);
void DFlash_SaveWorkTime(unsigned int *pData);
void DFlash_SaveZeroFlag(void);
unsigned long DFlash_ReadZeroFlag(void);



/**** P-Flash and D-Flash Commands ****/

#define ERASE_VERIFY_ALL_BLOCKS  0x01 
/* Verify that all program and data Flash blocks are erased. */
/* CCOBIX end = 0 */
/* CCOB Params - NONE */
/* MGSTAT set if fault */

#define ERASE_VERIFY_BLOCK       0X02      
/* Verify that a Flash block is erased. */
/* CCOBIX end = 0 */
/* CCOB Params - gpage */
/* MGSTAT set if fault */

#define ERASE_ALL_BLOCKS         0x08 
/* Erase all program and data Flash blocks.
   An erase of all Flash blocks is only possible when the FPLDIS, FPHDIS, and FPOPEN
   bits in the FPROT register and the EPDIS and EPOPEN bits in the EPROM register are
   set prior to launching the command. */
/* CCOBIX end = 0 */
/* CCOB Params - NONE */
/* MGSTAT set if fault, FPVIOL / ACCERR set where appropriate */

#define UNSECURE_FLASH           0x0B 
/*Supports a method of releasing MCU security by erasing all program and data Flash
  blocks and verifying that all program and data Flash blocks are erased. */
/* CCOBIX end = 0 */
/* CCOB Params - NONE */
/* MGSTAT set if fault */

#define SET_USER_MARGIN_LEVEL    0x0D 
/*Specifies a user margin read level for all program Flash blocks. */
/* CCOBIX end = 1 */
/* CCOB Params - gpage, level setting (0-2) in CCOB[1] */
/* ACCERR set if invalid level */

#define SET_FIELD_MARGIN_LEVEL   0x0E 
/*Specifies a field margin read level for all program Flash blocks (special modes only). */
/* CCOBIX end = 1 */
/* CCOB Params - gpage, level setting (0-4) in CCOB[1] */
/* ACCERR set if invalid level */

/* **** P-Flash Only Commands ****/

#define ERASE_VERIFY_P_FLASH_SECTION 0x03  
/*Verify that a given number of words starting at the address provided are erased. */
/* CCOBIX end = 2 */
/* CCOB Params - global address, number of phrases in CCOB[2]*/
/* MGSTAT set if fault */

#define READ_ONCE	               0x04  
/* Read a phrase from a dedicated 64 word area in a hidden region of a programFlash block
   that was previously programmed using the Program Once command. */
/* CCOBIX end = 1 */
/* CCOB Params - read once index (0-3) in CCOB[1], phrase in CCOB [5:2] */
/* returns phrase in CCOB [4:1] */

#define PROGRAM_P_FLASH          0x06 
/* Program a phrase in a program Flash block and any previously loaded phrases for any
   other program Flash block (see Load Data Field command). */
/* CCOBIX end = 5 */
/* CCOB Params - global address, phrase in CCOB [5:2] */
/* MGSTAT set if fault, FPVIOL / ACCERR set where appropriate */

#define PROGRAM_ONCE             0x07 
/* Program a dedicated 64 word area in a hidden region of a program Flash block that is
   allowed to be programmed only once. */
/* CCOBIX end = 5 */
/* CCOB Params - read once index (0-3) in CCOB[1], phrase in CCOB [5:2] */
/* MGSTAT set if fault */

#define ERASE_P_FLASH_BLOCK      0x09 
/* Erase a program Flash block.
   An erase of the full program Flash block is only possible when FPLDIS, FPHDIS and
   FPOPEN bits in the FPROT register are set prior to launching the command. */
/* CCOBIX end = 1 */
/* CCOB Params - global address */
/* MGSTAT set if fault, FPVIOL / ACCERR set where appropriate */

#define ERASE_P_FLASH_SECTOR 0x0A 
/* Erase all bytes in a program Flash sector. */
/* CCOBIX end = 1 */
/* CCOB Params - global address */
/* MGSTAT set if fault, FPVIOL / ACCERR set where appropriate */

#define VERIFY_BACKDOOR_ACCESS_KEY 0x0C 
/*Supports a method of releasing MCU security by verifying a set of security keys. */
/* CCOBIX end = 4 */
/* CCOB Params - backdoor key in CCOB [1:4] */
/* ACCERR set if not verified */

/**** D-Flash Only Commands ****/
#define ERASE_D_FLASH_BLOCK      0x09 
/* Erase a program Flash block.
   An erase of the full program Flash block is only possible when DPOPEN bit in the DFPROT
     register is set prior to launching the command. */
/* CCOBIX end = 1 */
/* CCOB Params - global address */
/* MGSTAT set if fault, FPVIOL / ACCERR set where appropriate */

#define ERASE_VERIFY_D_FLASH_SECTION 0x10 
/* Verify that a given number of words starting at the address provided are erased. */
/* CCOBIX end = 2 */
/* CCOB Params - global address of first word, number of words to verify CCOB[2]*/
/* MGSTAT set if fault */

#define PROGRAM_D_FLASH         0x11 
/* Program up to four words in the data Flash block (see Load Data Field command). */
/* CCOBIX end = 2 */
/* CCOB Params - global address, up to 4 data words in CCOB [2:5] */
/* MGSTAT set if fault, EPVIOL / ACCERR set where appropriate */

#define ERASE_D_FLASH_SECTOR    0x12 
/* Erase all bytes in a data Flash sector. */
/* CCOBIX end = 2 */
/* CCOB Params - global address */
/* MGSTAT set if fault, EPVIOL  set where appropriate */

extern	Uint32_t	work_time_addr;
extern	Uint16_t	work_addr_addr;
extern bool DFLASH_SaveWorkTimeFlag;
extern int DFLASH_WorkTimeSavePos;

Uint8_t FLASH_LaunchCmd(Uint8_t params, Uint8_t command, Uint8_t ccob0, Uint16_t ccob1, Uint16_t ccob2, Uint16_t ccob3, Uint16_t ccob4, Uint16_t ccob5);
void 	FLASH_Init(void);

#endif