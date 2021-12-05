#include "algorithm.h"

/***************************************************************
@ Func: 冒泡排序
***************************************************************/
void bubbleSort(unsigned long *p, unsigned int num, unsigned char sortMode)
{
	unsigned int i, j;
	unsigned long temp;
	
	for(i = 0; i < (num - 1); i++)
	{
		for(j = 0; j < (num -1 - i); j++)
		{
			switch(sortMode)
			{
				case BIG_TO_LITTLE:
					if(p[j] < p[j + 1])
					{
						temp = p[j];
						p[j] = p[j + 1];
						p[j + 1] = temp;
					}
				break;
				case LITTLE_TO_BIG:
					if(p[j] > p[j + 1])
					{
						temp = p[j];
						p[j] = p[j + 1];
						p[j + 1] = temp;
					}
				break;
				default:
				break;
			}
		}
	}
}

/***************************************************************
@ Func: 检测值是否改变
***************************************************************/
bool Algo_ValueChangeCheck(unsigned long newVar, unsigned long oldVar, unsigned long threshold)
{
  if(((newVar > oldVar) && ((newVar - oldVar) >= threshold)) || ((oldVar > newVar) && ((oldVar - newVar) >= threshold)))
  {
    return true;
  }
  
  return false;
}

/***************************************************************
@ Func:  按多数原则获取值
@ Param: pData -- 数据缓冲区指针
         majority -- 
         len -- 处理长度
         percent -- 百分比要求
***************************************************************/
bool Algo_GetMajority(int *pData, int *majority, unsigned int len, int percent)
{
  unsigned int i = 0;
  unsigned int j = 0;
  int maxNum = 0;
  int specNum = 0;
  int data = 0;
  
  for(i = 0; i < len; ++i)
  {
    specNum = 0;
    
    for(j = 0; j < len; ++j)
    {
      if(pData[i] == pData[j])
      {
        ++specNum;
      }
    }
    
    if(specNum > maxNum)
    {
      maxNum = specNum;
      data = pData[i];
    }
  }
  
  if((maxNum * 100 / len) >= percent)
  {
    *majority = data;
    
    return true;
  }
  
  return false;
}


//******************************************************************************
// Method     : Hash
// Description: 计算4个字节的Hash码
// Access     : 
// Returns    : 返回4个字节的Hash码
// Input      : 首地址
// Output	  : 
// Note	      : 
// Fiexed	  : [4/19/2013 rookie li] 
//******************************************************************************
unsigned long int Hash(unsigned char* pucPara)
{
	unsigned long int ulTemp = 0;
	unsigned char i;

	for (i=0;i<4;i++)
	{
		ulTemp = NANDOM * ulTemp + *(pucPara+i);
	}
	ulTemp = ulTemp %  1000000;
	return ulTemp;
}

unsigned char reverseByteOrder(unsigned char byte)
{
	unsigned char result = 0;
	unsigned char i = 0;
	
	for(i = 0; i < 4; ++i)
	{
		result |= ((byte & (1 << i)) << (7 - (i * 2)));
		result |= ((byte & (1 << (7 - i))) >> (7 - (i * 2)));
	}
	
	return result;
}

/***********************************************************************
* 功能: 计算一包数据的校验码 520字节
@ Brief: CRC校验涵盖单包数据信息帧（8 Bytes)和数据帧(512 Bytes）
/***********************************************************************/
 unsigned int Algo_CrcCheck(unsigned int crcReg, unsigned int num, unsigned char *p)
 {
	 unsigned int i = 0;
	 unsigned char j = 0;
	 unsigned int ret = 0;
	 
	 for(i = 0; i < num; ++i)
	 {
	   crcReg ^= p[i];
	   
	   for(j = 0; j < 8; ++j)
	   {
	     if(crcReg & 0x01)
	     {
	       crcReg >>= 1;
	       crcReg ^= 0xa001;
	     }
	     else
	     {
	       crcReg >>= 1;
	     }
	   }
	 }
	 
	 //GPS校验码没有进行高低字节交换，此处需要试验
	 //ret = crcReg >> 8;
	 //ret = ret + (crcReg << 8);
	 ret = crcReg;
	 
	 return ret;
 } 





