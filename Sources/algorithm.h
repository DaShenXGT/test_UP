#ifndef __ALGORITHM_H__
#define __ALGORITHM_H__

#include "app.h"
#include "board.h"

#define NANDOM				2


#define BIG_TO_LITTLE 0
#define LITTLE_TO_BIG 1

//Ã°ÅÝÅÅÐò
void bubbleSort(unsigned long *p, unsigned int num, unsigned char sortMode);

//¼ì²âÖµÊÇ·ñ¸Ä±ä
bool Algo_ValueChangeCheck(unsigned long newVar, unsigned long oldVar, unsigned long threshold);
bool Algo_GetMajority(int *pData, int *majority, unsigned int len, int percent);

unsigned long int Hash(unsigned char* pucPara);	  

unsigned char reverseByteOrder(unsigned char byte);

unsigned int Algo_CrcCheck(unsigned int crcReg, unsigned int num, unsigned char *p);


#endif