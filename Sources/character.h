/*
 * character.h
 *
 *  Created on: Jul 13, 2018
 *      Author: Administrator
 */

#ifndef CHARACTER_H_
#define CHARACTER_H_

//--------------------------------------------------------------------------------------------------
//-----------------------------Ӣ����ĸ�����֡����ţ�Size(H*W): 16*7--------------------------------
//--------------------------------------------------------------------------------------------------

//C
extern const unsigned char LCD_CharArray_16Row7Column_C[];
extern const unsigned char LCD_CharArray_24Row10Column_C[];
//D
extern const unsigned char LCD_CharArray_16Row7Column_D[];
extern const unsigned char LCD_CharArray_16Row8Column_D[];
extern const unsigned char LCD_CharArray_24Row10Column_D[];
//E
extern const unsigned char LCD_CharArray_16Row7Column_NumberSix_E[];
//F
extern const unsigned char LCD_CharArray_16Row7Column_F[];
extern const unsigned char LCD_CharArray_16Row8Column_F[];
extern const unsigned char LCD_CharArray_24Row9Column_F[];
//G
extern const unsigned char LCD_CharArray_16Row7Column_G[];
extern const unsigned char LCD_CharArray_16Row8Column_G[];
//h
extern const unsigned char LCD_CharArray_40Row19Column_h[];
extern const unsigned char LCD_CharArray_32Row14Column_h[];
extern const unsigned char LCD_CharArray_24Row12Column_h[];
extern const unsigned char LCD_CharArray_24Row9Column_h[];
extern const unsigned char LCD_CharArray_24Row9Column_H[];
extern const unsigned char LCD_CharArray_16Row7Column_h[];
extern const unsigned char LCD_CharArray_16Row8Column_h[];

//I
extern const unsigned char LCD_CharArray_16Row7Column_I[];
extern const unsigned char LCD_CharArray_16Row8Column_I[];
extern const unsigned char LCD_CharArray_24Row9Column_I[];
extern const unsigned char LCD_CharArray_16Row8Column_i[];

//K
extern const unsigned char LCD_CharArray_8Seg_16Com_K[];

//l
extern const unsigned char LCD_CharArray_24Row12Column_l[];
extern const unsigned char LCD_CharArray_24Row9Column_l[];
extern const unsigned char LCD_CharArray_24Row9Column_L[];
extern const unsigned char LCD_CharArray_8Seg_16Com_L[];

//M
extern const unsigned char LCD_CharArray_16Row7Column_M[];
extern const unsigned char LCD_CharArray_16Row8Column_M[];
extern const unsigned char LCD_CharArray_24Row9Column_M[];

extern const unsigned char LCD_CharArray_16Row8Column_m[];
//N
extern const unsigned char LCD_CharArray_16Row7Column_N[];
extern const unsigned char LCD_CharArray_16Row8Column_N[];
extern const unsigned char LCD_CharArray_24Row9Column_N[];
extern const unsigned char LCD_CharArray_16Row8Column_n[];
//n
extern const unsigned char LCD_CharArray_8Seg_16Com_n[];
//P
extern const unsigned char LCD_CharArray_16Row7Column_P[];
extern const unsigned char LCD_CharArray_16Row8Column_P[];
extern const unsigned char LCD_CharArray_24Row9Column_P[];
//R
extern const unsigned char LCD_CharArray_16Row7Column_R[];
extern const unsigned char LCD_CharArray_16Row7Column_NumberSix_R[];

extern const unsigned char LCD_CharArray_16Row8Column_R[];
//S
extern const unsigned char LCD_CharArray_16Row7Column_S[]; 
extern const unsigned char LCD_CharArray_16Row8Column_S[];
extern const unsigned char LCD_CharArray_24Row9Column_S[];
//U
extern const unsigned char LCD_CharArray_24Row10Column_U[];
//V
extern const unsigned char LCD_CharArray_7Row16Column_NumberSix_V[];
extern const unsigned char LCD_CharArray_8Row16Column_V[];

//0
extern const unsigned char LCD_DigitArray_16Row7Column_NumberSix_0[];
extern const unsigned char LCD_DigitArray_16Row8Column_0[];
//1
extern const unsigned char LCD_DigitArray_16Row7Column_NumberSix_1[];
extern const unsigned char LCD_DigitArray_16Row8Column_1[];
//2
extern const unsigned char LCD_DigitArray_16Row7Column_2[];
//3
extern const unsigned char LCD_DigitArray_16Row7Column_3[];
//4
extern const unsigned char LCD_DigitArray_16Row7Column_4[];
//5
extern const unsigned char LCD_DigitArray_16Row7Column_5[];
//6
extern const unsigned char LCD_DigitArray_16Row7Column_6[];
//7
extern const unsigned char LCD_DigitArray_16Row7Column_7[];
//8
extern const unsigned char LCD_DigitArray_16Row7Column_8[];
//9
extern const unsigned char LCD_DigitArray_16Row7Column_9[];

extern const unsigned char LCD_DigitArray_16Row7Column[10][14];
extern const unsigned char LCD_DigitArray_16Row8Column[10][16];
extern const unsigned char LCD_DigitArray_24Row12Column[10][24 / 8 * 12];
extern const unsigned char LCD_DigitArray_16Seg_32Com[10][32 / 8 * 16];
extern const unsigned char LCD_DigitArray_12Seg_24Com[10][24 / 8 * 12];
extern const unsigned char LCD_DigitArray_40Row19Column[10][40 / 8 * 19];
extern const unsigned char LCD_DigitArray_24Row9Column[10][24 / 8 * 9];

//�ո�
extern const unsigned char LCD_CharArray_16Row7Column_Space[];
//����
extern const unsigned char LCD_CharArray_16Row7Column_Minus[];
extern const unsigned char LCD_CharArray_8Seg16Com_Minus[];

//%
extern const unsigned char LCD_CharArray_16Row7Column_Percent[];
//.
extern const unsigned char LCD_CharArray_16Row7Column_Dot[];
extern const unsigned char LCD_CharArray_16Row8Column_Dot[];
extern const unsigned char LCD_CharArray_40Row19Column_Dot[];
extern const unsigned char LCD_CharArray_24Row9Column_Dot[];
extern const unsigned char LCD_CharArray_24Row12Column_Dot[];

//��
extern const unsigned char LCD_CharArray_8Seg16Com_Celsius_Left[];
extern const unsigned char LCD_CharArray_8Seg16Com_Celsius_Right[];

// /
extern const unsigned char LCD_CharArray_16Row7Column_Slash[];
extern const unsigned char LCD_CharArray_16Row8Column_Slash[];
extern const unsigned char LCD_CharArray_24Row9Column_Slash[];

// :
extern const unsigned char LCD_CharArray_16Row6Column_Colon[];
extern const unsigned char LCD_CharArray_16Row7Column_Colon[];
extern const unsigned char LCD_CharArray_16Row8Column_Colon[];
extern const unsigned char LCD_CharArray_24Row9Column_Colon[];

//��
extern const unsigned char LCD_CharArray_24Row18Column_Celsius[];


//--------------------------------------------------------------------------------------------------
// 6�����壬height * width = 16 * 13
//--------------------------------------------------------------------------------------------------
//A
extern const unsigned char LCD_ChineseCharArray_16Row13Column_An[];	    	//��
//ƴ����ĸ��C����ͷ
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Chang[];		//��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Che[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Cheng[];		//��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Chi[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Chong[];    //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ci[];			  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Chuan[];	  //��

//ƴ����ĸ��D����ͷ
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Dai[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Dai_1[];    //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Dao[];      //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Deng[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Dian[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ding[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_De[];       //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Di[];       //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Dong[];     //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Du[];       //��

//E
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Er[];		    //��

//F
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Fa[];       //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Fa_1[];     //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Fang[];     //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Fu[];       //��

//G
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Gan[]; 	    //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Gao[]; 	    //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Gao_1[]; 	  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Gu[];		    //��
extern const unsigned char LCD_ChineseCharArray_24Row10Column_Gu[];

//H
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Hao[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Hou[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Hou_1[];		//��

//J
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ji[];	  	  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ji_1[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ji_2[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Jian[];     //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Jiang[];    //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Jiao[];		  //У
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Jing[];	  	//��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Jing_1[];	  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Jin[];      //��


//K
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Kong[];	  	//��

//L
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Leng[];	  	//��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Liang[];	  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Li[];       //��

//M
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Men[];		  //��

//N
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Niu[];		  //Ť

//Q��ͷ
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Qi[];	  	  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Qi_1[];	  	//��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Qing[];     //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Quan[];     //ȫ
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Que[];      //ȴ

//S
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Shang[];		//��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Shao[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Shi[];		  //ʼ
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Shu[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Su[];		    //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Sheng[];		//��

//T
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Tian[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ting[];		  //ͣ
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Tong[];     //ͨ

//X
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Xia[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Xian[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Xin[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Xu[];		    //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Xing[];        //��

//W
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Wan[];	    //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Wen[];      //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Wu[];       //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Wu_1[];     //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Wu_2[];     //��

//Y
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ya[];       //ѹ

extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ye[];       //Һ
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yi[];       //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yi_1[];     //һ
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yi_2[];     //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yi_3[];     //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yi_4[];     //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_You[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_You_1[];		//��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yu[];		    //��

extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yuan[];     //ԭ

//Z��ͷ
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zai[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zhang[];		//��
extern const unsigned char LCD_ChineseCharArray_24Row20Column_Zhang[];
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zheng[];		//��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zheng_1[];	//��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zhi[];	  	//��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zhi_1[];	  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zhu[];		  //ע
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zhun[];		  //׼
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zi[];       //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zong[];     //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zai_1[];		  //��
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zhong[];     //��

extern const unsigned char LCD_Hourglass[72];
extern const unsigned char gImage_hourglass[32];
extern const unsigned char gImage_alarm[32];
extern const unsigned char gImage_alarm_10_10[24];
extern const unsigned char gImage_alarm_8_8[8];
extern const unsigned char gImage_logoSDLG[512];
extern const unsigned char gImage_Hourglass_16_12[24];

#endif /* CHARACTER_H_ */
