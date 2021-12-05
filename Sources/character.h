/*
 * character.h
 *
 *  Created on: Jul 13, 2018
 *      Author: Administrator
 */

#ifndef CHARACTER_H_
#define CHARACTER_H_

//--------------------------------------------------------------------------------------------------
//-----------------------------英文字母、数字、符号，Size(H*W): 16*7--------------------------------
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

//空格
extern const unsigned char LCD_CharArray_16Row7Column_Space[];
//减号
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

//℃
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

//℃
extern const unsigned char LCD_CharArray_24Row18Column_Celsius[];


//--------------------------------------------------------------------------------------------------
// 6号字体，height * width = 16 * 13
//--------------------------------------------------------------------------------------------------
//A
extern const unsigned char LCD_ChineseCharArray_16Row13Column_An[];	    	//安
//拼音字母“C”开头
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Chang[];		//常
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Che[];		  //车
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Cheng[];		//成
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Chi[];		  //池
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Chong[];    //充
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ci[];			  //次
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Chuan[];	  //传

//拼音字母“D”开头
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Dai[];		  //待
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Dai_1[];    //带
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Dao[];      //到
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Deng[];		  //等
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Dian[];		  //电
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ding[];		  //定
extern const unsigned char LCD_ChineseCharArray_16Row13Column_De[];       //的
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Di[];       //地
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Dong[];     //动
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Du[];       //度

//E
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Er[];		    //二

//F
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Fa[];       //法
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Fa_1[];     //发
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Fang[];     //方
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Fu[];       //服

//G
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Gan[]; 	    //感
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Gao[]; 	    //高
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Gao_1[]; 	  //告
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Gu[];		    //故
extern const unsigned char LCD_ChineseCharArray_24Row10Column_Gu[];

//H
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Hao[];		  //号
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Hou[];		  //候
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Hou_1[];		//后

//J
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ji[];	  	  //级
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ji_1[];		  //机
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ji_2[];		  //即
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Jian[];     //检
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Jiang[];    //将
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Jiao[];		  //校
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Jing[];	  	//警
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Jing_1[];	  //经
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Jin[];      //进


//K
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Kong[];	  	//控

//L
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Leng[];	  	//冷
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Liang[];	  //量
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Li[];       //立

//M
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Men[];		  //门

//N
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Niu[];		  //扭

//Q开头
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Qi[];	  	  //器
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Qi_1[];	  	//启
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Qing[];     //请
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Quan[];     //全
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Que[];      //却

//S
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Shang[];		//上
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Shao[];		  //稍
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Shi[];		  //始
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Shu[];		  //数
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Su[];		    //速
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Sheng[];		//生

//T
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Tian[];		  //天
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ting[];		  //停
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Tong[];     //通

//X
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Xia[];		  //下
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Xian[];		  //限
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Xin[];		  //信
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Xu[];		    //需
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Xing[];        //行

//W
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Wan[];	    //完
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Wen[];      //温
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Wu[];       //勿
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Wu_1[];     //无
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Wu_2[];     //务

//Y
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ya[];       //压

extern const unsigned char LCD_ChineseCharArray_16Row13Column_Ye[];       //液
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yi[];       //意
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yi_1[];     //一
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yi_2[];     //已
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yi_3[];     //移
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yi_4[];     //异
extern const unsigned char LCD_ChineseCharArray_16Row13Column_You[];		  //油
extern const unsigned char LCD_ChineseCharArray_16Row13Column_You_1[];		//有
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yu[];		    //于

extern const unsigned char LCD_ChineseCharArray_16Row13Column_Yuan[];     //原

//Z开头
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zai[];		  //在
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zhang[];		//障
extern const unsigned char LCD_ChineseCharArray_24Row20Column_Zhang[];
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zheng[];		//正
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zheng_1[];	//整
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zhi[];	  	//制
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zhi_1[];	  //至
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zhu[];		  //注
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zhun[];		  //准
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zi[];       //自
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zong[];     //总
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zai_1[];		  //再
extern const unsigned char LCD_ChineseCharArray_16Row13Column_Zhong[];     //中

extern const unsigned char LCD_Hourglass[72];
extern const unsigned char gImage_hourglass[32];
extern const unsigned char gImage_alarm[32];
extern const unsigned char gImage_alarm_10_10[24];
extern const unsigned char gImage_alarm_8_8[8];
extern const unsigned char gImage_logoSDLG[512];
extern const unsigned char gImage_Hourglass_16_12[24];

#endif /* CHARACTER_H_ */
