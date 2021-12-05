#ifndef	_APP_H
#define	_APP_H

#define		VERSION		20

#define _pasti(i)	((unsigned int )(jiffies - i))

typedef unsigned char	Uint8_t;   /*unsigned 8 bit definition */
typedef unsigned int	Uint16_t;   /*unsigned 16 bit definition*/
typedef unsigned long	Uint32_t;   /*unsigned 32 bit definition*/
typedef signed char		Sint8_t;   /*unsigned 8 bit definition */
typedef signed int		Sint16_t;   /*unsigned 16 bit definition*/
typedef signed long		Sint32_t;   /*unsigned 32 bit definition*/
typedef unsigned int UINT16;

//extern	queue_list	systemTask;

#define	TRANS_ID_0	264  //262	//3.7	//268
#define	TRANS_ID_1	226	//4.3	//229     2015-2-10 14:42:40
#define	TRANS_ID_2	238	//4.1	//243
#define	TRANS_ID_3	238 //354	//2.7	//3602015-2-10 14:42:25  182
#define	TRANS_ID_4	304	//3.2	//310
#define	TRANS_ID_5	303	//3.1	//300    2015-2-10 14:42:47
#define	TRANS_ID_6	280	//4.7	//211		953N
#define	TRANS_ID_7	280 //280	//2.4	//287   206


extern	Uint8_t		run_mode;
extern	Uint16_t	enging_paramter;
void Runing_Model(void);
void Enging_Select(void);
#endif