#ifndef	_DISPIAY_H
#define	_DISPIAY_H

#define	LCD_ON		1
#define	D4B3		1
#define	LCDINWAIT	0

#define	ALL_REDUCED_DRIVE	0xff
#define	ALL_PULLS_ON		0xff


extern  Uint8_t dis_data[7];

void LCD_Init(void);
void LCD_RAM_Clear_All(void);
void Dispaly_Drive(Uint8_t dis_id,Uint8_t *dtat_point);
#endif