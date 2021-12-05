#ifndef	_EEPROM_H
#define	_EEPROM_H

extern void EEPROM_IIC_Init(void);
//extern void EEPROM_Test(void);
extern unsigned char eeprom_rcv_byte(unsigned char devaddr, unsigned long romaddr, unsigned char *s, unsigned int num);
//用来向EEPROM中存储数据
extern unsigned char eeprom_send_byte(unsigned long romaddr, unsigned char *s, unsigned int num);
#endif
