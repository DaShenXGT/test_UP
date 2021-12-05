#ifndef	_ATD_H
#define	_ATD_H

#define		ADC_MAX_CHANNEL			8//9//5
#define		ADC_CONVERT_TIME		5
#define     ADC_HOLD_TIME			10

#define 	ATD_TIME		80

#define ATD_CHANNEL_TRANS_OIL_LOW       0
#define ATD_CHANNEL_FUEL_LEVEL          1
#define ATD_CHANNEL_CHARGING_INDICATOR  2
#define ATD_CHANNEL_TRANS_OIL_TEMP      3
#define ATD_CHANNEL_AVDD                4
#define ATD_CHANNEL_AI_IN5              5
#define ATD_CHANNEL_AN06                6
#define ATD_CHANNEL_BATTERY_VOLTAGE     7

extern	vehicle_ad		mnuich_atd[ADC_MAX_CHANNEL/*+1*/];
extern	Uint8_t			atd_channel_id;

void	Init_Atd_Paramter(void);
void	atd_handle(void);


#endif
