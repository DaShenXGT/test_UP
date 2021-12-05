#ifndef	_TIME_H
#define	_TIME_H

#define OSCCLK      8     // Oscillator Clock in MHz
#define BUSCLK      32    // Bus Clock in MHz
#define TIMERCLK    4     // Timer Clock in MHz
#define MCPWMCLK    16    // MCPWM Clock in MHz
// Timebase definitions
#define _50us     50 * TIMERCLK
#define _100us   100 * TIMERCLK
#define _200us   200 * TIMERCLK
#define _250us   250 * TIMERCLK
#define _280us   280 * TIMERCLK
#define _300us   300 * TIMERCLK
#define _350us   350 * TIMERCLK
#define _400us   400 * TIMERCLK
#define _500us   500 * TIMERCLK
#define _600us   600 * TIMERCLK
#define _700us   700 * TIMERCLK
#define _750us   750 * TIMERCLK
#define _800us   800 * TIMERCLK
#define _900us   900 * TIMERCLK
#define _1ms    1000 * TIMERCLK
#define _2ms    2000 * TIMERCLK
#define _3ms    3000 * TIMERCLK
#define _5ms    5000 * TIMERCLK
#define _8ms    8000 * TIMERCLK
#define _10ms  10000 * TIMERCLK

#define 	VALUE_100MS		100 
#define 	VALUE_500MS		500 
#define 	VALUE_1000MS	1000

#define 	DITIGAL_TIME	20
//#define 	LED_REFRESH_TIME	2000  //liu~
#define 	PULSE_VALUE		500

#define		TIME1S_VALID	  0x01
#define		TIME500mS_VALID	0x80

#define		TIME_1M_CONST	60
#define		TIME_6M_CONST	6

extern 	Uint32_t 	jiffies;
extern	Uint16_t	working_time_h;
extern	Uint16_t	working_time_l;
extern	unsigned long	working_hours;
extern	unsigned long	working_time;
extern 	Uint8_t		time_6m;
extern 	Uint8_t		time_1m;
//extern 	Uint32_t	enging_on_time;
extern 	vehicle_time	munich_time;

void Init_Time_Params(void);
void timeEventsProc(void);
void DelayMS(Uint16_t delay);
void TIM0_Init(void);
void TIM1_Init(void);


#endif