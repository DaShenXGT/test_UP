#ifndef _time1_capture_h_
#define _time1_capture_h_

#define bool unsigned char

#define PULSE_EDGE_MAX_NUM 50

struct  loader//装载机车速信号结构体
{
 unsigned char pulse_overflow[PULSE_EDGE_MAX_NUM];//防止在捕捉过程中产生了溢出导致的后面一个数据比前面一个小。此值记录2个边沿直接的溢出次数。此值是为了计算很小的频率测量。拓宽频率的测量范围。
 unsigned int  pulse_width[PULSE_EDGE_MAX_NUM];
 unsigned int  signal_pulse_count;
 unsigned int  pulse_sample_finish;
 unsigned long  loader_speed;
 unsigned long  pulse_frequency;

};


extern struct loader car_speed_parameter ;

void  tim1_ioc1_6_input_capture_init(void);

void restart_ioc1_6_capture(void);

void  calculating_speed(void); 


#endif
