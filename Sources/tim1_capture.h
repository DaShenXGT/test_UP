#ifndef _time1_capture_h_
#define _time1_capture_h_

#define bool unsigned char

#define PULSE_EDGE_MAX_NUM 50

struct  loader//װ�ػ������źŽṹ��
{
 unsigned char pulse_overflow[PULSE_EDGE_MAX_NUM];//��ֹ�ڲ�׽�����в�����������µĺ���һ�����ݱ�ǰ��һ��С����ֵ��¼2������ֱ�ӵ������������ֵ��Ϊ�˼����С��Ƶ�ʲ������ؿ�Ƶ�ʵĲ�����Χ��
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
