#ifndef	_BOARD_H
#define	_BOARD_H

//�������Ͷ���		
#define bool unsigned char

#define true  1 
#define false 0
#define TRUE  1  
#define FALSE 0

//�汾��1.0
#define  VER_INFO  10  

#define  CAN_COMM		1
//�ڴ˴�ѡ�����ⲿ���Ź�������������Ź�
#define	WDT_ON		      	1
//#define OUT_WDT 1
#ifdef OUT_WDT
extern void En_out_WatchDog(void); 
extern void Feed_WatchDog(void);
#endif

#define ENGINE_TYPE_WEICHAI 1
//#define ENGINE_TYPE_YUNNEI 1


//���ٱ����� By������ 20200731
extern unsigned long SPEEDRATIO;


//������־
typedef union
{
    unsigned long std;
    struct 
    {
        //Byte3
        unsigned long distantLight          :1;   //Զ��
        unsigned long preheat               :1;   //Ԥ��
        unsigned long postProcessingFault   :1;   //�������
        unsigned long ureaLevelLow          :1;   //����Һλ��
        unsigned long		oil_press		        :1;		//��ѹ״̬
        unsigned long		brake_press		      :1;		//�ƶ�ѹ��
        unsigned long		box_press		        :1;		//������ѹ��
        unsigned long		water_temp_h	      :1;		//ˮ�¸�
        
        //Byte2
        unsigned long		power_charge	      :1;		//���
        unsigned long		oil_water		        :1;		//��ˮ����
        unsigned long		oil_block		        :1;		//ȼ�ʹ���
        unsigned long		drive_block		      :1;		//�����ʹ���
        unsigned long		oil_level_l		      :1;		//��λ��
        unsigned long		net_block		        :1;		//��������
        unsigned long		engineGeneralFault  :1;		//����������
        unsigned long		engineSevereFault   :1;		//����������
        
        //Byte1
        unsigned long		speed_err		        :1;		//����������
        unsigned long		left_turn		        :1;		//��ת��
        unsigned long		right_turn		      :1;		//��ת��
        unsigned long		hand_brake		      :1;		//���ƶ�     			
        unsigned long		ign_on			        :1;		//������
        unsigned long		backlight_on	      :1;		//���⿪	
        unsigned long		flicher_bit		      :1;		//��˸	
        unsigned long		buzz_bit		        :1;		//������
        
        //Byte0
        unsigned long		lcd_flicher_bit	    :1;		//
        unsigned long		pulse_bit		        :1;		//
        unsigned long		motor_zero_bit	    :1;		//
        unsigned long   mute			          :1;	        	
        unsigned long   chinese			        :1;   
        unsigned long		oil_level_limit		  :1;		//��λ������
        unsigned long   can_warn            :1;      
                         
    } b;
    
} vehicle_warning;


//���񼤻��־
typedef union
{
	unsigned int w;
	struct 
	{
	  //Byte1
	  unsigned int coolingLiquidTempProcessFlag :1;
	  unsigned int ureaLevelProcessFlag         :1;
		unsigned int 	Atd_active		              :1;			//ADC0�ɼ������ʶ
		unsigned int	Ditigal_active	            :1;			//�������ɼ������ʶ
		unsigned int	Led_active		              :1;			//LED���������ʶ
		unsigned int	Lcd_active		              :1;			//LCD�����ʶ
		unsigned int	Time_active		              :1;
		unsigned int	Flash_active	              :1;
		//Byte0
		unsigned int	Buzz_active		              :1;			//�����������ʶ
		unsigned int	motor_active	              :1;			//���1�����ʶ
		unsigned int	sysPowerOnFlag	            :1;			//���1�����ʶ
		unsigned int	Pulse_active	              :1;			//ת�ٴ������ʶ
		unsigned int	KD_active	                  :1;			//KD�������ʶ
		unsigned int	motor5_active	              :1;			//���1�����ʶ
	  unsigned int	Lcd_error_active            :1;
	  
	} b;
	
} queue_list;

typedef   struct 
{
	unsigned char		Atd_id;						//ͨ����
	unsigned char		samp_time;
	unsigned int		samp_value[5];
	unsigned int		old_adc_value;				//��һ��ADC����ֵ
	unsigned int		new_adc_value;				//����ADC����ֵ
	unsigned int		old_convert_value;			//�ϴ�AD�ô��������
	unsigned int		new_convert_value;			//����AD�ô��������
	unsigned int		supply_value;
}vehicle_ad;

typedef   struct 
{
	unsigned char		check_time;
	unsigned long	old_status;					//ԭ������״̬
	unsigned long	new_status;					//��������״̬
  unsigned long validBits; //����λ

}vehicle_digital;

typedef   struct 
{
	//unsigned long		old_pulse_freq;			//�ϴ�����Ƶ��
	//unsigned long		new_pulse_freq;			//�ϴ�����Ƶ��
	unsigned long		old_speed;					//�ϴ�ת������
	unsigned long		new_speed;					//����ת������
	unsigned char		old_motor_angle;			//ԭ�����Ƕ�
	unsigned char		new_motor_angle;			//�����Ƕ�
}engineSpeedParams_t;

typedef   struct 
{
	unsigned char		motor_first;
	unsigned char		motor_dir;					//
	unsigned char		old_motor_angle;			//ԭ�����Ƕ�
	unsigned char		new_motor_angle;			//�����Ƕ�
}vehicle_motor;

typedef   struct 
{
	unsigned int	oid_press_on_time;
	unsigned int	oid_press_off_time;
	unsigned int	ign_on_time;
	unsigned int	ign_off_time;
	unsigned int	enging_on_time;
	unsigned int	enging_off_time;
	
}vehicle_time;

#endif
