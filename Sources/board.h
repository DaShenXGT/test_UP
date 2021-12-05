#ifndef	_BOARD_H
#define	_BOARD_H

//数据类型定义		
#define bool unsigned char

#define true  1 
#define false 0
#define TRUE  1  
#define FALSE 0

//版本号1.0
#define  VER_INFO  10  

#define  CAN_COMM		1
//在此处选择是外部看门狗，还是软件看门狗
#define	WDT_ON		      	1
//#define OUT_WDT 1
#ifdef OUT_WDT
extern void En_out_WatchDog(void); 
extern void Feed_WatchDog(void);
#endif

#define ENGINE_TYPE_WEICHAI 1
//#define ENGINE_TYPE_YUNNEI 1


//车速比设置 By周文熙 20200731
extern unsigned long SPEEDRATIO;


//报警标志
typedef union
{
    unsigned long std;
    struct 
    {
        //Byte3
        unsigned long distantLight          :1;   //远光
        unsigned long preheat               :1;   //预热
        unsigned long postProcessingFault   :1;   //后处理故障
        unsigned long ureaLevelLow          :1;   //尿素液位低
        unsigned long		oil_press		        :1;		//油压状态
        unsigned long		brake_press		      :1;		//制动压力
        unsigned long		box_press		        :1;		//变速箱压力
        unsigned long		water_temp_h	      :1;		//水温高
        
        //Byte2
        unsigned long		power_charge	      :1;		//充电
        unsigned long		oil_water		        :1;		//油水分离
        unsigned long		oil_block		        :1;		//燃油粗滤
        unsigned long		drive_block		      :1;		//传动油粗滤
        unsigned long		oil_level_l		      :1;		//油位低
        unsigned long		net_block		        :1;		//空滤阻塞
        unsigned long		engineGeneralFault  :1;		//发动机故障
        unsigned long		engineSevereFault   :1;		//发动机故障
        
        //Byte1
        unsigned long		speed_err		        :1;		//发动机防拆
        unsigned long		left_turn		        :1;		//左转弯
        unsigned long		right_turn		      :1;		//右转弯
        unsigned long		hand_brake		      :1;		//手制动     			
        unsigned long		ign_on			        :1;		//电锁开
        unsigned long		backlight_on	      :1;		//背光开	
        unsigned long		flicher_bit		      :1;		//闪烁	
        unsigned long		buzz_bit		        :1;		//蜂鸣器
        
        //Byte0
        unsigned long		lcd_flicher_bit	    :1;		//
        unsigned long		pulse_bit		        :1;		//
        unsigned long		motor_zero_bit	    :1;		//
        unsigned long   mute			          :1;	        	
        unsigned long   chinese			        :1;   
        unsigned long		oil_level_limit		  :1;		//油位低限制
        unsigned long   can_warn            :1;      
                         
    } b;
    
} vehicle_warning;


//任务激活标志
typedef union
{
	unsigned int w;
	struct 
	{
	  //Byte1
	  unsigned int coolingLiquidTempProcessFlag :1;
	  unsigned int ureaLevelProcessFlag         :1;
		unsigned int 	Atd_active		              :1;			//ADC0采集激活标识
		unsigned int	Ditigal_active	            :1;			//数字量采集激活标识
		unsigned int	Led_active		              :1;			//LED驱动激活标识
		unsigned int	Lcd_active		              :1;			//LCD激活标识
		unsigned int	Time_active		              :1;
		unsigned int	Flash_active	              :1;
		//Byte0
		unsigned int	Buzz_active		              :1;			//蜂鸣器激活标识
		unsigned int	motor_active	              :1;			//马达1激活标识
		unsigned int	sysPowerOnFlag	            :1;			//马达1激活标识
		unsigned int	Pulse_active	              :1;			//转速处理激活标识
		unsigned int	KD_active	                  :1;			//KD处理激活标识
		unsigned int	motor5_active	              :1;			//马达1激活标识
	  unsigned int	Lcd_error_active            :1;
	  
	} b;
	
} queue_list;

typedef   struct 
{
	unsigned char		Atd_id;						//通道号
	unsigned char		samp_time;
	unsigned int		samp_value[5];
	unsigned int		old_adc_value;				//上一次ADC采样值
	unsigned int		new_adc_value;				//本次ADC采样值
	unsigned int		old_convert_value;			//上次AD置处理后数据
	unsigned int		new_convert_value;			//本次AD置处理后数据
	unsigned int		supply_value;
}vehicle_ad;

typedef   struct 
{
	unsigned char		check_time;
	unsigned long	old_status;					//原数字量状态
	unsigned long	new_status;					//新数字量状态
  unsigned long validBits; //抖动位

}vehicle_digital;

typedef   struct 
{
	//unsigned long		old_pulse_freq;			//上次脉冲频率
	//unsigned long		new_pulse_freq;			//上次脉冲频率
	unsigned long		old_speed;					//上次转速数据
	unsigned long		new_speed;					//本次转速数据
	unsigned char		old_motor_angle;			//原来马达角度
	unsigned char		new_motor_angle;			//新马达角度
}engineSpeedParams_t;

typedef   struct 
{
	unsigned char		motor_first;
	unsigned char		motor_dir;					//
	unsigned char		old_motor_angle;			//原来马达角度
	unsigned char		new_motor_angle;			//新马达角度
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
