#ifndef	_SMC_MOTOR_H
#define	_SMC_MOTOR_H

#define PWMPER         0x3FF // 10-bit PWMs
#define PWMPER11       0xFF // 8-bit PWMs
#define TABLELENGTH    512   // 512 point SIN table
#define SAMPLE_RATE    1000  // Motor Algorithm executes every 1ms
#define HY

//#define MOVE_PERIOD         100000							//ɾ�� rookie_lxp
#define MAX_STEP_PERIOD_0		3333//6666//4000//20000			//�޸�rookie_lxp
#define MIN_STEP_PERIOD_0		820//800    //2016.05.09  Jzhua//��������ʹ��
#define MIN_STEP_PERIOD_01		740//606//667//750//606//750// 720   //2016.05.09  Jzhua//����ʹ��
#define STEP_TIME_01					10//100//300			//ǿ�����Ƕȹ���ʹ��
//#define STEP_TIME_02					20//100//600
#define STEP_TIME_0						10//100//200
#define DEC_STEP_TIME_0     (MAX_STEP_PERIOD_0-MIN_STEP_PERIOD_0)/STEP_TIME_0 //15----rookie_lxp

#define MAX_STEP_PERIOD			20000
#define MIN_STEP_PERIOD			6000
#define STEP_TIME							200
#define DEC_STEP_TIME     (MAX_STEP_PERIOD-MIN_STEP_PERIOD)/STEP_TIME		//10---rookie_lxp
/*
#define MAX_STEP_PERIOD    30000
#define MIN_STEP_PERIOD    5000
#define STEP_TIME          250
#define DEC_STEP_TIME     (MAX_STEP_PERIOD-MIN_STEP_PERIOD)/STEP_TIME
*/
#define DEFAULT_DECAY     40
#define DEFAULT_CMDVEL    300

#define MCDC_SIGN         0x8000
#define FULL_STEP_NUM     24
#define ONE_DEGREES_NUM   12
#define MOTOR_ACC_VALUE   4
#define MOTOR_DEC_VALUE   4
#define ACC_STEP_NUM      6
#define DEC_STEP_NUM      6

//���ת������
enum
{
  CCW,
  CW
};

//���������Ȧ����
#define MOTOR_0_COIL1    MCDC0       
#define MOTOR_0_COIL2    MCDC1  

#define MOTOR_1_COIL1    MCDC2   
#define MOTOR_1_COIL2    MCDC3   

#define MOTOR_2_COIL1    MCDC4     
#define MOTOR_2_COIL2    MCDC5    

#define MOTOR_3_COIL1     MCDC6     
#define MOTOR_3_COIL2     MCDC7

#define MOTOR_4_COIL1_1 PWMDTY0            
#define MOTOR_4_COIL1_2 PWMDTY1
#define MOTOR_4_COIL2_1 PWMDTY2
#define MOTOR_4_COIL2_2 PWMDTY3

#define MOTOR_4_COIL1_1 PWMDTY0
#define MOTOR_4_COIL1_2 PWMDTY1
#define MOTOR_4_COIL2_1 PWMDTY2
#define MOTOR_4_COIL2_2 PWMDTY3

#define MOTOR4_PIN1 PWMDTY2
#define MOTOR4_PIN2 PWMDTY3
#define MOTOR4_PIN3 PWMDTY1
#define MOTOR4_PIN4 PWMDTY0  

//����ͷ�����������
#define MOTOR_ROTATION_SPEED      0
#define MOTOR_COOLING_LIQUID_TEMP 1
#define MOTOR_TRANS_OIL_TEMP      2
#define MOTOR_UREA_LEVEL          3
#define MOTOR_FUEL_LEVEL          4

 
//��ͷ���Ƕ�
#define MOTOR_ENGINE_SPEED_MAX_ANGLE    235
#define MOTOR_FUEL_LEVEL_MAX_ANGLE      90
#define MOTOR_COOLING_LIQUID_MAX_ANGLE  130
#define MOTOR_TRANS_OIL_TEMP_MAX_ANGEL  130
#define MOTOR_UREA_LEVEL_MAX_ANGLE      90

#define u8  unsigned char
#define u16 unsigned int
#define u32 unsigned long
#define s8  signed char
#define s16 signed int
#define s32 signed long



typedef struct 
{
    
// variables for normal motor movement
    u8    u8_mtr;
    u8    u8_coilangle;
    u8    u8_pwmchan;         //������
    u8    u8_run_status;      //����״̬,0--ֹͣ���У�1--�������У�2--��������;3--��������;4--˥�����̣�
    u8    u8_decayval;        //˥��ֵ
    u8    u8_dir;             //��ǰת������
    u8    u8_cmddir;          //����ת������
    u8    u8_complete;        //ֹͣת��
    u32   u32_step;           //���в���
    u16   u16_vel;            //�����ٶ�
    u8    u8_accel_value;     //����ֵ
    u8    u8_decel_value;     //����ֵ
    u32   u32_pos;            //��ǰָ��λ��
    u32   u32_cmdpos;         //�µ�ָ��λ��
    u32   u32_poschange;      //λ�øı���
    u8    u8_direction;       //0-������ϣ�1-ͬ��仯��2-����仯
    u8    u8_status_id;       //״̬ 0-�ϵ��ʼ����1-��������
}motorparams;



extern motorparams *mtr0,*mtr1,*mtr2,*mtr3,*mtr4;
extern motorparams Motor0, Motor1, Motor2, Motor3,Motor4;

void Init_Motor_Params(void);
void	Config_Motor_io(void);
void GoToPosition(motorparams *mtr, u8 position);
u8 CheckIfStopped(motorparams *mtr);
void WaitUntilStopped(motorparams *mtr);
void	motor_zero(void);
void  small_motor_zero(void);
void	motor_set_zero(void);
void	motor_max(void);
void Motor_SelfCheck(void);
//void	get_motor_par(void);

#endif
