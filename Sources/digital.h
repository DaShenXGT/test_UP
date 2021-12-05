#ifndef	_DIGITAL_H
#define	_DIGITAL_H

#define	ENGINE_SPEED_SAMPLE_NUM			10

//74HC165驱动引脚定义
#define	DITIGAL_LOAD	 	PTH_PTH5
#define	DITIGAL_CLK			PTH_PTH6
#define	DITIGAL_DATA		PTH_PTH4 

extern	vehicle_digital	digital_status;
extern	Uint8_t		Engine_SpeedSampleCnt;
extern	Uint16_t	Engine_SpeedSampleBuffer[ENGINE_SPEED_SAMPLE_NUM];

void Init_Digital_Params(void);
void	Config_Ditigal_Io(void);
Uint8_t	Digital_Drive(void);

#endif