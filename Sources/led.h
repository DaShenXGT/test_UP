#ifndef	_LED_H
#define	_LED_H

//LED����оƬ�������Ŷ���
#define	LED_DATA	 	PTR_PTR0
#define	LED_CLK			PTR_PTR1
#define	LED_LAT			PTR_PTR2
#define	LED_BLANK		PTR_PTR3
#define	LED_FB			PTP_PTP7

//LED״̬�ָ�Bit����
#define LED_BATTERY                 (unsigned long)0x00000001              //D21 ���
#define LED_LEFT                    (unsigned long)0x00000002              //D22 ��ת������ΪӲ��ֱ�ӿ���
#define LED_OIL_TEMP                ((unsigned long)0x00000002)            //D50 ����
#define LED_WARN                    (unsigned long)0x00000104              //D23&D29  ���뱨��   
#define LED_RIGHT                   (unsigned long)0x00000008              //D24 ��ת������ΪӲ��ֱ�ӿ���
#define LED_BREAK_ERR               (unsigned long)0x00000010              //D25 �ƶ�ѹ��
#define LED_POST_PROCESSING_BIT     (unsigned long)0x00000020              //D26 �������
#define LED_PARKING_BREAK           (unsigned long)0x00000040              //D27 פ���ƶ�
#define LED_HIGH_BEAM               (unsigned long)0x00000080              //D28 Զ��
#define LED_FUEL_WATER              (unsigned long)0x00000400              //D31 ��ˮ���� 
#define LED_RECOVERY_BIT            (unsigned long)0x00002000              //D34 ����
#define LED_RECOVERY_INHIBITION_BIT (unsigned long)0x00010000              //D37 ��������
#define LED_TRANS_OIL_P             (unsigned long)0x00020000              //D38 ������ѹ����
#define LED_WATER_TEMP              (unsigned long)0x00040000              //D39 ��ȴҺ�¶ȸ�
#define LED_MUTE                    (unsigned long)0x00080000              //D40 ����
#define LED_ENGINE_FAULT_BIT        (unsigned long)0x00200000              //D42 ����������
#define LED_FUEL_LOW                (unsigned long)0x00400000              //D43 ȼ��Һλ��
#define LED_RECOVERY_TEMP_BIT       (unsigned long)0x01000000              //D45 �����¶�
#define LED_PREHEAT                 (unsigned long)0x02000000              //D46 Ԥ��
#define LED_UREA_LEVEL              (unsigned long)0x08000000              //D48 ����Һλ
#define LED_LOW_OIL_P               (unsigned long)0x10000000              //D49 ����ѹ����  

//LED��˸����
#define LED_BLINK_MASK (LED_BREAK_ERR | LED_TRANS_OIL_P | LED_FUEL_LOW | LED_WATER_TEMP | LED_WARN)

//LED״̬
typedef enum
{
  LED_ON,
  LED_OFF,
  LED_TOGGLE,
  
} LED_Action_t;

void LED_IOInit(void);
void Led_Drive(Uint32_t val);
void LED_LeftTurnLightOpen(void);
void LED_LeftTurnLightClose(void);
void LED_RightTurnLightOpen(void);
void LED_RightTurnLightClose(void);
void LED_BacklightOnOff(LED_Action_t action);
void LED_UreaLevelBacklightOn(void);
void LED_UreaLevelBacklightOff(void);
void LED_WaterTempBacklightOn(void);
void LED_WaterTempBacklightOff(void);
void LED_OilTempBacklightOn(void);
void LED_OilTempBacklightOff(void);
void LED_OilLevelBacklightOn(void);
void LED_OilLevelBacklightOff(void);

#endif