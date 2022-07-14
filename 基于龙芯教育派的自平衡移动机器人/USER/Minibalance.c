#include "sys.h"
u8 Flag_Left,Flag_Right;   // 
u8 Flag_Stop=1,Flag_Zero=0,Flag_Show,Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_OK;       //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
float Motor_X,Motor_Y,Motor_Z;
long int Motor_A,Motor_B,Motor_C;        //���PWM����
long int Target_A,Target_B,Target_C;     //���Ŀ��ֵ
int Voltage;              //��ص�ѹ������صı���
float Show_Data1,Show_Data2,Show_Data3,Show_Data4;       //ȫ����ʾ������������ʾ��Ҫ�鿴������                         
u8 delay_50,delay_flag;   //��ʱ��ر���
u8 PID_Send;  //CAN�ʹ��ڿ�����ر���
float Pitch,Roll,Yaw,Move_X,Move_Y,Move_Z,Roll_Bias,Pitch_Bias,Roll_Zero,Pitch_Zero; 
float	Balance_Kp=200,Balance_Kd=19,Velocity_Kp=55,Velocity_Ki=10;  //λ�ÿ���PID����
int main(void)
{ 
	Stm32_Clock_Init(9);            //=====ϵͳʱ������
	delay_init(72);                 //=====��ʱ��ʼ��
	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
	LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
	KEY_Init();                     //=====������ʼ��
	OLED_Init();                    //=====OLED��ʼ��
	uart_init(72,128000);           //=====����1��ʼ��
	uart2_init(36,9600);            //=====����2��ʼ��
  uart3_init(36,115200);          //=====����3��ʼ�� 
	Adc_Init();                     //=====adc��ʼ��
	IIC_Init();                     //=====IIC��ʼ��
	delay_ms(50);                    
  MPU6050_initialize();           //=====MPU6050��ʼ��	
  DMP_Init();                     //=====��ʼ��DMP     
	delay_ms(500);                  //=====��ʱ�ȴ���ʼ���ȶ�
  EXTI_Init();                    //=====MPU6050 5ms��ʱ�жϳ�ʼ��  CAN1_Mode_Init(1,2,3,6,0);      //=====CAN��ʼ��
	MiniBalance_PWM_Init(7199,14);  //=====��ʼ��PWM �����������
	while(1)
		{		
			if(Flag_Show==0)
			{
		  	DataScope();	            //===��λ��
				delay_flag=1;	            //===50ms�жϾ�׼��ʱ��־λ
				oled_show();              //===��ʾ����	  	
				while(delay_flag);        //===50ms�жϾ�׼��ʱ  ��Ҫ�ǲ�����ʾ��λ����Ҫ�ϸ��50ms�������� 
			}
			else
			{
				APP_Show();               //===APP
				oled_show();              //===��ʾ����	  	
			  delay_flag=1;	            //===50ms�жϾ�׼��ʱ��־λ
				while(delay_flag);        //===50ms�жϾ�׼��ʱ  ��Ҫ�ǲ�����ʾ��λ����Ҫ�ϸ��50ms��������   	
			}
		} 
}
