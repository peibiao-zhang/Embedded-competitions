#include "sys.h"
u8 Flag_Left,Flag_Right;   // 
u8 Flag_Stop=1,Flag_Zero=0,Flag_Show,Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_OK;       //停止标志位和 显示标志位 默认停止 显示打开
float Motor_X,Motor_Y,Motor_Z;
long int Motor_A,Motor_B,Motor_C;        //电机PWM变量
long int Target_A,Target_B,Target_C;     //电机目标值
int Voltage;              //电池电压采样相关的变量
float Show_Data1,Show_Data2,Show_Data3,Show_Data4;       //全局显示变量，用于显示需要查看的数据                         
u8 delay_50,delay_flag;   //延时相关变量
u8 PID_Send;  //CAN和串口控制相关变量
float Pitch,Roll,Yaw,Move_X,Move_Y,Move_Z,Roll_Bias,Pitch_Bias,Roll_Zero,Pitch_Zero; 
float	Balance_Kp=200,Balance_Kd=19,Velocity_Kp=55,Velocity_Ki=10;  //位置控制PID参数
int main(void)
{ 
	Stm32_Clock_Init(9);            //=====系统时钟设置
	delay_init(72);                 //=====延时初始化
	JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
	LED_Init();                     //=====初始化与 LED 连接的硬件接口
	KEY_Init();                     //=====按键初始化
	OLED_Init();                    //=====OLED初始化
	uart_init(72,128000);           //=====串口1初始化
	uart2_init(36,9600);            //=====串口2初始化
  uart3_init(36,115200);          //=====串口3初始化 
	Adc_Init();                     //=====adc初始化
	IIC_Init();                     //=====IIC初始化
	delay_ms(50);                    
  MPU6050_initialize();           //=====MPU6050初始化	
  DMP_Init();                     //=====初始化DMP     
	delay_ms(500);                  //=====延时等待初始化稳定
  EXTI_Init();                    //=====MPU6050 5ms定时中断初始化  CAN1_Mode_Init(1,2,3,6,0);      //=====CAN初始化
	MiniBalance_PWM_Init(7199,14);  //=====初始化PWM 用于驱动电机
	while(1)
		{		
			if(Flag_Show==0)
			{
		  	DataScope();	            //===上位机
				delay_flag=1;	            //===50ms中断精准延时标志位
				oled_show();              //===显示屏打开	  	
				while(delay_flag);        //===50ms中断精准延时  主要是波形显示上位机需要严格的50ms传输周期 
			}
			else
			{
				APP_Show();               //===APP
				oled_show();              //===显示屏打开	  	
			  delay_flag=1;	            //===50ms中断精准延时标志位
				while(delay_flag);        //===50ms中断精准延时  主要是波形显示上位机需要严格的50ms传输周期   	
			}
		} 
}
