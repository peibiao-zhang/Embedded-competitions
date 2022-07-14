#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#define PI 3.14159265
#define ZHONGZHI 0 
#define FILTERING_TIMES  25
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
int EXTI15_10_IRQHandler(void);
void Kinematic_Analysis(float Vx,float Vy,float Vz);
void Forward_Kinematics(float Va,float Vb,float Vc);
int velocity_X(int velocity);
int velocity_Y(int velocity);
void Key(void);
void Xianfu_Pwm(int amplitude);
void Xianfu_Velocity(int amplitude_A,int amplitude_B,int amplitude_C);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
void Get_RC(u8 mode);
void Count_Velocity(void);
void CAN_N_Usart_Control(void);
u16  Linear_Conversion(int motor);
void Set_Pwm(int motor_a,int motor_b,int motor_c);
int balance_Roll(float Angle,float Gyro);
int balance_Pitch(float Angle,float Gyro);
int Mean_Filter_Y(int motor);
int Mean_Filter_X(int motor);
#endif
