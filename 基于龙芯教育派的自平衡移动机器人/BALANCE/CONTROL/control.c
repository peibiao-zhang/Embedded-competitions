#include "control.h"	
#include "filter.h"	
u8 Flag_Target;                             //相关标志位
float Voltage_Count,Voltage_All;  //电压采样相关变量
int Balance_Pwm_X,Velocity_Pwm_X,Balance_Pwm_Y,Velocity_Pwm_Y;
#define X_PARAMETER           (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER           (1.0f)  
/**************************************************************************
函数功能：小车运动 逆运动学分析
入口参数：X Y Z 三轴速度或者位置
返回  值：无
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
	      Target_A   = Vx + L_PARAMETER*Vz;
        Target_B   = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
	      Target_C   = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;
}
/**************************************************************************
函数功能：小车运动 正运动学分析 注：实际注释掉的是理想的运动学分析，实际使用放大了3倍，对计算结果没影响，主要是避免舍去误差
入口参数：A B C三个电机的速度
返回  值：无
**************************************************************************/
void Forward_Kinematics(float Va,float Vb,float Vc)
{
//		Motor_X=(Va*2-Vb-Vc)/3;
//		Motor_Y=(Vb-Vc)/2/Y_PARAMETER;
//		Motor_Z=(Va+Vb+Vc)/3;
			Motor_X=Va*2-Vb-Vc;
		  Motor_Y=(Vb-Vc)*sqrt(3);
		  Motor_Z=Va+Vb+Vc;
}
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	 if(INT==0)		
	{     
		  EXTI->PR=1<<15;   //清除LINE5上的中断标志位  		
		  Flag_Target=!Flag_Target;
		  if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;  //给主函数提供50ms的精准延时
			 }
		  if(Flag_Target==1)                     //5ms读取一次陀螺仪和加速度计的值
			{	 
				Read_DMP();                          //===更新姿态		
				Key();													     //扫描按键变化	
				if(Pitch_Bias>-0.8&&Pitch_Bias<0.8&&Roll_Bias>-0.8&&Roll_Bias<0.8)Led_Flash(0);     //接近平衡位置，常亮
        else 		Led_Flash(100);  		     //===LED闪烁
				Voltage_All+=Get_battery_volt();     //多次采样累积
				if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//求平均值 获取电池电压	
				return 0;	                                               
			}     //===10ms控制一次	
			 Read_DMP();    //===更新姿态		
			 Roll_Bias =Roll-Roll_Zero;		//获取Y方向的偏差
		   Pitch_Bias=Pitch-Pitch_Zero; //获取X方向的偏差		
			 Forward_Kinematics(Motor_A,Motor_B,Motor_C);  //正运动学分析，得到X Y Z 方向的速度
		   Balance_Pwm_X=balance_Pitch(Pitch_Bias,gyro[1]);   //X方向的倾角控制
			 Balance_Pwm_Y=-balance_Roll(Roll_Bias, gyro[0]);   //Y方向的倾角控制
			 Velocity_Pwm_X=velocity_X(Motor_X);      //X方向的速度控制
			 Velocity_Pwm_Y=velocity_Y(Motor_Y);  	  //Y方向的速度控制
			 Move_X =Balance_Pwm_X+Velocity_Pwm_X;    //===X方向控制量累加				
			 Move_Y =Balance_Pwm_Y+Velocity_Pwm_Y;    //===Y方向控制量累加	
			 Move_Z=0;  //Z轴不做控制
				Kinematic_Analysis(Move_X,Move_Y,Move_Z);//逆运动学分析，得到A B C电机控制量
				Motor_A=Target_A;//直接调节PWM频率
				Motor_B=Target_B;//直接调节PWM频率
				Motor_C=Target_C;//直接调节PWM频率	 			 		 
				Xianfu_Pwm(1300);//===PWM频率限幅  因为频率太大之后扭矩减小不再满足小车需求
			 if(Turn_Off(Voltage)==0)	Set_Pwm(Motor_A,Motor_B,Motor_C);    //赋值给PWM寄存器
			 if(Flag_Zero)   //这是人为更新零点值
			 {
				 Roll_Zero=Roll; //ROLL更新
				 Pitch_Zero=Pitch; //Pitch更新
				 Flag_Zero=0;  //清零过程仅执行一次 等待下一次指令
			 }
 }
	 return 0;	 
} 

/**************************************************************************
函数功能：直立PD控制Y
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/
int balance_Roll(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle;  //===求出平衡的角度中值 和机械等重心分布相关
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd/100;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}
/**************************************************************************
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/
int balance_Pitch(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle;        //===求出平衡的角度中值和机械等重心分布相关
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd/100;   //===计算平衡控制的电机PWM  PD控制  kp是P系数 kd是D系数 
	 return balance;
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity
入口参数：左轮速度、右轮速度
返回  值：速度控制PWM
**************************************************************************/
int velocity_X(int velocity)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Target_Velocity=2500;
	  static float Encoder_Integral;  
		if(1==Flag_Left)    	Movement=-Target_Velocity;	           //===前进标志位置1 
		else if(1==Flag_Right)	Movement=Target_Velocity;           //===后退标志位置1
  	else  Movement=0;
    //=============速度PI控制器=======================//	
		Encoder_Least=Mean_Filter_X(velocity);        //速度滤波  
		Encoder *= 0.7;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.3;	                                    //===一阶低通滤波器    
 		Encoder_Integral +=Encoder;                                       //===积分出位移 
		Encoder_Integral +=Movement;                                      //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>150000)  	Encoder_Integral=150000;               //===积分限幅
		if(Encoder_Integral<-150000)	Encoder_Integral=-150000;              //===积分限幅	
	  if(Flag_Stop)   Encoder_Integral=0; //===电机关闭后清除积分
		Velocity=Encoder*Velocity_Kp/100+Encoder_Integral*Velocity_Ki/5000;        //===速度控制	
	  if(Flag_Stop)   Velocity=0;      //===电机关闭后清除积分
		Show_Data1=Encoder_Integral;
	  Show_Data3=Velocity;
		if(Velocity>1000)  	Velocity=1000;               //===速度环限幅
		if(Velocity<-1000)	  Velocity=-1000;              //===速度环限幅
	  return Velocity;
}
/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity
入口参数：左轮速度、右轮速度
返回  值：速度控制PWM
**************************************************************************/
int velocity_Y(int velocity)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Target_Velocity=2500;
	  static float Encoder_Integral;  
	  if(1==Flag_Qian)    	  Movement=Target_Velocity;	           //===前进标志位置1 
		else if(1==Flag_Hou)	  Movement=-Target_Velocity;           //===后退标志位置1
  	else  Movement=0;
//   //=============速度PI控制器=======================//	
		Encoder_Least=Mean_Filter_Y(velocity);          //速度滤波      
		Encoder *= 0.7;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.3;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 
		Encoder_Integral +=Movement;                                  //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>150000)  	Encoder_Integral=150000;            //===积分限幅
		if(Encoder_Integral<-150000)	Encoder_Integral=-150000;              //===积分限幅	
    if(Flag_Stop)   Encoder_Integral=0;      //===电机关闭后清除积分
  	Velocity=Encoder*Velocity_Kp/100+Encoder_Integral*Velocity_Ki/5000;      //===速度控制	
	  if(Flag_Stop)   Velocity=0;      //===电机关闭后清除积分
   	Show_Data2=Encoder_Integral;
	  Show_Data4=Velocity;
	  if(Velocity>1000)  	Velocity=1000;               //===速度环限幅
		if(Velocity<-1000)	  Velocity=-1000;              //===速度环限幅
	  return Velocity;
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c)
{
			int Final_Motor_A,Final_Motor_B,Final_Motor_C;
	   	if(motor_a>0)			    INA=0;   //电机A方向控制
			else 	             	  INA=1;
	   	if(motor_b>0)			    INB=0;   //电机B方向控制
			else 	             	  INB=1;
			if(motor_c>0)			    INC=0;   //电机C方向控制
			else 	                INC=1;

			Final_Motor_A=Linear_Conversion(motor_a);  //线性化
    	Final_Motor_B=Linear_Conversion(motor_b);
			Final_Motor_C=Linear_Conversion(motor_c);
			Set_PWM_Final(Final_Motor_A,Final_Motor_B,Final_Motor_C);  
}
/**************************************************************************
函数功能：对控制输出的PWM线性化,便于给系统寄存器赋值
入口参数：PWM
返回  值：线性化后的PWM
**************************************************************************/
u16  Linear_Conversion(int motor)
{ 
	 u32 temp;
   u16 Linear_Moto;
   temp=1000000/myabs(motor);   //1000000是经验值
	 if(temp>65535) Linear_Moto=65535;
	 else Linear_Moto=(u16)temp;
	 return Linear_Moto;
}	

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{	
    if(Motor_A<-amplitude) Motor_A=-amplitude;	
		if(Motor_A>amplitude)  Motor_A=amplitude;	
	  if(Motor_B<-amplitude) Motor_B=-amplitude;	
		if(Motor_B>amplitude)  Motor_B=amplitude;		
	  if(Motor_C<-amplitude) Motor_C=-amplitude;	
		if(Motor_C>amplitude)  Motor_C=amplitude;			
}

/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click_N_Double(75);    //读取按键信息
  tmp2=Long_Press();
	if(tmp==1)Flag_Stop=!Flag_Stop;//单击开机/关闭电机   
	if(tmp==2)Flag_Zero=!Flag_Zero;//双击读取零点 

	if(tmp2==1)Flag_Show=!Flag_Show;//长按切换显示模式 	
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(Flag_Stop==1||Pitch<-60||Pitch>60||Roll<-60||Roll>60)// 角度过大或者Flag_Stop置1则关闭并失能电机
			{	                                                
			temp=1;      
      INA=0;
      INB=0;
      INC=0;				
			ST=0;   //失能电机
			Flag_Stop=1;
      }
			else
			ST=1,	
      temp=0;
      return temp;			
}

/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
函数功能：速度滤波
入口参数：速度
返回  值：滤波后的速度
**************************************************************************/
int Mean_Filter_X(int motor)
{
  u8 i;
  s32 Sum_Speed = 0; 
	s16 Filter_Speed;
  static  s16 Speed_Buf[FILTERING_TIMES]={0};
  for(i = 1 ; i<FILTERING_TIMES; i++)//平滑数据
  {
    Speed_Buf[i - 1] = Speed_Buf[i];
  }
  Speed_Buf[FILTERING_TIMES - 1] =motor;

  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Speed += Speed_Buf[i];
  }
  Filter_Speed = (s16)(Sum_Speed / FILTERING_TIMES);//取平均
	return Filter_Speed;
}
/**************************************************************************
函数功能：速度滤波
入口参数：速度
返回  值：滤波后的速度
**************************************************************************/
int Mean_Filter_Y(int motor)
{
  u8 i;
  s32 Sum_Speed = 0; 
	s16 Filter_Speed;
  static  s16 Speed_Buf[FILTERING_TIMES]={0};
  for(i = 1 ; i<FILTERING_TIMES; i++)
  {
    Speed_Buf[i - 1] = Speed_Buf[i];
  }
  Speed_Buf[FILTERING_TIMES - 1] =motor;

  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Speed += Speed_Buf[i];
  }
  Filter_Speed = (s16)(Sum_Speed / FILTERING_TIMES);//取平均
	return Filter_Speed;
}
