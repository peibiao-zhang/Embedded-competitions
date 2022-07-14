#include "control.h"	
#include "filter.h"	
u8 Flag_Target;                             //��ر�־λ
float Voltage_Count,Voltage_All;  //��ѹ������ر���
int Balance_Pwm_X,Velocity_Pwm_X,Balance_Pwm_Y,Velocity_Pwm_Y;
#define X_PARAMETER           (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER           (1.0f)  
/**************************************************************************
�������ܣ�С���˶� ���˶�ѧ����
��ڲ�����X Y Z �����ٶȻ���λ��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{
	      Target_A   = Vx + L_PARAMETER*Vz;
        Target_B   = -X_PARAMETER*Vx + Y_PARAMETER*Vy + L_PARAMETER*Vz;
	      Target_C   = -X_PARAMETER*Vx - Y_PARAMETER*Vy + L_PARAMETER*Vz;
}
/**************************************************************************
�������ܣ�С���˶� ���˶�ѧ���� ע��ʵ��ע�͵�����������˶�ѧ������ʵ��ʹ�÷Ŵ���3�����Լ�����ûӰ�죬��Ҫ�Ǳ�����ȥ���
��ڲ�����A B C����������ٶ�
����  ֵ����
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
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	 if(INT==0)		
	{     
		  EXTI->PR=1<<15;   //���LINE5�ϵ��жϱ�־λ  		
		  Flag_Target=!Flag_Target;
		  if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;  //���������ṩ50ms�ľ�׼��ʱ
			 }
		  if(Flag_Target==1)                     //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ
			{	 
				Read_DMP();                          //===������̬		
				Key();													     //ɨ�谴���仯	
				if(Pitch_Bias>-0.8&&Pitch_Bias<0.8&&Roll_Bias>-0.8&&Roll_Bias<0.8)Led_Flash(0);     //�ӽ�ƽ��λ�ã�����
        else 		Led_Flash(100);  		     //===LED��˸
				Voltage_All+=Get_battery_volt();     //��β����ۻ�
				if(++Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ ��ȡ��ص�ѹ	
				return 0;	                                               
			}     //===10ms����һ��	
			 Read_DMP();    //===������̬		
			 Roll_Bias =Roll-Roll_Zero;		//��ȡY�����ƫ��
		   Pitch_Bias=Pitch-Pitch_Zero; //��ȡX�����ƫ��		
			 Forward_Kinematics(Motor_A,Motor_B,Motor_C);  //���˶�ѧ�������õ�X Y Z ������ٶ�
		   Balance_Pwm_X=balance_Pitch(Pitch_Bias,gyro[1]);   //X�������ǿ���
			 Balance_Pwm_Y=-balance_Roll(Roll_Bias, gyro[0]);   //Y�������ǿ���
			 Velocity_Pwm_X=velocity_X(Motor_X);      //X������ٶȿ���
			 Velocity_Pwm_Y=velocity_Y(Motor_Y);  	  //Y������ٶȿ���
			 Move_X =Balance_Pwm_X+Velocity_Pwm_X;    //===X����������ۼ�				
			 Move_Y =Balance_Pwm_Y+Velocity_Pwm_Y;    //===Y����������ۼ�	
			 Move_Z=0;  //Z�᲻������
				Kinematic_Analysis(Move_X,Move_Y,Move_Z);//���˶�ѧ�������õ�A B C���������
				Motor_A=Target_A;//ֱ�ӵ���PWMƵ��
				Motor_B=Target_B;//ֱ�ӵ���PWMƵ��
				Motor_C=Target_C;//ֱ�ӵ���PWMƵ��	 			 		 
				Xianfu_Pwm(1300);//===PWMƵ���޷�  ��ΪƵ��̫��֮��Ť�ؼ�С��������С������
			 if(Turn_Off(Voltage)==0)	Set_Pwm(Motor_A,Motor_B,Motor_C);    //��ֵ��PWM�Ĵ���
			 if(Flag_Zero)   //������Ϊ�������ֵ
			 {
				 Roll_Zero=Roll; //ROLL����
				 Pitch_Zero=Pitch; //Pitch����
				 Flag_Zero=0;  //������̽�ִ��һ�� �ȴ���һ��ָ��
			 }
 }
	 return 0;	 
} 

/**************************************************************************
�������ܣ�ֱ��PD����Y
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
**************************************************************************/
int balance_Roll(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle;  //===���ƽ��ĽǶ���ֵ �ͻ�е�����ķֲ����
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd/100;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}
/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
**************************************************************************/
int balance_Pitch(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle;        //===���ƽ��ĽǶ���ֵ�ͻ�е�����ķֲ����
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd/100;   //===����ƽ����Ƶĵ��PWM  PD����  kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}

/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶȣ�����Target_Velocity
��ڲ����������ٶȡ������ٶ�
����  ֵ���ٶȿ���PWM
**************************************************************************/
int velocity_X(int velocity)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Target_Velocity=2500;
	  static float Encoder_Integral;  
		if(1==Flag_Left)    	Movement=-Target_Velocity;	           //===ǰ����־λ��1 
		else if(1==Flag_Right)	Movement=Target_Velocity;           //===���˱�־λ��1
  	else  Movement=0;
    //=============�ٶ�PI������=======================//	
		Encoder_Least=Mean_Filter_X(velocity);        //�ٶ��˲�  
		Encoder *= 0.7;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.3;	                                    //===һ�׵�ͨ�˲���    
 		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� 
		Encoder_Integral +=Movement;                                      //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>150000)  	Encoder_Integral=150000;               //===�����޷�
		if(Encoder_Integral<-150000)	Encoder_Integral=-150000;              //===�����޷�	
	  if(Flag_Stop)   Encoder_Integral=0; //===����رպ��������
		Velocity=Encoder*Velocity_Kp/100+Encoder_Integral*Velocity_Ki/5000;        //===�ٶȿ���	
	  if(Flag_Stop)   Velocity=0;      //===����رպ��������
		Show_Data1=Encoder_Integral;
	  Show_Data3=Velocity;
		if(Velocity>1000)  	Velocity=1000;               //===�ٶȻ��޷�
		if(Velocity<-1000)	  Velocity=-1000;              //===�ٶȻ��޷�
	  return Velocity;
}
/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶȣ�����Target_Velocity
��ڲ����������ٶȡ������ٶ�
����  ֵ���ٶȿ���PWM
**************************************************************************/
int velocity_Y(int velocity)
{  
    static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Target_Velocity=2500;
	  static float Encoder_Integral;  
	  if(1==Flag_Qian)    	  Movement=Target_Velocity;	           //===ǰ����־λ��1 
		else if(1==Flag_Hou)	  Movement=-Target_Velocity;           //===���˱�־λ��1
  	else  Movement=0;
//   //=============�ٶ�PI������=======================//	
		Encoder_Least=Mean_Filter_Y(velocity);          //�ٶ��˲�      
		Encoder *= 0.7;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.3;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� 
		Encoder_Integral +=Movement;                                  //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>150000)  	Encoder_Integral=150000;            //===�����޷�
		if(Encoder_Integral<-150000)	Encoder_Integral=-150000;              //===�����޷�	
    if(Flag_Stop)   Encoder_Integral=0;      //===����رպ��������
  	Velocity=Encoder*Velocity_Kp/100+Encoder_Integral*Velocity_Ki/5000;      //===�ٶȿ���	
	  if(Flag_Stop)   Velocity=0;      //===����رպ��������
   	Show_Data2=Encoder_Integral;
	  Show_Data4=Velocity;
	  if(Velocity>1000)  	Velocity=1000;               //===�ٶȻ��޷�
		if(Velocity<-1000)	  Velocity=-1000;              //===�ٶȻ��޷�
	  return Velocity;
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c)
{
			int Final_Motor_A,Final_Motor_B,Final_Motor_C;
	   	if(motor_a>0)			    INA=0;   //���A�������
			else 	             	  INA=1;
	   	if(motor_b>0)			    INB=0;   //���B�������
			else 	             	  INB=1;
			if(motor_c>0)			    INC=0;   //���C�������
			else 	                INC=1;

			Final_Motor_A=Linear_Conversion(motor_a);  //���Ի�
    	Final_Motor_B=Linear_Conversion(motor_b);
			Final_Motor_C=Linear_Conversion(motor_c);
			Set_PWM_Final(Final_Motor_A,Final_Motor_B,Final_Motor_C);  
}
/**************************************************************************
�������ܣ��Կ��������PWM���Ի�,���ڸ�ϵͳ�Ĵ�����ֵ
��ڲ�����PWM
����  ֵ�����Ի����PWM
**************************************************************************/
u16  Linear_Conversion(int motor)
{ 
	 u32 temp;
   u16 Linear_Moto;
   temp=1000000/myabs(motor);   //1000000�Ǿ���ֵ
	 if(temp>65535) Linear_Moto=65535;
	 else Linear_Moto=(u16)temp;
	 return Linear_Moto;
}	

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
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
�������ܣ������޸�С������״̬ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click_N_Double(75);    //��ȡ������Ϣ
  tmp2=Long_Press();
	if(tmp==1)Flag_Stop=!Flag_Stop;//��������/�رյ��   
	if(tmp==2)Flag_Zero=!Flag_Zero;//˫����ȡ��� 

	if(tmp2==1)Flag_Show=!Flag_Show;//�����л���ʾģʽ 	
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(Flag_Stop==1||Pitch<-60||Pitch>60||Roll<-60||Roll>60)// �Ƕȹ������Flag_Stop��1��رղ�ʧ�ܵ��
			{	                                                
			temp=1;      
      INA=0;
      INB=0;
      INC=0;				
			ST=0;   //ʧ�ܵ��
			Flag_Stop=1;
      }
			else
			ST=1,	
      temp=0;
      return temp;			
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
�������ܣ��ٶ��˲�
��ڲ������ٶ�
����  ֵ���˲�����ٶ�
**************************************************************************/
int Mean_Filter_X(int motor)
{
  u8 i;
  s32 Sum_Speed = 0; 
	s16 Filter_Speed;
  static  s16 Speed_Buf[FILTERING_TIMES]={0};
  for(i = 1 ; i<FILTERING_TIMES; i++)//ƽ������
  {
    Speed_Buf[i - 1] = Speed_Buf[i];
  }
  Speed_Buf[FILTERING_TIMES - 1] =motor;

  for(i = 0 ; i < FILTERING_TIMES; i++)
  {
    Sum_Speed += Speed_Buf[i];
  }
  Filter_Speed = (s16)(Sum_Speed / FILTERING_TIMES);//ȡƽ��
	return Filter_Speed;
}
/**************************************************************************
�������ܣ��ٶ��˲�
��ڲ������ٶ�
����  ֵ���˲�����ٶ�
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
  Filter_Speed = (s16)(Sum_Speed / FILTERING_TIMES);//ȡƽ��
	return Filter_Speed;
}
