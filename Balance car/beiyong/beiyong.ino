#include <DATASCOPE.h>      //这是PC端上位机的库文件
DATASCOPE data;
unsigned char Send_Count;  //上位机相关变量

#include <KalmanFilter.h>    //卡尔曼滤波
#include <MsTimer2.h>        //定时中断
#include <PinChangeInt.h>    //外部中断
#include "I2Cdev.h"        
#include "MPU6050_6Axis_MotionApps20.h"//MPU6050库文件
#include "Wire.h"           
MPU6050 Mpu6050; //实例化一个 MPU6050 对象，对象名称为 Mpu6050
KalmanFilter KalFilter;//实例化一个卡尔曼滤波器对象，对象名称为 KalFilter
int16_t ax, ay, az, gx, gy, gz;  //MPU6050的三轴加速度和三轴陀螺仪数据
double angle, Gryo;  //用于显示的角度和临时变量

//左电机
int AIN2=13;
int AIN1=12;
int PWMA=10;
//左电机编码器读取
int LeftA=2;
int LeftB=5;
//左电机编码器转化
int La,Lb,La1,Lb1;
int leftcount=10000;//脉冲计数
int model;//记录标志位zuo

//右电机
int BIN2=7;
int BIN1=6;
int PWMB=9;
//右电机编码器读取
int RightA=4;
int RightB=8;
//右电机编码器转化
int Ra,Rb,Ra1,Rb1;
int Rightcount=10000;//脉冲计数
int modeR;//记录标志位zuo


/*************陀螺仪滤波参数***********/
int16_t A_X_sz[20] = {0},A_Z_sz[20] = {0};
double A_X_, A_Z_;
int32_t A_X_Count = 0, A_Z_Count = 0;
int8_t Ai = 0;
double bias_A;

//窗口滤波
float Angle_Cale(int16_t a_x, int16_t a_z, int16_t g_y)
{
  double Angle_A;
  double Angle;
  //////////均值滤波///
  A_X_Count = A_X_Count + a_x - A_X_sz[Ai];
  A_X_sz[Ai] = a_x; 

  A_Z_Count = A_Z_Count + a_z - A_Z_sz[Ai];
  A_Z_sz[Ai] = a_z; 
  //////////////
  
  Ai = Ai+1;
  if(Ai >= 20)
  {Ai = 0;}
    
  A_X_ = 1.0 * A_X_Count / 20;
  A_Z_ = 1.0 * A_Z_Count / 20;
  double j = atan(A_X_/A_Z_);
  Angle_A = -(j*180/3.14);// 算出角度


  g_y=g_y/135;
  bias_A=Angle_A*0.02 + (bias_A+0.005*g_y)*0.98; //一阶互补
  Angle= bias_A;
  return Angle;


}
/**************************************************************************/
//PID
float P_KP=200,P_KI=0.1,P_KD=0;      //PID系数
float Motor;
float Target_angle=-3.6;//目标位置


int Position_PID (int Encoder,int Target)
{   
   static float Pwm,Integral_bias,Last_Bias;
   float Bias;
   Bias=Encoder-Target;                                  //计算偏差
   Integral_bias+=Bias;                                  //求出偏差的积分
   Pwm=P_KP*Bias/28+P_KI*Integral_bias/2800+P_KD*(Bias-Last_Bias)/28;       //位置式PID控制器
  
   Last_Bias=Bias;                                       //保存上一次偏差 
   return Pwm;                                           //增量输出
}

int Balance_pwm(double T_ang,double angle,int g_y )
{ int B_pwm;
  double ang;
  ang=angle-T_ang;
  g_y=g_y/135;
  B_pwm= 40*ang+0.6*g_y;

    
  return B_pwm;
  }
/**************************************************************************


**************************************************************************/
void control()
{
  sei();//全局中断开启
  Mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //获取MPU6050陀螺仪和加速度计的数据
    //偏差
    ax-=775;
    az+=1600;
    gy+=123;
    angle = Angle_Cale(ax,az,gy);

   Motor=Balance_pwm(Target_angle,angle,gy);
Set_PwmL(Motor);//正死区 19  负 —18
Set_PwmR(Motor);//正死区 16   负 —18  

}




void DataScope(void)
{
  int i;
  data.DataScope_Get_Channel_Data(leftcount, 1);  
  data.DataScope_Get_Channel_Data(Rightcount, 2);
  data.DataScope_Get_Channel_Data(Motor, 3);
  data.DataScope_Get_Channel_Data(A_X_, 4);
  data.DataScope_Get_Channel_Data(bias_A, 5);
  data.DataScope_Get_Channel_Data(angle, 6);
  Send_Count = data.DataScope_Data_Generate(6); 
  for ( i = 0 ; i < Send_Count; i++)
  {
    Serial.write(DataScope_OutPut_Buffer[i]);  
  }
  delay(50);  
}


/**************************************************************************
                        step


**************************************************************************/
void setup() {
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(LeftA,INPUT);
  pinMode(LeftB,INPUT);

  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(RightA,INPUT);
  pinMode(RightB,INPUT);


  digitalWrite(AIN1, 0);        
  digitalWrite(AIN2, 0);          
  digitalWrite(PWMA, 0);  
  
  digitalWrite(BIN1, 0);        
  digitalWrite(BIN2, 0);          
  digitalWrite(PWMB, 0);   
  
  Wire.begin();             //加入 IIC 总线
  Serial.begin(115200);       //开启串口，设置波特率为 9600
  delay(1000);              //延时等待初始化完成
  Mpu6050.initialize();     //初始化MPU6050
  delay(20); 
  MsTimer2::set(5, control);  //使用Timer2设置5ms定时中断
  MsTimer2::start();          //使用中断使能

  attachInterrupt(0, READL, CHANGE);           //开启外部中断 编码器接口1
  attachPinChangeInterrupt(4, READR, CHANGE);  //开启外部中断 编码器接口2
}





void loop() {

  Serial.begin(115200), DataScope();          //延时等待初始化完成
         //延时等待初始化完成
}




//左计数///////////////////////////////////////////////////////////////////////
void READL() {
  if (digitalRead(LeftA) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(LeftB) == LOW)      leftcount--;  //根据另外一相电平判定方向
    else      leftcount++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(LeftB) == LOW)     leftcount++; //根据另外一相电平判定方向
    else     leftcount--;
  }

}

//右计数///////////////////////////////////////////////////////////////////////
void READR() {
  if (digitalRead(RightA) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(RightB) == LOW)      Rightcount--;  //根据另外一相电平判定方向
    else      Rightcount++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(RightB) == LOW)     Rightcount++; //根据另外一相电平判定方向
    else     Rightcount--;
  }

}
 /***********************************PWM电机控制***************************************/

void Set_PwmL(int motor){

  if (motor> 0)      digitalWrite(AIN1, HIGH),      digitalWrite(AIN2, LOW);  //TB6612的电平控制
  else               digitalWrite(AIN1, LOW),       digitalWrite(AIN2, HIGH); //TB6612的电平控制
if(motor>0)  motor+=16;
if(motor<0)  motor-=9;
  analogWrite(PWMA, abs(motor)); 
}

void Set_PwmR(int motor){

  if (motor> 0)      digitalWrite(BIN1, HIGH),      digitalWrite(BIN2, LOW);  //TB6612的电平控制
  else               digitalWrite(BIN1, LOW),       digitalWrite(BIN2, HIGH); //TB6612的电平控制
  if(motor>0)  motor+=13;
  if(motor<0)  motor-=9;
  analogWrite(PWMB, abs(motor)); 
}
