/****************************************************************************
   作者：平衡小车之家
   产品名称：Minibalance For Arduino
****************************************************************************/
#include <DATASCOPE.h>      //这是PC端上位机的库文件
#include <PinChangeInt.h>    //外部中断
#include <MsTimer2.h>        //定时中断
#include <KalmanFilter.h>    //卡尔曼滤波
#include "I2Cdev.h"        
#include "MPU6050_6Axis_MotionApps20.h"//MPU6050库文件
#include "Wire.h"  
#include <EEPROM.h>         
MPU6050 Mpu6050; //实例化一个 MPU6050 对象，对象名称为 Mpu6050
DATASCOPE data;//实例化一个 上位机 对象，对象名称为 data
KalmanFilter KalFilter;//实例化一个卡尔曼滤波器对象，对象名称为 KalFilter
int16_t ax, ay, az, gx, gy, gz;  //MPU6050的三轴加速度和三轴陀螺仪数据
#define KEY 3     //按键引脚
#define IN1 12   //TB6612FNG驱动模块控制信号 共6个
#define IN2 13
#define IN3 7
#define IN4 6
#define PWMA 10
#define PWMB 9
#define ENCODER_L 2  //编码器采集引脚 每路2个 共4个
#define DIRECTION_L 5
#define ENCODER_R 4
#define DIRECTION_R 8
#define ZHONGZHI 0//小车的机械中值  DIFFERENCE
#define DIFFERENCE 2
int Balance_Pwm, Velocity_Pwm, Turn_Pwm;   //直立 速度 转向环的PWM
int Motor1, Motor2;      //电机叠加之后的PWM
float Battery_Voltage;   //电池电压 单位是V
volatile long Velocity_L, Velocity_R = 0;   //左右轮编码器数据
int Velocity_Left, Velocity_Right = 0;     //左右轮速度
int Flag_Qian, Flag_Hou, Flag_Left, Flag_Right; //遥控相关变量
int Angle, Show_Data,PID_Send;  //用于显示的角度和临时变量
unsigned char Flag_Stop = 1,Send_Count,Flash_Send;  //停止标志位和上位机相关变量
float Balance_Kp=15,Balance_Kd=0.4,Velocity_Kp=2,Velocity_Ki=0.01;
//***************下面是卡尔曼滤波相关变量***************//
float K1 = 0.05; // 对加速度计取值的权重
float Q_angle = 0.001, Q_gyro = 0.005;
float R_angle = 0.5 , C_0 = 1;
float dt = 0.005; //注意：dt的取值为滤波器采样时间 5ms
int addr = 0;
/**************************************************************************
函数功能：检测小车是否被拿起
入口参数：Z轴加速度 平衡倾角 左轮编码器 右轮编码器
返回  值：0：无事件 1：小车被拿起
**************************************************************************/
int Pick_Up(float Acceleration, float Angle, int encoder_left, int encoder_right){
  static unsigned int flag, count0, count1, count2;
  if (flag == 0) //第一步
  {
    if (abs(encoder_left) + abs(encoder_right) < 15)         count0++;  //条件1，小车接近静止
    else       count0 = 0;
    if (count0 > 10)      flag = 1, count0 = 0;
  }
  if (flag == 1) //进入第二步
  {
    if (++count1 > 400)       count1 = 0, flag = 0;                         //超时不再等待2000ms
    if (Acceleration > 27000 && (Angle > (-14 + ZHONGZHI)) && (Angle < (14 + ZHONGZHI)))  flag = 2; //条件2，小车是在0度附近被拿起
  }
  if (flag == 2)  //第三步
  {
    if (++count2 > 200)       count2 = 0, flag = 0;       //超时不再等待1000ms
    if (abs(encoder_left + encoder_right) > 300)           //条件3，小车的轮胎因为正反馈达到最大的转速      
     {
        flag = 0;  return 1;
      }                                           
  }
  return 0;
}
/**************************************************************************
函数功能：检测小车是否被放下 作者：平衡小车之家
入口参数： 平衡倾角 左轮编码器 右轮编码器
返回  值：0：无事件 1：小车放置并启动
**************************************************************************/
int Put_Down(float Angle, int encoder_left, int encoder_right){
  static u16 flag, count;
  if (Flag_Stop == 0)         return 0;                   //防止误检
  if (flag == 0)
  {
    if (Angle > (-10 + ZHONGZHI) && Angle < (10 + ZHONGZHI) && encoder_left == 0 && encoder_right == 0)      flag = 1; //条件1，小车是在0度附近的
  }
  if (flag == 1)
  {
    if (++count > 100)       count = 0, flag = 0;  //超时不再等待 500ms
    if (encoder_left > 12 && encoder_right > 12 && encoder_left < 80 && encoder_right < 80) //条件2，小车的轮胎在未上电的时候被人为转动
    {
      flag = 0;
      flag = 0;
      return 1;    //检测到小车被放下
    }
  }
  return 0;
}
/**************************************************************************
函数功能：异常关闭电机 作者：平衡小车之家
入口参数：倾角和电池电压
返回  值：1：异常  0：正常
**************************************************************************/
unsigned char Turn_Off(float angle, float voltage)
{
  unsigned char temp;
  if (angle < -40 || angle > 40 || 1 == Flag_Stop || voltage < 11.1) //电池电压低于11.1V关闭电机 //===倾角大于40度关闭电机//===Flag_Stop置1关闭电机
  {                                                                         
    temp = 1;                                          
    analogWrite(PWMA, 0);  //PWM输出为0
    analogWrite(PWMB, 0); //PWM输出为0
  }
  else    temp = 0;   //不存在异常，返回0
  return temp;
}
/**************************************************************************
函数功能：虚拟示波器往上位机发送数据 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void DataScope(void)
{
  int i;
  data.DataScope_Get_Channel_Data(Angle, 1);  //显示第一个数据，角度
  data.DataScope_Get_Channel_Data(Velocity_Left, 2);//显示第二个数据，左轮速度  也就是每40ms输出的脉冲计数
  data.DataScope_Get_Channel_Data(Velocity_Right, 3);//显示第三个数据，右轮速度 也就是每40ms输出的脉冲计数
  data.DataScope_Get_Channel_Data(Battery_Voltage, 4);//显示第四个数据，电池电压，单位V
  Send_Count = data.DataScope_Data_Generate(4); 
  for ( i = 0 ; i < Send_Count; i++)
  {
    Serial.write(DataScope_OutPut_Buffer[i]);  
  }
  delay(50);  //上位机必须严格控制发送时序
}
/**************************************************************************
函数功能：按键扫描  作者：平衡小车之家
入口参数：无
返回  值：按键状态，1：单击事件，0：无事件。
**************************************************************************/
unsigned char My_click(void){
  static unsigned char flag_key = 1; //按键按松开标志
  unsigned char Key;   
  Key = digitalRead(KEY);   //读取按键状态
  if (flag_key && Key == 0) //如果发生单击事件
  {
    flag_key = 0;
    return 1;            // 单击事件
  }
  else if (1 == Key)     flag_key = 1;
  return 0;//无按键按下
}
/**************************************************************************
函数功能：直立PD控制  作者：平衡小车之家
入口参数：角度、角速度
返回  值：直立控制PWM
**************************************************************************/
int balance(float Angle, float Gyro)
{
 int y=1;
}
/**************************************************************************
函数功能：速度PI控制 作者：平衡小车之家
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
**************************************************************************/
int velocity(int encoder_left, int encoder_right)
{
 int y=1;
}
/**************************************************************************
函数功能：转向控制 作者：平衡小车之家
入口参数：Z轴陀螺仪
返回  值：转向控制PWM
**************************************************************************/
int turn(float gyro)//转向控制
{
int y=1;
}
/**************************************************************************
函数功能：赋值给PWM寄存器 作者：平衡小车之家
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1, int moto2)
{
  if (moto1 > 0)     digitalWrite(IN1, HIGH),      digitalWrite(IN2, LOW);  //TB6612的电平控制
  else             digitalWrite(IN1, LOW),       digitalWrite(IN2, HIGH); //TB6612的电平控制
  analogWrite(PWMA, abs(moto1)); //赋值给PWM寄存器
  if (moto2 < 0) digitalWrite(IN3, HIGH),     digitalWrite(IN4, LOW); //TB6612的电平控制
  else        digitalWrite(IN3, LOW),      digitalWrite(IN4, HIGH); //TB6612的电平控制
  analogWrite(PWMB, abs(moto2));//赋值给PWM寄存器
}
/**************************************************************************
函数功能：限制PWM赋值  作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{
 int y=1;
}
/**************************************************************************
函数功能：5ms控制函数 核心代码 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void control()
{
  int y=1;
}
/**************************************************************************
函数功能：初始化 相当于STM32里面的Main函数 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void setup() {
  pinMode(IN1, OUTPUT);        //TB6612控制引脚，控制电机1的方向，01为正转，10为反转
  pinMode(IN2, OUTPUT);          //TB6612控制引脚，
  pinMode(IN3, OUTPUT);          //TB6612控制引脚，控制电机2的方向，01为正转，10为反转
  pinMode(IN4, OUTPUT);          //TB6612控制引脚，
  pinMode(PWMA, OUTPUT);         //TB6612控制引脚，电机PWM
  pinMode(PWMB, OUTPUT);         //TB6612控制引脚，电机PWM
  digitalWrite(IN1, 0);          //TB6612控制引脚拉低
  digitalWrite(IN2, 0);          //TB6612控制引脚拉低
  digitalWrite(IN3, 0);          //TB6612控制引脚拉低
  digitalWrite(IN4, 0);          //TB6612控制引脚拉低
  analogWrite(PWMA, 0);          //TB6612控制引脚拉低
  analogWrite(PWMB, 0);          //TB6612控制引脚拉低
  pinMode(2, INPUT);       //编码器引脚
  pinMode(4, INPUT);       //编码器引脚
  pinMode(5, INPUT);       //编码器引脚
  pinMode(8, INPUT);       //编码器引脚
  pinMode(3, INPUT);       //按键引脚
  Wire.begin();             //加入 IIC 总线
  Serial.begin(9600);       //开启串口，设置波特率为 9600
  delay(1500);              //延时等待初始化完成
  Mpu6050.initialize();     //初始化MPU6050
  delay(20); 
  if(digitalRead(KEY)==0) {    //读取EEPROM的参数
  Balance_Kp =  (float)((EEPROM.read(addr+0)*256)+EEPROM.read(addr+1) )/100;
  Balance_Kd =  (float)((EEPROM.read(addr+2)*256)+EEPROM.read(addr+3))/100;
  Velocity_Kp = (float)((EEPROM.read(addr+4)*256)+EEPROM.read(addr+5))/100;
  Velocity_Ki = (float)((EEPROM.read(addr+6)*256)+EEPROM.read(addr+7))/100;
  }
  MsTimer2::set(5, control);  //使用Timer2设置5ms定时中断
  MsTimer2::start();          //使用中断使能
  attachInterrupt(0, READ_ENCODER_L, CHANGE);           //开启外部中断 编码器接口1
  attachPinChangeInterrupt(4, READ_ENCODER_R, CHANGE);  //开启外部中断 编码器接口2
}
/**************************************************************************
函数功能：主循环程序体
入口参数：无
返回  值：无
**************************************************************************/
void loop() {
 Serial.println(Velocity_L);
}
/**************************************************************************
函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
入口参数：无
返回  值：无
**************************************************************************/
void READ_ENCODER_L() {
  if (digitalRead(ENCODER_L) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L--;  //根据另外一相电平判定方向
    else      Velocity_L++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L++; //根据另外一相电平判定方向
    else     Velocity_L--;
  }
}
/**************************************************************************
函数功能：外部中断读取编码器数据，具有二倍频功能 注意外部中断是跳变沿触发
入口参数：无
返回  值：无
**************************************************************************/
void READ_ENCODER_R() {
  if (digitalRead(ENCODER_R) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R--;//根据另外一相电平判定方向
    else      Velocity_R++;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R++; //根据另外一相电平判定方向
    else     Velocity_R--;
  }
}
/**************************************************************************
函数功能：串口接收中断
入口参数：无
返回  值：无
**************************************************************************/
