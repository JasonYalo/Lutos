#include <DATASCOPE.h>      //这是PC端上位机的库文件
DATASCOPE data;
unsigned char Send_Count;  //上位机相关变量
float a;
int show1,show2,show3,show4;
/***************************************************************************/
#include <KalmanFilter.h>    //卡尔曼滤波
#include <MsTimer2.h>       
#include "I2Cdev.h"        
#include "MPU6050_6Axis_MotionApps20.h"//MPU6050库文件
#include "Wire.h"           
MPU6050 Mpu6050; //实例化一个 MPU6050 对象，对象名称为 Mpu6050
KalmanFilter KalFilter;//实例化一个卡尔曼滤波器对象，对象名称为 KalFilter
int16_t ax, ay, az, gx, gy, gz;  //MPU6050的三轴加速度和三轴陀螺仪数据
float Angle, Gryo;  //用于显示的角度和临时变量
//***************下面是卡尔曼滤波相关变量***************//
float K1 = 0.05; // 对加速度计取值的权重
float Q_angle = 0.001, Q_gryo = 0.005;
float R_angle = 0.5 , C_0 = 1;
float dt = 0.005; //注意：dt的取值为滤波器采样时间 5ms

/**************************************************************************
函数功能：5ms中断服务函数 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void control()
{
  sei();//全局中断开启
  Mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //获取MPU6050陀螺仪和加速度计的数据
  KalFilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gryo, R_angle, C_0, K1);          //通过卡尔曼滤波获取角度
  Angle = KalFilter.angle;
  Gryo= KalFilter.Gyro_y;
}

void DataScope(void)
{
  int i;
  data.DataScope_Get_Channel_Data(ax, 1);  //显示第一个数据，角度
  data.DataScope_Get_Channel_Data(ay, 2);//显示第二个数据，左轮速度  也就是每40ms输出的脉冲计数
  data.DataScope_Get_Channel_Data(az, 3);//显示第三个数据，右轮速度 也就是每40ms输出的脉冲计数
  data.DataScope_Get_Channel_Data(show4, 4);//显示第四个数据，电池电压，单位V
  Send_Count = data.DataScope_Data_Generate(4); 
  for ( i = 0 ; i < Send_Count; i++)
  {
    Serial.write(DataScope_OutPut_Buffer[i]);  
  }
  delay(50);  //上位机必须严格控制发送时序
}

/**************************************************************************

**************************************************************************/
void setup() {
  Wire.begin();             //加入 IIC 总线
  Serial.begin(115200);       //开启串口，设置波特率为 9600
  delay(1500);              //延时等待初始化完成
  Mpu6050.initialize();     //初始化MPU6050
  delay(20); 
  MsTimer2::set(5, control);  //使用Timer2设置5ms定时中断
  MsTimer2::start();          //使用中断使能
}

/*
**************************************************************************/
void loop() {
   Serial.begin(115200), DataScope(); //使用上位机时，波特率是128000
     
}
