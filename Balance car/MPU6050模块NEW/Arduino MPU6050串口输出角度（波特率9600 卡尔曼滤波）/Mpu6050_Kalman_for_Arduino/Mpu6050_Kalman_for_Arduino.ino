#include <DATASCOPE.h>      //这是PC端上位机的库文件
DATASCOPE data;
unsigned char Send_Count;  //上位机相关变量
float a;
int show1,show2,show3,show4;

/****************************************************************************
   作者：平衡小车之家
   产品名称：Minibalance For Arduino
****************************************************************************/
#include <KalmanFilter.h>    //卡尔曼滤波
#include <MsTimer2.h>        //定时中断
#include "I2Cdev.h"        
#include "MPU6050_6Axis_MotionApps20.h"//MPU6050库文件
#include "Wire.h"           
MPU6050 Mpu6050; //实例化一个 MPU6050 对象，对象名称为 Mpu6050
KalmanFilter KalFilter;//实例化一个卡尔曼滤波器对象，对象名称为 KalFilter
int16_t ax, ay, az, gx, gy, gz;  //MPU6050的三轴加速度和三轴陀螺仪数据
float angle, Gryo;  //用于显示的角度和临时变量

#define lvbo_LEN 20
int16_t A_X_sz[lvbo_LEN] = {0},A_Z_sz[lvbo_LEN] = {0}, A_X_, A_Z_;
int32_t A_X_Count = 0, A_Z_Count = 0;


//窗口滤波
float Angle_Cale(int16_t a_x, int16_t a_z, int16_t g_y)
{
  
  static int8_t i = 0;
  float Angle;
  A_X_Count = A_X_Count + a_x - A_X_sz[i];
  A_X_sz[i] = a_x; 

  A_Z_Count = A_Z_Count + a_z - A_Z_sz[i];
  A_Z_sz[i] = a_z; 

  i++;
  if(i > lvbo_LEN)
    i = 0;
    
  A_X_ = A_X_Count / lvbo_LEN;
  A_Z_ = A_Z_Count / lvbo_LEN;
  Angle=atan(A_X_ /A_Z_ ) ;
  
return Angle;


}

/**************************************************************************
函数功能：5ms中断服务函数 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void control()
{
  sei();//全局中断开启
  Mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //获取MPU6050陀螺仪和加速度计的数据


  //偏差
    ax-=775;
    az+=1600;
    angle = Angle_Cale(ax,az,1);
    
    
  
}
/**************************************************************************
函数功能：初始化 相当于STM32里面的Main函数 作者：平衡小车之家
入口参数：无
返回  值：无
**************************************************************************/
void setup() {
  Wire.begin();             //加入 IIC 总线
  Serial.begin(128000);       //开启串口，设置波特率为 9600
  delay(1500);              //延时等待初始化完成
  Mpu6050.initialize();     //初始化MPU6050
  delay(20); 
  MsTimer2::set(5, control);  //使用Timer2设置5ms定时中断
  MsTimer2::start();          //使用中断使能
}



void DataScope(void)
{
  int i;
  data.DataScope_Get_Channel_Data(ax, 1);  //显示第一个数据，角度
  data.DataScope_Get_Channel_Data(ay, 2);//显示第二个数据，左轮速度  也就是每40ms输出的脉冲计数
  data.DataScope_Get_Channel_Data(az, 3);//显示第三个数据，右轮速度 也就是每40ms输出的脉冲计数
  data.DataScope_Get_Channel_Data(A_X_, 4);//显示第四个数据，电池电压，单位
  data.DataScope_Get_Channel_Data(A_Z_, 5);
  data.DataScope_Get_Channel_Data(angle, 6);
  Send_Count = data.DataScope_Data_Generate(6); 
  for ( i = 0 ; i < Send_Count; i++)
  {
    Serial.write(DataScope_OutPut_Buffer[i]);  
  }
  delay(50);  //上位机必须严格控制发送时序
}
/**************************************************************************
函数功能：主循环程序体
入口参数：无
返回  值：无
**************************************************************************/
void loop() {
    /*
    Serial.print(ax);   //左轮编码器
    Serial.print("   | ");
    Serial.print(ay);  //右轮编码器
Serial.print("   | ");
    Serial.print(az);
    Serial.print("   | ");
    Serial.print(gx);
    Serial.print("   | ");
    Serial.print(gy);
    Serial.print("   | ");
    Serial.print(gz);
    Serial.println("   | ");*/

    Serial.begin(128000), DataScope();          //延时等待初始化完成
}
