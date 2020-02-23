#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
u8 Way_Angle=1;      //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 （有的6050使用DMP时，需要开机后不停摇晃小车10S左右，等待数据稳定）
float Angle_Balance; 
int main(void)
{ 
	Stm32_Clock_Init(9);            //系统时钟设置
	delay_init(72);                 //延时初始化
	uart_init(72,9600);           //初始化串口1
	IIC_Init();                     //模拟IIC初始化
  MPU6050_initialize();           //=====MPU6050初始化	
	DMP_Init();                     //初始化DMP     
	while(1)
		{
			Read_DMP();                      //===读取倾角
			printf("Roll:%f  ",Roll);  //X 
			printf("Pitch:%f  ",Pitch); //Y
 			printf("Yaw:%f\r\n",Yaw);   //Z
		} 
}
