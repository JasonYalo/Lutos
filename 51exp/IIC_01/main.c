#include<reg51.h>
#include<intrins.h>

#define uint unsigned int
#define uchar unsigned char
	
#define At24c02ADDR 0xA0
#define I2cRead 1
#define I2cWrite 0

sbit DU = P2^6;//段选
sbit WE = P2^7;//位选

sbit SCL = P2^1;
sbit SDA = P2^0;

bit AckFlag;
uchar mSec,Sec;


uchar code SMGduan[]=
{ 0x3f,0x06,0x5b,0x4f,
	0x66,0x6d,0x7d,0x07,
	0x7f,0x6f,0x77,0x7c,
	0x39,0x5e,0x79,0x71
};
//数码管段选
uchar code SMGwei[]=
{
	0xFE,0xFD,0xFB
};
//数码管位选

void delay(uint z)
{
    uint x,y;
    for(x=z;x>0;x--)
	{
	  for(y=144;y>0;y--);
	}
}

void display(uchar i)
{
   static uchar wei;
	
	P0=0xFF;//清楚段码
	WE=1;
	P0=SMGwei[wei];
	WE=0;
	switch(wei)
		{case 0: DU=1;P0=SMGduan[i/100];DU=0;break;
		 case 1: DU=1;P0=SMGduan[i%100/10];DU=0;break;
		 case 2: DU=1;P0=SMGduan[i%10];DU=0;break;
	
	}
	wei++;
	if(wei==3)
	{wei=0;}
}


void timer0Init()
{
  EA=1;
	ET0=1;//定时器0的中断允许位
	TR0 =1;//启动定时器0
	TMOD =0x01;
	TH0 = 0xed; 
	TL0 = 0xfe;//定时50ms

}
/****************
IIC通信
*****************/
void delay5us()
{
   _nop_();//在intrins.h里，执行一次约5us，表示空循环一个机器指令的时间
}

void I2cStart()
{
  SCL=1;
	SDA=1;
	delay5us();
	SDA=0;
	delay5us();
	
}

void I2cStop()
{
  SCL=0;
	SDA=0;
	SCL=1;
	delay5us();
	SDA=1;
	delay5us();

}

/****主机读从机应答***/
bit ReadACK()
{
    SCL =1;
	  delay5us();
	  if(SDA==1)
		{
			SCL=0;
			return(1);//非应答
		}
		else
		{
	    SCL=0;
      return(0);//应答	
		}

}

/*****主机发送应答*****/
void SendACK(bit i)
{
  SCL=0;
	if(i==1)
		SDA =1;
	if(i==0)
		SDA =0;
	SCL=1;
	delay5us();
	SCL=0;
	SDA=1;//释放数据总线
	
}
//
void I2cSendByte(uchar DAT)
{  
	 uchar i;
	 for(i=0;i<8;i++)
	{
   SCL = 0;
	 if(DAT & 0x80)//&运算符
		 SDA=1;
	 else
		 SDA=0;
	 SCL=1;
	 DAT<<=1;//左移一位，最高位移除，最低位补0
  }
   SCL=0;
	 SDA=1;
	
}
	

//地址1010+000
void At24c02Write(uchar ADDR,uchar DAT)
{
   I2cStart();
	 I2cSendByte(0xA0 + 0);
	 if(ReadACK())
		 AckFlag=1;   //无应答
	 else
		 AckFlag=0;
	 
	 I2cSendByte(ADDR);//存储区的地址
	 if(ReadACK())
		 AckFlag=1;   //无应答
	 else
		 AckFlag=0;
	 
	 I2cSendByte(DAT);
	 if(ReadACK())
		 AckFlag=1;   //无应答
	 else
		 AckFlag=0;
	 
	 I2cStop();
}

//接收一个字节的函数
uchar I2cReadByte()
{
  uchar i,DAT;
  
	for(i=0;i<8;i++)
	{
	  DAT<<=1;
		SCL = 0;
		SCL = 1;
		if(SDA)
			DAT |=0x01 ;  //把接受的从机数据存在DAT中
	}
	
  return(DAT);
}
	
uchar At24c02Read(uchar ADDR)
{  
	 uchar DAT;
   I2cStart();
	 I2cSendByte(At24c02ADDR + I2cWrite);
	 if(ReadACK())
		 AckFlag=1;   //无应答
	 else
		 AckFlag=0;
	 I2cSendByte(ADDR);//存储区的地址
	 ReadACK();
	 I2cStart();
	 I2cSendByte(At24c02ADDR + I2cRead);
	 if(ReadACK())
		 AckFlag=1;   //无应答
	 else
		 AckFlag=0;
	 DAT=I2cReadByte();
	 SendACK(1);
	 I2cStop();
	 return(DAT);
}

void main()
{ timer0Init();  
	EA=0;//屏蔽中断
	At24c02Write(2,9);
	delay(1);
	Sec = At24c02Read(2);
//	if(AckFlag)
//		P1=0;
//	else
//		P1=0xFF;
//用来检测从机是否非应答 
	EA=1;
	while(1);
	
}

void timer0() interrupt 1
{
  TH0 = 0xed; 
	TL0 = 0xfe;//定时5ms
//  mSec++;
//	if(mSec == 200)
//	{
//	   mSec =0;
//		Sec++;
//	}
	display(Sec); 
}