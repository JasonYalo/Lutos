#include<reg51.h>
#include<intrins.h>

#define uint unsigned int
#define uchar unsigned char
	
#define At24c02ADDR 0xA0
#define I2cRead 1
#define I2cWrite 0

sbit DU = P2^6;//��ѡ
sbit WE = P2^7;//λѡ

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
//����ܶ�ѡ
uchar code SMGwei[]=
{
	0xFE,0xFD,0xFB
};
//�����λѡ

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
	
	P0=0xFF;//�������
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
	ET0=1;//��ʱ��0���ж�����λ
	TR0 =1;//������ʱ��0
	TMOD =0x01;
	TH0 = 0xed; 
	TL0 = 0xfe;//��ʱ50ms

}
/****************
IICͨ��
*****************/
void delay5us()
{
   _nop_();//��intrins.h�ִ��һ��Լ5us����ʾ��ѭ��һ������ָ���ʱ��
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

/****�������ӻ�Ӧ��***/
bit ReadACK()
{
    SCL =1;
	  delay5us();
	  if(SDA==1)
		{
			SCL=0;
			return(1);//��Ӧ��
		}
		else
		{
	    SCL=0;
      return(0);//Ӧ��	
		}

}

/*****��������Ӧ��*****/
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
	SDA=1;//�ͷ���������
	
}
//
void I2cSendByte(uchar DAT)
{  
	 uchar i;
	 for(i=0;i<8;i++)
	{
   SCL = 0;
	 if(DAT & 0x80)//&�����
		 SDA=1;
	 else
		 SDA=0;
	 SCL=1;
	 DAT<<=1;//����һλ�����λ�Ƴ������λ��0
  }
   SCL=0;
	 SDA=1;
	
}
	

//��ַ1010+000
void At24c02Write(uchar ADDR,uchar DAT)
{
   I2cStart();
	 I2cSendByte(0xA0 + 0);
	 if(ReadACK())
		 AckFlag=1;   //��Ӧ��
	 else
		 AckFlag=0;
	 
	 I2cSendByte(ADDR);//�洢���ĵ�ַ
	 if(ReadACK())
		 AckFlag=1;   //��Ӧ��
	 else
		 AckFlag=0;
	 
	 I2cSendByte(DAT);
	 if(ReadACK())
		 AckFlag=1;   //��Ӧ��
	 else
		 AckFlag=0;
	 
	 I2cStop();
}

//����һ���ֽڵĺ���
uchar I2cReadByte()
{
  uchar i,DAT;
  
	for(i=0;i<8;i++)
	{
	  DAT<<=1;
		SCL = 0;
		SCL = 1;
		if(SDA)
			DAT |=0x01 ;  //�ѽ��ܵĴӻ����ݴ���DAT��
	}
	
  return(DAT);
}
	
uchar At24c02Read(uchar ADDR)
{  
	 uchar DAT;
   I2cStart();
	 I2cSendByte(At24c02ADDR + I2cWrite);
	 if(ReadACK())
		 AckFlag=1;   //��Ӧ��
	 else
		 AckFlag=0;
	 I2cSendByte(ADDR);//�洢���ĵ�ַ
	 ReadACK();
	 I2cStart();
	 I2cSendByte(At24c02ADDR + I2cRead);
	 if(ReadACK())
		 AckFlag=1;   //��Ӧ��
	 else
		 AckFlag=0;
	 DAT=I2cReadByte();
	 SendACK(1);
	 I2cStop();
	 return(DAT);
}

void main()
{ timer0Init();  
	EA=0;//�����ж�
	At24c02Write(2,9);
	delay(1);
	Sec = At24c02Read(2);
//	if(AckFlag)
//		P1=0;
//	else
//		P1=0xFF;
//�������ӻ��Ƿ��Ӧ�� 
	EA=1;
	while(1);
	
}

void timer0() interrupt 1
{
  TH0 = 0xed; 
	TL0 = 0xfe;//��ʱ5ms
//  mSec++;
//	if(mSec == 200)
//	{
//	   mSec =0;
//		Sec++;
//	}
	display(Sec); 
}