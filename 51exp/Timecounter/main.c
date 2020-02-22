#include<reg51.h>
#include<intrins.h>

#define uint unsigned int
#define uchar unsigned char
	
sbit DU = P2^6;//��ѡ
sbit WE = P2^7;//λѡ
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

void main()
{ timer0Init();  
	while(1);
	
}

void timer0() interrupt 1
{
  TH0 = 0xed; 
	TL0 = 0xfe;//��ʱ5ms
  mSec++;
	if(mSec == 200)
	{
	   mSec =0;
		Sec++;
	}
	display(Sec); 
}