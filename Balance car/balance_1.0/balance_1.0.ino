#include <PinChangeInt.h>  

//左电机
int AIN2=13;
int AIN1=12;
int PWMA=10;
//左电机编码器读取
int LeftA=2;
int LeftB=5;
//左电机编码器转化
int La,Lb,La1,Lb1;
int leftcount=0;//脉冲计数

int model;

int zhixing=0;

void setup() {
  // put your setup code here, to run once:
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(LeftA,INPUT);
  pinMode(LeftB,INPUT);
Serial.begin(9600);
  //attachInterrupt(0, READR, CHANGE);//A相2管脚，B相5
   attachPinChangeInterrupt(2, READL, CHANGE); 
}

void loop() {
  // put your main code here, to run repeatedly:


if(leftcount < 780)
{
  
digitalWrite(AIN2,LOW);
digitalWrite(AIN1,HIGH);
analogWrite(PWMA,40);
}
if(leftcount >780 )
{
  
digitalWrite(AIN2,HIGH);
digitalWrite(AIN1,LOW);
analogWrite(PWMA,20);
}
if(leftcount ==780){
 analogWrite(PWMA,0);
 delay(10);}


La1=digitalRead(LeftA);

Lb1=digitalRead(LeftB);

if(Lb>500)Lb1=HIGH;
else Lb1=LOW;

if(La1 != model){
Serial.print(La1);   //转化为高低电平
Serial.print("   ");

Serial.print(Lb1);
Serial.print("   ");
Serial.print(zhixing);
Serial.print("   ");
Serial.println(leftcount);
    

}
model=La1;//记录标志位

}

//左计数
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




//电机状态
void motor_L(int dir,int pwm)
{  
 if(dir==1)
 {digitalWrite(AIN2,LOW);
  digitalWrite(AIN1,HIGH);
 }
 if(dir==0)
 {digitalWrite(AIN2,HIGH);
  digitalWrite(AIN1,LOW); 
 }
  analogWrite(PWMA,pwm);
   
  }
