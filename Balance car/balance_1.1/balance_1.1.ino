//左电机
int AIN2=12;
int AIN1=13;
int PWMA=11;
//左电机编码器读取
int LeftA=A0;
int LeftB=A1;
//左电机编码器转化
int La,Lb,La1,Lb1;
int leftcount=0;

int model;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:



}


void serialEvent()
{
 
  La=analogRead(A0);
Lb=analogRead(A1);
if(La>500)La1=HIGH;
else La1=LOW;


if(Lb>500)Lb1=HIGH;
else Lb1=LOW;

if(model != La1)
{
Serial.print(La);    //读取电平
Serial.print("   ");
Serial.print(La1);   //转化为高低电平
Serial.print("   ");
Serial.print(Lb);
Serial.print("   ");
Serial.print(Lb1);
Serial.print("   ");
READL();
Serial.println(leftcount);
delay(10);
}
model=La1;//记录标志位

  
  }

//左计数
void READL() {
  if (La1 == LOW) {     //如果是下降沿触发的中断
    if (Lb1 == LOW)      leftcount--;  //根据另外一相电平判定方向
    else      leftcount++;
  }
  else {     //如果是上升沿触发的中断
    if (Lb1 == LOW)      leftcount++; //根据另外一相电平判定方向
    else     leftcount--;
  }
}
