
#include <DATASCOPE.h>      //这是PC端上位机的库文件
#include <PinChangeInt.h>    //外部中断
#include <MsTimer2.h>        //定时中断
DATASCOPE data;//实例化一个 上位机 对象，对象名称为 data
unsigned char Send_Count;  //上位机相关变量


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

//PID
float P_KP=100,P_KI=0,P_KD=500;      //PID系数
float Motor;
float Target_leftcount=10390;//目标位置



//shangweiji///////////////////////

void DataScope(void)
{
  int i,d;
  d=leftcount-10000;
  data.DataScope_Get_Channel_Data(d, 1);  //显示第一个数据，角度
  data.DataScope_Get_Channel_Data(La1, 2);//显示第二个数据，左轮速度  也就是每40ms输出的脉冲计数
  data.DataScope_Get_Channel_Data(Lb1, 3);//显示第三个数据，右轮速度 也就是每40ms输出的脉冲计数
 
  Send_Count = data.DataScope_Data_Generate(3); 
  for ( i = 0 ; i < Send_Count; i++)
  {
    Serial.write(DataScope_OutPut_Buffer[i]);  
  }
  delay(50);  //上位机必须严格控制发送时序
}


/**************************************************************************/
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
/**************************************************************************/
void control()
{  
      static float Voltage_All,Voltage_Count;//电压采样相关变量
      int Temp;//临时变量
      static unsigned char Count_Velocity;  //位置控制分频用的变量
      sei();//全局中断开启
    
       Motor=Position_PID(leftcount,Target_leftcount);    //===位置PID控制器
       
     
         Set_PwmL(Motor);    //如果不存在异常  输出电机控制量
         //如果不存在异常  输出电机控制量
     
 }

///////////////////////////////////////////////////////////



void setup() {
  // put your setup code here, to run once:
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(LeftA,INPUT);
  pinMode(LeftB,INPUT);

  digitalWrite(AIN1, 0);        
  digitalWrite(AIN2, 0);          
  digitalWrite(PWMA, 0);          
  Serial.begin(115200);
  delay(1000);
  MsTimer2::set(10, control);       //使用Timer2设置5ms定时中断
  MsTimer2::start();               //中断使能
  //attachInterrupt(0, READR, CHANGE);//A相2管脚，B相5
  attachPinChangeInterrupt(2, READL, CHANGE); 
}

void loop() {


 
 La1=digitalRead(LeftA);
 Lb1=digitalRead(LeftB);

 if(La1 != model){

 Serial.println(leftcount);
    

  }
 model=La1;//记录标志位
 Serial.begin(115200), DataScope(); //使用上位机时，波特率是128000

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




//电机状态/////////////////////////////////////////////////////////////////////
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


  ///////////////RRRRRRRRRRRRRRRRRRR/////
  



  /***********************************PWM电机控制***************************************/
void Set_PwmL(int motor){

  if (motor< 0)      digitalWrite(AIN1, HIGH),      digitalWrite(AIN2, LOW);  //TB6612的电平控制
  else               digitalWrite(AIN1, LOW),       digitalWrite(AIN2, HIGH); //TB6612的电平控制
  analogWrite(PWMA, abs(motor)); //赋值给PWM寄存器
}
