
// These constants won't change. They're used to give names to the pins used:
const int analogInPin = A5;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 10; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
byte inByte;
int quan = 0;
int mo=1;          //做霍尔传感器的标识
long time1;
long t1=0;
long t3,t2;
float speed1,speed2;//1是圈下来的平均速度，2是40圈平均速度（单位秒）
int sp[40];
int i=0;
int j;


void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(115200);
}

void loop() {
  if(Serial.available()>0){
    inByte=Serial.read();
    while(1){
      time1=millis(); //计时函数 毫秒
      switch(inByte){
        case('1'):
      
          
          analogWrite(analogOutPin,255); 
          sensorValue=analogRead(analogInPin);
          delayMicroseconds(1);
          Serial.print("Speed:255; Sensor:   ");
          Serial.print(sensorValue);
          delayMicroseconds(1);
          inByte=Serial.read();
         
          
          
          if(inByte!='2' and inByte!='3')
            inByte='1';
          else
            break;
          break;

        case('2'):
          delayMicroseconds(1);
          analogWrite(analogOutPin, 100); 
          sensorValue=analogRead(analogInPin);
          delayMicroseconds(1);
          Serial.print("Speed:100; Sensor:\t");
          Serial.print(sensorValue);
          delayMicroseconds(1);
          inByte=Serial.read();
          if(inByte!='1' or inByte!='3')
            inByte='2';
          else
            break;
          break;

        case('3'):
            delayMicroseconds(1);
            analogWrite(analogOutPin, 60); 
            sensorValue=analogRead(analogInPin);
            delayMicroseconds(1);
            Serial.print("Speed:0; Sensor:\t");
            Serial.println(sensorValue);
            delayMicroseconds(1);
            inByte=Serial.read();
            if(inByte!='2' or inByte!='1')
              inByte='3';
            else
              break;
            break;
        default:
          Serial.print("4：\t");
          Serial.println(inByte);
          delayMicroseconds(1);
          analogWrite(analogOutPin, 0); 
          Serial.println("Speed:0;");
          inByte=Serial.read();
          if(inByte=='1' or inByte=='2' or inByte=='3')
            break;
          
          }
      
          
          if(mo==1)
          {
            if(sensorValue<10)
            {
             quan++;
             mo=0;
             t2=time1;
             t3=t2-t1;
             t1=t2;
            }
          }

          
          if(sensorValue>100)
          {mo=1;
          }

          speed1=1000/t3;
            
          Serial.print(" ");
          Serial.print(quan);
          Serial.print(" ");
          Serial.print(time1);
          Serial.print("     ");
          Serial.print(t3);
          Serial.print("   Speed:");
          Serial.print(speed1);
          Serial.print("     ");

          //计算风扇速度
          if(i<40){
          sp[i]=speed1;
        
          int leiji;
          for(j=0;j<i;j++)
          {
            leiji +=sp[j];
            }
          speed2=(leiji * 60)/(i+1);
          Serial.println(speed2);
          
          i++;  
          }

          else 
          {int leiji;
          for(j=0;j<40;j++)
          {sp[j]=sp[j+1];
           leiji += sp[j];
           
            }
          sp[40]=speed1;
          leiji +=sp[40];
          speed2=(leiji * 60)/40;
          Serial.println(speed2);
            
            
            
            }

          


          
    } 
  }
  //delayMicroseconds(1);
}
