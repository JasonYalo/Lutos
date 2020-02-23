#include <Wire.h>

int16_t ax,ay,az,tap;

void setup() {
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.endTransmission(true);
  Serial.begin(9600);
  delay(500);
  
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
Wire.beginTransmission(0x68);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(0x68,8,true);

ax=Wire.read() << 8 | Wire.read();
ay=Wire.read() << 8 | Wire.read();
az=Wire.read() << 8 | Wire.read();
tap=Wire.read() << 8 | Wire.read();


Serial.print(" ax= ");
Serial.print(ax);
Serial.print("  | ");

Serial.print(" ay= ");
Serial.print(ay);
Serial.print("  | ");

Serial.print(" az= ");
Serial.print(az);
Serial.print("  | ");


Serial.print(" tap= ");
Serial.println(tap);
delay(500);

}
