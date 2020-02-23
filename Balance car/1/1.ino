int analogInPin = A4;
int z;

void setup() {
  // put your setup code here, to run once:
 
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
 z=analogRead(analogInPin);
Serial.println(z);
delay(10);
  
}
