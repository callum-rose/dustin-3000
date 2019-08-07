void setup() {

  Serial.begin(9600);
  
  pinMode(9,OUTPUT); digitalWrite(9,HIGH);
  
  Serial.println("Enter AT commands:");
  
  Serial3.begin(115200);

}

void loop()

{
  
  if (Serial3.available())
  
  Serial.write(Serial3.read());
  
  if (Serial.available())
  
  Serial3.write(Serial.read());

}
