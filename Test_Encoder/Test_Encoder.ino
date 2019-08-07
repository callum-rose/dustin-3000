long encoderCount1 = 0;
long encoderCount2 = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("1: ");
  Serial.print(encoderCount1);
  Serial.print(", 2: ");
  Serial.print(encoderCount2);

  Serial.println("");
}

void encoderSignal1()
{
  encoderCount1++;
}

void encoderSignal2()
{
  encoderCount2++;
}
