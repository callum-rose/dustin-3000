

void setup() {

  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(23, OUTPUT);
  pinMode(22, INPUT);
}

void loop() {
  digitalWrite(23, HIGH);
  delayMicroseconds(10);
  digitalWrite(23, LOW);

  while (digitalRead(22) == LOW);

  long pulseStartTime = micros();

  while (digitalRead(22) == HIGH);

  Serial.println((float)(micros() - pulseStartTime) * 340 / (2 * 1000000));

  delay(1000);
}
