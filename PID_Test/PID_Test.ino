#include <SharedData.h>
#include <Encoder.h>

#define MOTOR_DUTY_PIN 5
#define MOTOR_DIRECTION_PIN 6

Encoder motorEncoder(2, 3);

long position = 0;
long prevPosition = 0;

double speed = 0;
double prevSpeed;

long I

unsigned long prevReadMillis = 0;

int duty;

void setup() {
  Serial.begin(9600);
  
  pinMode(MOTOR_DUTY_PIN, OUTPUT);
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);

  analogWrite(MOTOR_DUTY_PIN, 255);
  digitalWrite(MOTOR_DIRECTION_PIN, HIGH);
}

void loop() {
  if (Serial.available() >= 1)
  {
    duty = Serial.parseInt();
    analogWrite(MOTOR_DUTY_PIN, 255 - duty);
    Serial.flush();
  }
  
  position = motorEncoder.read();
  speed = ((double)(position - prevPosition)) / ((double)(millis() - prevReadMillis));
  
  Serial.print((double)(position - prevPosition) / 7);
  Serial.print("\t");
  Serial.print(position - prevPosition);
  Serial.print("\t");
  Serial.print(millis() - prevReadMillis);
  Serial.println("");

  prevPosition = position;
  prevReadMillis = millis();
}
