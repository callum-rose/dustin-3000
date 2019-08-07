#include <TimerThree.h>

#include <Thread.h>
#include <ThreadController.h>

#include <Encoder.h>
#include <PID_v1.h>

#include <NewPing.h>

#define BACKWARD 0
#define FORWARD 1

#define BATTERYREADPIN A7

// *******
// Sonar
// *******

class SonarThread : public Thread
{
public:
  float distance;

  SonarThread (int trigPin, int echoPin, int maxDist) : Thread ()
  {
    m_sensor = new NewPing(trigPin, echoPin, maxDist);
    m_maxDist = maxDist;
  }

  void run ()
  {
    distance = 0;
    float tempDist;
    int countedPings = 0;
    for (int i = 0; i < m_repeatCount; i++)
    {
      tempDist = m_sensor->ping();
      if (tempDist == 0 || tempDist > m_maxDist * US_ROUNDTRIP_CM)
      {
        continue;
      }
       
      distance += tempDist;
      countedPings++;
    }
    
    if (countedPings > 0)
    {
      distance /= US_ROUNDTRIP_CM * countedPings;
    }
    
    if (distance > m_maxDist)
    {
      distance = 0;
    }
     
    runned();
  }

private:
  NewPing* m_sensor;
  int m_maxDist;

  const int m_repeatCount = 3;
};

SonarThread sonar1Thread = SonarThread(30, 30, 20);
SonarThread sonar2Thread = SonarThread(31, 31, 20);

// *******
// Motors
// *******

class MotorThread : public Thread
{
public:

  float encoderRpm;
  float shaftRpm;
  float shaftPos;

  MotorThread (String name, int pwmPin, int directionPin, int encoderAPin, int encoderBPin, int encoderPulsesPerRev, float gearRatio) : Thread()
  {
    m_name = name;
    
    m_pwmPin = pwmPin;
    m_directionPin = directionPin;
    
    pinMode(m_pwmPin, OUTPUT);
    pinMode(m_directionPin, OUTPUT);
        
    m_encoder = new Encoder(encoderAPin, encoderBPin);
    m_encoder2ShaftRatio = 1.0 / (gearRatio * (float)encoderPulsesPerRev);

    m_pid = new PID(&m_inputSpeed, &m_outputPwm, &m_setpointSpeed, 0.2, 10, 0.05, DIRECT);
    m_pid->SetOutputLimits(-255, 255);
    m_pid->SetMode(AUTOMATIC);
  }

  void run()
  {
    long currentEncoderPosition = m_encoder->read();
    long currentTime = micros();

    long encoderPositionDelta = currentEncoderPosition - m_lastEncoderPosition;
    long timeDelta = currentTime - m_lastEncoderReadTime;
    
    shaftPos = (float)currentEncoderPosition * m_encoder2ShaftRatio;

    encoderRpm = (float)encoderPositionDelta / timeDelta;
    shaftRpm = encoderRpm * 60 * 1000000 * m_encoder2ShaftRatio;

    // do PID

    m_inputSpeed = -shaftRpm;

    if (m_setpointSpeed != 0)
    {
      m_pid->Compute();
      setPwmDuty(m_outputPwm);

      //Serial.println(m_outputPwm);
    }
    else
    {
      m_pid->Compute();
      setPwmDuty(0);
    }

//    if (m_name == "Starboard Motor")
//    {
//      Serial.print(m_inputSpeed);
//      Serial.print(" ");
//      Serial.print(m_setpointSpeed);
//      Serial.print(" ");
//      Serial.println(m_outputPwm);
//    }
    
    // save for next run
    
    m_lastEncoderPosition = currentEncoderPosition;
    m_lastEncoderReadTime = currentTime;

    runned();
  }

  void setDesiredSpeed(double speed)
  {
    if (sign(m_setpointSpeed) != sign(speed))
    {
      // go to zero first
      
    }
    
    m_setpointSpeed = speed;

//    if (m_setpointSpeed == 0)
//    {
//      m_pid->SetMode(MANUAL);
//    }
//    else
//    {
//      m_pid->SetMode(AUTOMATIC);
//    }
  }

  float getPwm()
  {
    return m_outputPwm;
  }

  float getShaftPos()
  {
    return shaftPos;
  }

  float getEncoderRpm()
  {
    return encoderRpm;
  }

  float getShaftRpm()
  {
    return shaftRpm;
  }

private:
  String m_name;
  
  int m_pwmPin;
  int m_directionPin;
  Encoder* m_encoder;

  PID* m_pid; 
  double m_inputSpeed, m_setpointSpeed;
  double m_outputPwm;

  float m_encoder2ShaftRatio;

  long m_lastEncoderPosition;
  long m_lastEncoderReadTime;

  // speed value between -255 and 255
  void setPwmDuty (double duty)
  {    
    int direction;
    if (duty > 0)
    {
      direction = FORWARD;
    }
    else
    {
      direction = BACKWARD;
    }

    duty = abs(duty);
    
    if (duty > 255)
    {
      duty = 255;
    }
    
    if (direction == HIGH)
    {
      analogWrite(m_pwmPin, (int)(255 - duty));
    }
    else
    {
      analogWrite(m_pwmPin, (int)duty);
    }
    
    digitalWrite(m_directionPin, direction);
  }

  int sign (double value)
  {
    if (value > 0)
    {
      return 1;
    }
    if (value < 0)
    {
      return -1;
    }
    return 0;
  }
};

MotorThread starboardMotorThread("Starboard Motor", 8, 22, 2, 3, 44, 64.99); // the one by itself
MotorThread portsideMotorThread("Portside Motor", 9, 23, 18, 19, 44, 64.99); 
MotorThread agitatorMotorThread("Agitator Motor", 10, 24, 20, 21, 44, 40);

// *******
// Battery
// *******

//class VoltTime
//{
//public:
//  float voltage;
//  long time;
//
//  VoltTime(float _voltage, long _time)
//  {
//    voltage = _voltage;
//    time = _time;
//  }
//
//};

class BatteryThread : public Thread
{
public:
  float lastVoltage;
  
  bool isCriticallyLow = false;

  BatteryThread (int readPin) : Thread ()
  {
    m_readPin = readPin;

//    m_voltages = LinkedList<VoltTime>();
  }
  
  void run ()
  {
    lastVoltage = readBatteryVoltage();

    // low voltage check
    if (lastVoltage < 11.5)
    {
      if (!m_hasSentLowVoltageMessage)
      {
        Serial2.print("Battery critically low at ");
        Serial2.print(lastVoltage);
        Serial2.println("V");

        m_hasSentLowVoltageMessage = true;
      }
    } 
    else if (lastVoltage < 11.3)
    {
      // 11.1V should be the lowest limit
      Serial2.print("Battery critically low at ");
      Serial2.print(lastVoltage);
      Serial2.println("V");

      isCriticallyLow = true;
    } 
    else
    {      
      isCriticallyLow = false;
      m_hasSentLowVoltageMessage = false;
    }

    // add log
//    VoltTime voltTime(lastVoltage, millis());
//    m_voltages.add(voltTime);
    
    runned();
  }

  float readBatteryVoltage()
  {
    int value = 0;
  
    int i;
    for (i = 0; i < 3; i++)
      value += analogRead(m_readPin);
    value /= i;
    
    // compensate for voltage divider
    float multiplier = (float)(4700 + 2200) / 2200;
    
    return (multiplier * value) * (5.00 / 1023.00);
  }

private:
  int m_readPin;

  bool m_hasSentLowVoltageMessage;

//  LinkedList<VoltTime> m_voltages;
};

BatteryThread batteryThread = BatteryThread(BATTERYREADPIN);

// *******
// Bluetooth
// *******

Thread btCheckThread = Thread();

void checkBT()
{ 
  if (Serial2.available())
  {
    String message = "";

    char c;
    while (Serial2.available() > 0)
    {
      c = (char)Serial2.read();
      if (c != 92)
      {
        message += c;
      }
      else
      {
        break;
      }
    }

    Serial.print("BT CMD: ");
    Serial.println(message);

    if (message[0] == 'F')
    {
      if (message.length() == 1)
      {
        starboardMotorThread.setDesiredSpeed(80);
        portsideMotorThread.setDesiredSpeed(-80);
      }
      else
      {
        String numStr = message.substring(1, message.length());
        int speed = (int)numStr.toInt();
        starboardMotorThread.setDesiredSpeed(speed);
        portsideMotorThread.setDesiredSpeed(-speed);
      }
    }
    else if (message[0] == 'B')
    {
      if (message.length() == 1)
      {
        starboardMotorThread.setDesiredSpeed(-80);
        portsideMotorThread.setDesiredSpeed(80);
      }
      else
      {
        String numStr = message.substring(1, message.length());
        int speed = (int)numStr.toInt();
        starboardMotorThread.setDesiredSpeed(-speed);
        portsideMotorThread.setDesiredSpeed(speed);
      }
    }
    else if (message == "stop")
    {
      starboardMotorThread.setDesiredSpeed(0);
      portsideMotorThread.setDesiredSpeed(0);
      agitatorMotorThread.setDesiredSpeed(0);
    }
    else if (message == "dispense")
    {
      agitatorMotorThread.setDesiredSpeed(-1000);
    }
    else if (message == "spread")
    {
      starboardMotorThread.setDesiredSpeed(80);
      portsideMotorThread.setDesiredSpeed(80);
      agitatorMotorThread.setDesiredSpeed(-200);
    } 
    else if (message == "bat")
    {        
      Serial2.print("Battery at ");
      Serial2.print(batteryThread.readBatteryVoltage());
      Serial2.println("V");    
    } 
    else if (message == "time")
    {
//      Serial2.println(getTimeString());
    }
  }

  if (Serial2.available() > 0)
  {
    // more commands
    checkBT();
  }
}

// *******
// I2C
// *******

//#define SDA_PORT PORTB
//#define SDA_PIN 5
//#define SCL_PORT PORTB
//#define SCL_PIN 6
//#include <SoftWire.h>
//
//SoftWire Wire = SoftWire();
//
//// *******
//// DateTime Chip
//// *******
//
//byte decToBcd(byte val)
//{
//  return( (val/10*16) + (val%10) );
//}
//
//// Convert binary coded decimal to normal decimal numbers
//byte bcdToDec(byte val)
//{
//  return( (val/16*10) + (val%16) );
//}
//
//void SetTime(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
//{
//  // sets time and date data to DS3231
//  Wire.beginTransmission(0x68);
//  
//  Wire.write(0); // set next input to start at the seconds register
//  Wire.write(decToBcd(second)); // set seconds
//  Wire.write(decToBcd(minute)); // set minutes
//  Wire.write(decToBcd(hour)); // set hours
//  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
//  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
//  Wire.write(decToBcd(month)); // set month
//  Wire.write(decToBcd(year)); // set year (0 to 99)
//  
//  Wire.endTransmission();
//}
//
//void GetTime(byte *second, byte *minute, byte *hour, byte *dayOfWeek, byte *dayOfMonth, byte *month, byte *year)
//{
//  Wire.beginTransmission(0x68);
//  Wire.write(0); // set DS3231 register pointer to 00h
//  Wire.endTransmission();
//  Wire.requestFrom(0x68, 7);
//  
//  // request seven bytes of data from DS3231 starting from register 00h
//  *second = bcdToDec(Wire.read() & 0x7f);
//  *minute = bcdToDec(Wire.read());
//  *hour = bcdToDec(Wire.read() & 0x3f);
//  *dayOfWeek = bcdToDec(Wire.read());
//  *dayOfMonth = bcdToDec(Wire.read());
//  *month = bcdToDec(Wire.read());
//  *year = bcdToDec(Wire.read());
//}
//
//String getTimeString ()
//{
//  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
//  // retrieve data from DS3231
//  GetTime(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month,
//  &year);
//
//  String timeString = "";
//  timeString += (int)year;
//  timeString += "/";
//  timeString += (int)month;
//  timeString += "/";
//  timeString += (int)dayOfMonth;
//  timeString += " ";
//
//  if (hour<10)
//    timeString += "0";
//  timeString += (int)hour;
//  timeString += ":";
//  if (minute<10)
//    timeString += "0";
//  timeString += (int)minute;
//  timeString += ":";
//  if (second<10)
//    timeString += "0";
//  timeString += (int)second;
//
//  return timeString;
//}

// *******
// Timer
// *******

void timerCallback()
{
  if (starboardMotorThread.shouldRun())
    starboardMotorThread.run();
  if (portsideMotorThread.shouldRun())
    portsideMotorThread.run();
  if (agitatorMotorThread.shouldRun())
    agitatorMotorThread.run();
  if (btCheckThread.shouldRun())
    btCheckThread.run();
  if (batteryThread.shouldRun())
    batteryThread.run();
//  if (sonar1Thread.shouldRun())
//    sonar1Thread.run();
//  if (sonar2Thread.shouldRun())
//    sonar2Thread.run();
}

void setup() 
{
  Serial.begin(9600);
  Serial2.begin(115200);

  // battery
  pinMode(BATTERYREADPIN, INPUT);
  
  // Threads
  
  sonar1Thread.setInterval(500); // in ms
  sonar2Thread.setInterval(500);

  btCheckThread.setInterval(100);
  btCheckThread.onRun(checkBT);

  batteryThread.setInterval(5000);

  starboardMotorThread.setInterval(100);
  portsideMotorThread.setInterval(100);
  agitatorMotorThread.setInterval(100);
  
  // Timer

  Timer3.initialize(50000); // every X us
  Timer3.attachInterrupt(timerCallback);
  Timer3.start();

  starboardMotorThread.setDesiredSpeed(0);
  portsideMotorThread.setDesiredSpeed(0);
  agitatorMotorThread.setDesiredSpeed(0);

  // I2C

//  if (!i2c_init())
//    Serial.println("I2C init failed");
}

void loop() 
{
  Serial.print(sonar1Thread.distance);
  Serial.print(" ");
  Serial.println(sonar2Thread.distance);

  delay(500);
}
