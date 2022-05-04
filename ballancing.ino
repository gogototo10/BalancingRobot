#include <Wire.h>
#include <EEPROM.h>


int Step1Pin = 9;
int Dir1Pin = 3;
int Step2Pin = 11;
int Dir2Pin = 5;

/*300.0,5.7,100.0,-3.9*/ //kp ki kd setpoint
float kp = 3.0;
float ki = 2.0;
float kd = 2.5;
float Setpoint = -1.3;

float XAngle;
class MotorSet {
  public:
    bool Dir = false;
    long int TargetHz = 500;
    int StepPin = 2;
    int DirPin = 3;

    void setinit( bool dir, int targetHz, int stepPin, int dirPin)
    {
      Dir = dir;
      TargetHz = targetHz;
      StepPin = stepPin;
      DirPin = dirPin;
    }


};

MotorSet motor[2];


void initMotor()
{

  motor[0].setinit(false, 500, Step1Pin, Dir1Pin);
  motor[1].setinit(true, 500, Step2Pin, Dir2Pin);

  pinMode(motor[0].StepPin, OUTPUT);
  pinMode(motor[0].DirPin, OUTPUT);
  pinMode(motor[1].StepPin, OUTPUT);
  pinMode(motor[1].DirPin, OUTPUT);

  TCCR2A = _BV(COM2A0) | _BV(WGM21);
  TCCR2B = _BV(CS22) | _BV(CS21) ;//| _BV(CS20);
}

void initMpu6050()
{
  // Wire init
  Wire.begin();
  // Power Management
  Wire.beginTransmission(0x68);
  Wire.write(107);
  Wire.write(0);
  Wire.endTransmission();
  // Register 26
  for (uint8_t i = 2; i <= 7; i++)
  {
    Wire.beginTransmission(0x68);
    Wire.write(26);
    Wire.write(i << 3 | 0x03);
    Wire.endTransmission();
  }
  // Register 27
  Wire.beginTransmission(0x68);
  Wire.write(27);
  Wire.write(3 << 3);
  Wire.endTransmission();
  // Register 28
  Wire.beginTransmission(0x68);
  Wire.write(28);
  Wire.write(0);
  Wire.endTransmission();
}


void inLoopMotor(float XDeg)
{
  static unsigned long currentTime , previousTime;
  static float lastError;
  static float cumError, rateError;
  static float tempHz;

  double elapsedTime;
  float error;

  float tempcumError = cumError;
  currentTime = millis();
  elapsedTime = (float)(currentTime - previousTime);        //compute time elapsed from previous computation

  if (elapsedTime > 30)
  {
    error = Setpoint - XDeg;                                // determine error
    cumError += error * elapsedTime;                // compute integral
    rateError = (error - lastError) / elapsedTime; // compute derivative

    tempHz = kp * error + ki * cumError + kd * rateError;         //PID output


    lastError = error;                                //remember current error
    previousTime = currentTime;                        //remember current time

    if (tempHz > 10000)
    {
      tempHz = 10000;
      cumError = tempcumError;
    }
    else if (tempHz < -10000)
    {
      tempHz = -10000;
      cumError = tempcumError;
    }
    motor[0].TargetHz = int(tempHz);
  }

  if (motor[0].TargetHz > 0)
  {
    motor[0].Dir = false;
    motor[1].Dir = true;
  }
  else
  {
    motor[0].Dir = true;
    motor[1].Dir = false;
  }
  digitalWrite(motor[0].DirPin, motor[0].Dir);
  digitalWrite(motor[1].DirPin, motor[1].Dir);

  uint16_t tempOCR =  (16000000 / (256 * 2.0 * (abs(motor[0].TargetHz)))) - 1;


  if (tempOCR < 1)
  {
    tempOCR = 1;
  }
  else if (tempOCR > 255)
  {
    tempOCR = 255;
  }

  OCR2A = tempOCR;
}

float inLoopMpu6050()
{
  int16_t offset[3] = { -22, 15, -4};
  uint8_t i;
  static int16_t acc_raw[3] = {0,}, gyro_raw[3] = {0,};
  // Get Accel
  Wire.beginTransmission(0x68);
  Wire.write(59);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  for (i = 0; i < 3; i++) acc_raw[i] = (Wire.read() << 8) | Wire.read();
  // Get Gyro
  Wire.beginTransmission(0x68);
  Wire.write(67);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  for (i = 0; i < 3; i++)
    gyro_raw[i] = gyro_raw[i] * 0.8 + 0.2 * (((Wire.read() << 8) | Wire.read()) - offset[i]);
  // Get DT
  static unsigned long p = 0;
  unsigned long c = micros();
  float dt = (c - p) * 0.000001F;
  p = c;
  // Gyro Rate
  float gyro_rate[3];
  for (i = 0; i < 3; i++) gyro_rate[i] = gyro_raw[i] / 16.4 * dt;
  // Calculate
  static float angle[3] = {0,}, vec;
  vec = sqrt(pow(acc_raw[0], 2) + pow(acc_raw[2], 2));
  angle[0] = (angle[0] + gyro_rate[0]) * 0.98
             + atan2(acc_raw[1], vec) * RAD_TO_DEG * 0.02;
  vec = sqrt(pow(acc_raw[1], 2) + pow(acc_raw[2], 2));
  angle[1] = (angle[1] - gyro_rate[1]) * 0.98
             + atan2(acc_raw[0], vec) * RAD_TO_DEG * 0.02;
  // Serial print
  angle[2] += gyro_rate[2];
  return angle[0];
}

void handleSeiral()
{
  static char tempbuffer[8];
  if (Serial.available() > 0)
  {
    char temp = Serial.read();

    for (int i = 0; i < 7; i++)
    {
      tempbuffer[i] = tempbuffer[i + 1];
    }
    tempbuffer[7] = temp;
    if (tempbuffer[0] == '/' && tempbuffer[1] == '*' && tempbuffer[6] == '*' && tempbuffer[7] == '/')
    {
      if (tempbuffer[2] == 'H')
      {
        motor[0].TargetHz = (tempbuffer[3] - '0') * 100 + (tempbuffer[4] - '0') * 10 + (tempbuffer[5] - '0');
        motor[1].TargetHz = motor[0].TargetHz;

      }
      else if (tempbuffer[2] == 'D')
      {
        if ((tempbuffer - '0') == 0)
        {
          motor[0].Dir = false;
          motor[1].Dir = !motor[0].Dir;
        }
        else
        {
          motor[0].Dir = true;
          motor[1].Dir = !motor[0].Dir;
        }
      }
    }
  }
}
void SetPidBySerial()
{
  static char tempbuffer[40] = {NULL,};
  if (Serial.available() > 0)
  {
    char temp = Serial.read();
    int tempLenth = sizeof(tempbuffer) / sizeof(tempbuffer[0]);
    for (int i = 0; i < tempLenth; i++)
    {
      tempbuffer[i] = tempbuffer[i + 1];
    }
    tempbuffer[tempLenth - 1] = temp;

    if (tempbuffer[0] == '/' && tempbuffer[1] == '*' )
    {
      int tempi = 0;
      for (int i = 2; i < tempLenth - 1; i++)
      {
        if (tempbuffer[i] == '*' && tempbuffer[i + 1] == '/')
        {
          tempi = i;
          i = tempLenth;
        }
      }
      if (tempi > 0)
      {
        String pid[4];
        int tempcomma = 0;
        for (int i = 2; i < tempi; i++)
        {
          if (tempbuffer[i] == ',')
          {
            tempcomma++;
          }
          else
          {
            pid[tempcomma] += String(tempbuffer[i]);
          }
        }
        if (tempcomma == 3)
        {
          kp = pid[0].toFloat();
          ki = pid[1].toFloat();
          kd = pid[2].toFloat();
          Setpoint = pid[3].toFloat();
          EEPROM.put(0, kp);
          EEPROM.put(4, ki);
          EEPROM.put(8, kd);
          EEPROM.put(12, Setpoint);
        }
      }

    }
  }
}
void initEEPORM()
{
  int FirstEeporom;
  EEPROM.get(40, FirstEeporom);
  if (FirstEeporom != 7777) //처음실행이 아닐경우
  {
    EEPROM.put(40, 7777);//8888를 넣어 처음 실행이 아님을 저장한다.
    EEPROM.put(0, kp);
    EEPROM.put(4, ki);
    EEPROM.put(8, kd);
    EEPROM.put(12, Setpoint);
  }
  else
  {
    EEPROM.get(0, kp);
    EEPROM.get(4, ki);
    EEPROM.get(8, kd);
    EEPROM.get(12, Setpoint);
  }
  Serial.println();
  Serial.println(kp);
  Serial.println(ki);
  Serial.println(kd);
  Serial.println(Setpoint);
  delay(2000);

}
void setup() {
  Serial.begin(115200);
  initEEPORM();
  initMpu6050();

  initMotor();

}
void loop() {
  //handleSeiral();
  XAngle = inLoopMpu6050();
  inLoopMotor(XAngle);
  SetPidBySerial();
  Serial.println(motor[0].TargetHz);
  Serial.println(XAngle);
}
