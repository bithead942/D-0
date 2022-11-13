/************************************************************************
  D-O Control for HotRC D600
  by Bithead942

  Board:  Arduino Mega

  Controls Sound for D-O

  Pins:
   0 - Serial TX
   1 - Serial RX
   2 -
   3 - Motor Controller - INPUT RC mixed
   4 - Motor Controller - INPUT RC mixed

   8 - Servo - neck tilt
   9 - Servo - arm tilt
  10 - Motor Controller - OUTPUT Speed 2
  11 - Motor Controller - OUTPUT Direction 2
  12 - Motor Controller - OUTPUT Speed 1
  13 - Motor Controller - OUTPUT Direction 1

  53 - Sound FX - Audio Trigger

*************************************************************************/
#include <Wire.h>
#include <Servo.h>

const int _chHeadTrigger = 5;

int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float elapsedTime, time, timePrev;

int ArmTiltPos = 90;
int NeckTiltPos = 90;
int ArmTiltTargetPos = 90;
int NeckTiltTargetPos = 90;
int Acc_Y_Average = 0;
const int NeckTrim = 5;
const int ArmTrim = 5;
const int MoveIncrement = 3;
// create servo object to control a servo
Servo ArmTiltServo;
Servo NeckTiltServo;

void setup()
{
  pinMode(_chHeadTrigger, INPUT);   //RC channel for Head Trigger

  Wire.begin(); /////////////TO BEGIN I2C COMMUNICATIONS///////////////
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // ** uncomment to allow debugging**
  //Serial.begin(9600);
  //Serial.println("Ready");

  // attaches the servo on pin 9 to the servo object
  NeckTiltServo.attach(8);
  ArmTiltServo.attach(9);
  NeckTiltServo.write(90 + NeckTrim);  // start level with ground
  ArmTiltServo.write(90 + ArmTrim);   // start straight up
  delay(1000);  //Wait for servos to center

  time = millis(); ///////////////STARTS COUNTING TIME IN MILLISECONDS/////////////
}


void loop()
{
  int x = 0;

  int HeadTrigger = pulseIn(_chHeadTrigger, HIGH, 25000); // RC channel for Sound Trigger

  if (HeadTrigger <= 1400)
  {
    NeckTiltServo.write(90 + NeckTrim);  // level with ground
    ArmTiltServo.write(90 + ArmTrim);   // straight up
  }
  else
  {

    for (x = 0; x < 50; x++)
    {
      timePrev = time;
      time = millis();
      elapsedTime = (time - timePrev) / 1000;

      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 6, true);
      ////////////////////PULLING RAW ACCELEROMETER DATA FROM IMU/////////////////
      Acc_rawX = Wire.read() << 8 | Wire.read();
      Acc_rawY = Wire.read() << 8 | Wire.read();
      Acc_rawZ = Wire.read() << 8 | Wire.read();
      int Acc_Y = map(Acc_rawY, -16000, 16000, -90, 270);  //smooth it so the angle generally falls between 0 and 180
      if (Acc_Y < 60) {  //Set floor
        Acc_Y = 60;
      }
      if (Acc_Y > 105) {  //Set ceiling
        Acc_Y = 105;
      }
      //Serial.print("aY = "); Serial.print(String(Acc_Y));
      //Serial.println();

      Acc_Y_Average += Acc_Y;
    }

    Acc_Y_Average /= x;
    //Serial.print("| Avg = "); Serial.print(String(Acc_Y_Average));
    //Serial.println();
    MoveHead (180 - Acc_Y_Average);
  }

}

void MoveHead(int TargetPos) {
  if (NeckTiltPos <= TargetPos - MoveIncrement || NeckTiltPos >= TargetPos + MoveIncrement) {  //avoid jitter between 2 positions
    if (NeckTiltPos < TargetPos) {
      //ArmTiltPos += MoveIncrement;
      //ArmTiltServo.write(ArmTiltPos + ArmTrim);
      NeckTiltPos += MoveIncrement;
      NeckTiltServo.write(NeckTiltPos + NeckTrim);
    }
    else if (NeckTiltPos > TargetPos) {
      //ArmTiltPos -= MoveIncrement;
      //ArmTiltServo.write(ArmTiltPos + ArmTrim);
      NeckTiltPos -= MoveIncrement;
      NeckTiltServo.write(NeckTiltPos + NeckTrim);
    }
  }
}
