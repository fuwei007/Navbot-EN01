/*
The SyncWritePos example passed the test in ST3215 Servo, 
and if testing other models of ST series servos
please change the appropriate position, speed and delay parameters.
*/

#include <SCServo.h>
SMS_STS st;

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

byte ID[2];
s16 Position[2];
u16 Speed[2];
byte ACC[2];

void setup()
{
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);
  ID[0] = 1;
  ID[1] = 2;
  Speed[0] = 3400;
  Speed[1] = 3400;
  ACC[0] = 50;
  ACC[1] = 50;
}

void loop()
{
  Position[0] = 3000;
  Position[1] = 3000;
  st.SyncWritePosEx(ID, 2, Position, Speed, ACC);//servo(ID1/ID2) speed=3400，acc=50，move to position=3000.
  delay(2000);

  Position[0] = 100;
  Position[1] = 100;
  st.SyncWritePosEx(ID, 2, Position, Speed, ACC);//servo(ID1/ID2) speed=3400，acc=50，move to position=100.
  delay(2000);
}
