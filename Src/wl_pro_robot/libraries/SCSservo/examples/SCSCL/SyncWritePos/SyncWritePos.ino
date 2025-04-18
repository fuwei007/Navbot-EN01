/*
The SyncWritePos example passed the test in SCS15, 
and if testing other models of SCS series servos
please change the appropriate position, speed and delay parameters.
*/

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

#include <SCServo.h>

SCSCL sc;

byte ID[2];
u16 Position[2];
u16 Speed[2];

void setup()
{
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sc.pSerial = &Serial1;
  delay(1000);
  ID[0] = 1;
  ID[1] = 2;
}

void loop()
{
  Position[0] = 1000;
  Position[1] = 1000;
  Speed[0] = 1500;
  Speed[1] = 1500;
  sc.SyncWritePos(ID, 2, Position, 0, Speed);//Servo((ID1/ID2)) moves at max speed=1500, moves to position=1000.
  delay(754);//[(P1-P0)/V]*1000+100

  Position[0] = 20;
  Position[1] = 20;
  Speed[0] = 1500;
  Speed[1] = 1500;
  sc.SyncWritePos(ID, 2, Position, 0, Speed);//Servo((ID1/ID2)) moves at max speed=1500, moves to position=20.
  delay(754);//[(P1-P0)/V]*1000+100
}
