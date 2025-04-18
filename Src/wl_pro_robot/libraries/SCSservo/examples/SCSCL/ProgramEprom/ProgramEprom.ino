/*
example for changing ID.
*/

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

#include <SCServo.h>

SCSCL sc;

int ID_ChangeFrom = 1;
int ID_Changeto   = 2;

void setup()
{
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sc.pSerial = &Serial1;
  delay(1000);

  sc.unLockEprom(ID_ChangeFrom);//unlock EPROM-SAFE
  sc.writeByte(ID_ChangeFrom, SCSCL_ID, ID_Changeto);//ID
  sc.LockEprom(ID_Changeto);// EPROM-SAFE locked
}

void loop()
{

}
