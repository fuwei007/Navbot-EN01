/*
example for changing ID.
*/

#include <SCServo.h>

SMS_STS sms_sts;
// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

int ID_ChangeFrom = 1;
int ID_Changeto   = 2;

void setup()
{
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sms_sts.pSerial = &Serial1;
  delay(1000);
  sms_sts.unLockEprom(1);//unlock EPROM-SAFE
  sms_sts.writeByte(ID_ChangeFrom, SMS_STS_ID, ID_Changeto);//ID
  sms_sts.LockEprom(ID_Changeto);//EPROM-SAFE locked
}

void loop()
{

}
