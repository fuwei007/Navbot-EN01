/*
Ping the servo to check if it is ready.
*/

#include <SCServo.h>

SMS_STS sms_sts;
// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

int TEST_ID = 3;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sms_sts.pSerial = &Serial1;
  delay(1000);
}

void loop()
{
  int ID = sms_sts.Ping(TEST_ID);
  if(ID!=-1){
    Serial.print("Servo ID:");
    Serial.println(ID, DEC);
    delay(100);
  }else{
    Serial.println("Ping servo ID error!");
    delay(2000);
  }
}