/*
The normal write example passed the test in ST3215 Servo, 
and if testing other models of ST series servos
please change the appropriate position, speed and delay parameters.
*/

#include <SCServo.h>

SMS_STS st;

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

void setup()
{
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  delay(1000);
}

void loop()
{
  st.WritePosEx(1, 4095, 3400, 50);//servo(ID1) speed=3400，acc=50，move to position=4095.
  delay(2000);
  
  st.WritePosEx(1, 2000, 1500, 50);//servo(ID1) speed=3400，acc=50，move to position=2000.
  delay(2000);
}
