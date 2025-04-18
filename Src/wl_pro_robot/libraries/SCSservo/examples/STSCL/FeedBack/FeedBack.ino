/*
回读所有舵机反馈参数:位置、速度、负载、电压、温度、移动状态、电流；
FeedBack函数回读舵机参数于缓冲区，Readxxx(-1)函数返回缓冲区中相应的舵机状态；
函数Readxxx(ID)，ID=-1返回FeedBack缓冲区参数；ID>=0，通过读指令直接返回指定ID舵机状态,
无需调用FeedBack函数。
Read back all feedback parameters: position, speed, load, voltage, temperature, movement status;
The FeedBack function reads back the servo parameters in the buffer, and the Readxxx (-1) function returns the corresponding servo state in the buffer;
Function Readxxx (ID), ID=1 returns the FeedBack buffer parameter; ID > 0, and directly returns the specified ID rudder state by reading the instruction.
There is no need to call the FeedBack function.
*/

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19
#include <SCServo.h>

SMS_STS sms_sts;

void setup()
{
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  Serial.begin(115200);
  sms_sts.pSerial = &Serial1;
  delay(1000);
}

void loop()
{
  int Pos;
  int Speed;
  int Load;
  int Voltage;
  int Temper;
  int Move;
  int Current;
  if(sms_sts.FeedBack(1)!=-1){
    Pos = sms_sts.ReadPos(-1);
    Speed = sms_sts.ReadSpeed(-1);
    Load = sms_sts.ReadLoad(-1);
    Voltage = sms_sts.ReadVoltage(-1);
    Temper = sms_sts.ReadTemper(-1);
    Move = sms_sts.ReadMove(-1);
    Current = sms_sts.ReadCurrent(-1);
    Serial.print("Position:");
    Serial.println(Pos);
    Serial.print("Speed:");
    Serial.println(Speed);
    Serial.print("Load:");
    Serial.println(Load);
    Serial.print("Voltage:");
    Serial.println(Voltage);
    Serial.print("Temper:");
    Serial.println(Temper);
    Serial.print("Move:");
    Serial.println(Move);
    Serial.print("Current:");
    Serial.println(Current);
    delay(10);
  }else{
    Serial.println("FeedBack err");
    delay(500);
  }
  
  Pos = sms_sts.ReadPos(1);
  if(Pos!=-1){
    Serial.print("Servo position:");
    Serial.println(Pos, DEC);
    delay(10);
  }else{
    Serial.println("read position err");
    delay(500);
  }
  
  Voltage = sms_sts.ReadVoltage(1);
  if(Voltage!=-1){
	Serial.print("Servo Voltage:");
    Serial.println(Voltage, DEC);
    delay(10);
  }else{
    Serial.println("read Voltage err");
    delay(500);
  }
  
  Temper = sms_sts.ReadTemper(1);
  if(Temper!=-1){
    Serial.print("Servo temperature:");
    Serial.println(Temper, DEC);
    delay(10);
  }else{
    Serial.println("read temperature err");
    delay(500);    
  }

  Speed = sms_sts.ReadSpeed(1);
  if(Speed!=-1){
    Serial.print("Servo Speed:");
    Serial.println(Speed, DEC);
    delay(10);
  }else{
    Serial.println("read Speed err");
    delay(500);    
  }
  
  Load = sms_sts.ReadLoad(1);
  if(Load!=-1){
    Serial.print("Servo Load:");
    Serial.println(Load, DEC);
    delay(10);
  }else{
    Serial.println("read Load err");
    delay(500);    
  }
  
  Current = sms_sts.ReadCurrent(1);
  if(Current!=-1){
    Serial.print("Servo Current:");
    Serial.println(Current, DEC);
    delay(10);
  }else{
    Serial.println("read Current err");
    delay(500);    
  }

  Move = sms_sts.ReadMove(1);
  if(Move!=-1){
    Serial.print("Servo Move:");
    Serial.println(Move, DEC);
    delay(10);
  }else{
    Serial.println("read Move err");
    delay(500);    
  }
  Serial.println();
}
