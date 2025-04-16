void onReceiveFunction(void) {
  while (Serial.available() > 0) {
    char data = Serial.read();
    Serial2.write(data);  // 将数据发送到舵机
  }
}
void onReceiveFunction2(void) {
  while (Serial2.available() > 0) {
    char data = Serial2.read();
    Serial.write(data);  // 将数据发送到串口
  }
}
//Serial、Serial1、Serial2 分别对应了 UART0、UART1 和 UART2。
void setup() {
  //串口0初始化
  Serial.begin(1000000);                  //usb串口 
  Serial2.begin(1000000);                 //舵机串口
  Serial.onReceive(onReceiveFunction);    //为串口设置回调函数
  Serial2.onReceive(onReceiveFunction2);  //为串口设置回调函数
}
void loop() {
  // 从串口读取数据
   if (Serial.available())
   {
     char data = Serial.read();
  //   // 将数据发送到 舵机
     Serial2.write(data);
   }
  // // 从舵机读取输入数据
   if (Serial2.available())
   {
     char data = Serial2.read();
  //   // 将数据发送到 UART0
     Serial.write(data);
   }
}
