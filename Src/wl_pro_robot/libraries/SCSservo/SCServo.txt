通信层:SCS
----------------------------
硬件接口层:SCSerial
----------------------------
应用层:SMS_STS SCSCL分别对应飞特三个系列舵机



SMS_STS sms_sts;//定义SMSBL/SMSCL/STSCL系列舵机
SCSCL sc;//定义SCSCL系列舵机



INST.h---指令定义头文件
SCS.h/SCS.cpp---通信层程序
SCSerial.h/SCSerial.cpp---硬件接口程序
SMS_STS.h/SMS_STS.cpp---SMSBL/SMSCL/STSCL应用层程序
SCSCL.h/SCSCL.cpp---SCSCL应用层程序
(内存表定义于应用层程序头文件SMS_STS.h\SCSCL.h中不同系列舵内存表定义存在差异)


SCS类<---SCSerial类<---SMS_STS类/SCSCL类

arduino使用Atmega2560测试