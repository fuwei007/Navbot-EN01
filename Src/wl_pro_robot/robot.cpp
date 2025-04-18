
#include "robot.h"

Wrobot wrobot;
RobotProtocol::RobotProtocol(uint8_t len)
{
    _len = len;
    _now_buf = new uint8_t[_len];
    _old_buf = new uint8_t[_len];

    for(int i=0;i<_len;i++) {
        _now_buf[i] = 0;
    }

    _now_buf[0] = 0xAA;
    _now_buf[1] = 0x55;
}

RobotProtocol::~RobotProtocol()
{
    delete [] _now_buf;
    delete [] _old_buf;
}

void RobotProtocol::spinOnce(void)
{
    int flag = checkBufRefresh();
    if(flag) 
    {
        //UART_WriteBuf(); //This will convert the control information on the web end into a serial port protocol for transmission

        //Serial.println(wrobot.dir);//Test data acquisition
        //Serial.println("date have send\n");
        //Serial.printf("height:%d\n", wrobot.height);
        //Serial.printf("roll:%d\n", wrobot.pitch);
        //Serial.printf("linear:%d\n", wrobot.linear);
//        Serial.printf("\n");
//        Serial.printf("joy_X:%d\n", wrobot.joyx);
//        Serial.printf("joy_Y:%d\n", wrobot.joyy);
    }
}









/**************The following is simultaneously used for serial port output control protocols***************/

void RobotProtocol::UART_WriteBuf(void)
{
    for(int i=0; i<_len; i++) {
        Serial.write(_now_buf[i]);
    }
}

int RobotProtocol::checkBufRefresh(void)
{
    int ret = 0;
    for(int i=0; i<_len; i++)
    {
        if(_now_buf[i] != _old_buf[i]) {
            ret = 1;
            break;
        }else {
            ret = 0;
        }
    }

    for(int i=0; i<_len; i++) {
        _old_buf[i]= _now_buf[i];
    }
    return ret;
}

void RobotProtocol::parseBasic(StaticJsonDocument<300> &doc)
{
 
    _now_buf[2] = BASIC;

    String dir = doc["dir"];
    if(dir == "stop") {
        _now_buf[3] = STOP;
        wrobot.dir = STOP;  
    } else {
        if(dir == "jump") {      	
            _now_buf[3] = JUMP;
            wrobot.dir = JUMP;
        }else if(dir == "forward") { 
            _now_buf[3] = FORWARD; 
            wrobot.dir = FORWARD;
        }else if(dir == "back") { 	
            _now_buf[3] = BACK; 
            wrobot.dir = BACK;
        }else if(dir == "left") { 	
            _now_buf[3] = LEFT;
            wrobot.dir = LEFT;
        }else if(dir == "right") { 
            _now_buf[3] = RIGHT;
            wrobot.dir = RIGHT;
        }else{
            _now_buf[3] = STOP;
            wrobot.dir = STOP;
        }
    }

    int height = doc["height"];
    _now_buf[4] = height;
    wrobot.height = height;

    int roll = doc["roll"];
    wrobot.roll = roll;
    if(roll >= 0) {
        _now_buf[5] = 0;
    }else {
        _now_buf[5] = 1;
    }
    _now_buf[6] = abs(roll);    
    
    int linear = doc["linear"];
    wrobot.linear = linear;
    if(linear >= 0) {
        _now_buf[7] = 0;
    }else {
        _now_buf[7] = 1;
    }
    _now_buf[8] = abs(linear); 
    
    int angular = doc["angular"];
    wrobot.angular = angular;
    if(angular >= 0) {
        _now_buf[9] = 0;
    }else {
        _now_buf[9] = 1;
    }
    _now_buf[10] = abs(angular); 
   

    int stable = doc["stable"];
    wrobot.go = stable;
    if(stable) {
        _now_buf[11] = 1;
    }else {
        _now_buf[11] = 0;
    }
    
    int joy_x = doc["joy_x"];
    wrobot.joyx = joy_x;
    if(joy_x >= 0) {
        _now_buf[12] = 0;
    }else {
        _now_buf[12] = 1;
    }
    _now_buf[13] = abs(joy_x); 


    int joy_y = doc["joy_y"];
    wrobot.joyy = joy_y;
    if(joy_y >= 0) {
        _now_buf[14] = 0;
    }else {
        _now_buf[14] = 1;
    }
    _now_buf[15] = abs(joy_y); 
}
