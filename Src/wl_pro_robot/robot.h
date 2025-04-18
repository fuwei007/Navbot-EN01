
#pragma once

#include <WiFi.h>
#include <ArduinoJson.h>

typedef struct {
  int height = 38;
  int roll;
  int linear;
  int angular;
  int dir;
  int dir_last;
  int joyy;
  int joyy_last;
  int joyx;
  int joyx_last;
  bool go;
}Wrobot;

extern Wrobot wrobot;

// Enumeration of robot motion states
typedef enum {
	FORWARD = 0, 	
	BACK,        	       		
	RIGHT,       	
	LEFT,        	
	STOP,
  JUMP,         		
} QR_State_t; 


	
// Robot mode enumeration type
typedef enum {
  BASIC = 0,
} Robot_Mode_t; 

class RobotProtocol
{
	public:
		RobotProtocol(uint8_t len);
		~RobotProtocol();
		void spinOnce(void);
		void parseBasic(StaticJsonDocument<300> &doc);
    private:
        uint8_t *_now_buf;
		uint8_t *_old_buf;
		uint8_t _len;
		void UART_WriteBuf(void);
		int checkBufRefresh(void);
};
