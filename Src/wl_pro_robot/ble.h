#pragma once

#define BLE_DATA_SIZE 80


typedef struct CmdManeuverTypDef{
  int8_t roll;
  int8_t height;
  uint8_t linear_H;
  uint8_t linear_L;
  int8_t angular;
  int8_t stable;
  int8_t mode;
  int8_t dir;
  int8_t joy_y;
  int8_t joy_x;
}CmdManeuverTypDef;

typedef struct CmdWifiTypDef{
  uint8_t header[2];
  uint8_t idle1[3];
  
}CmdWifiTypDef;


enum BLE_STATE
{
  BLE_OFF_LINE=0,
  BLE_STATE_IDLE,
  BLE_STATE_RECEIVE_OK,
  BLE_STATE_RECEIVE_WAIT,
  BLE_STATE_WAITING_PROCEDD,
};
enum BLE_CMD
{
  CMD_MANEUVER=0,
  CMD_WIFI=1,
};

typedef struct BleDataTypDef{
  uint8_t frame[20];
  uint8_t cmd;              //The processing command is consistent with the third byte in the received data
  uint8_t remaining_pack;   //The received data frames are still needed. After the reception is completed, data processing will be carried out
  uint8_t data[BLE_DATA_SIZE];         //Bluetooth cache data, including the merged ones, should be released after processing
  int16_t len;
  int16_t index;
  uint8_t state;
  uint8_t timeout;
}BleDataTypDef;


void ble_init() ;
//void ble_send_data(uint8_t* data, uint8_t len);
void ble_loop(void);
void ble_test(void);
extern BleDataTypDef ble_rx;







