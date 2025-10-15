#pragma once

#include <WiFi.h>
#include <ArduinoJson.h>
#include "esp_adc_cal.h"
#include "feedback_util.h"
#include "eeprom_util.h"


#define CONFIG_JSON_SIZE 300
#define WIFI_INFO_JSON_SIZE 110

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
  int uncontrolable = 0;  //Too much tilt and loss of control
} MemotyTabTypeDef;


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
  int uncontrolable = 0;  //Too much tilt and loss of control
} Wrobot;


// Enumeration of robot motion states
typedef enum {
  FORWARD = 0,
  BACK,
  RIGHT,
  LEFT,
  STOP,
  JUMP,
} QR_State_t;


struct {

  const String SYS_WIFI = "sys_wifi";
  const String SYS_WEB_SOCKET_SERVER = "sys_web_socket_server";
  const String SYS_RESTART = "sys_restart";
  const String GET_DEVICE_INFO = "get_device_info";
  const String OFF_SERVO = "off_servo";
  const String ON_SERVO = "on_servo";
  const String CALIBRATION_SERVO = "calibrate_servo";
  const String SET_SERVO_ID1 = "set_servo_ID1";
  const String SET_SERVO_ID2 = "set_servo_ID2";

  const String SET_NAME = "set_name";
  const String SHOW_EXPRESSION = "show_expression";
  const String GET_EXPRESSION = "get_expression";
  const String SET_CLOUD_TOKEN = "set_cloud_token";
  const String SET_OPENAI_TOKEN = "set_openai_token";

  const String SET_TEST_NUMBER = "set_test_number";

} MESSAGE_TYPE;

struct {

  const String NAME = "name";
  const String WIFI_SSID = "wifi_ssid";
  const String WIFI_PASSWORD = "wifi_password";
  const String WIFI_STATE = "wifi_state";
  const String CLOUD_TOKEN = "cloude_token";
  const String OPENAI_TOKEN = "openai_token";
} CONFIG_KEY;

struct {

  const String SSID = "ssid";
  const String PASSWORD = "password";
  const String STATE = "state";
} WIFI_INFO_KEY;


// Robot mode enumeration type
typedef enum {
  BASIC = 0,
} Robot_Mode_t;

#define WIFI_CLOSE 0
#define WIFI_SERVOR 1
#define WIFI_CLIENT 2

class RobotProtocol {
public:
  double battery_voltage = 8.4f;
  double pcb_version;
  double fahrenheit;
  double centigrade;
  double battery_level;
  int status;
  int16_t offset_roll = 0;
  int uncontrolable = 0;   //Too much tilt and loss of control
  float uncontrolable_angle = 40.0f;

  bool charge = false;
  bool socket_connected = false;
  bool ble_connected = false;
  bool wifi_connected = false;
  char wifi_state      = WIFI_CLOSE;

  bool send_status_flag = false;

  String show_expression;
  int show_expression_time = -1;  //

  uint8_t test_number=0;

  StaticJsonDocument<WIFI_INFO_JSON_SIZE> wifi_info_json;
  StaticJsonDocument<CONFIG_JSON_SIZE> config_json;
  StaticJsonDocument<512> status_json;

  RobotProtocol(uint8_t len);
  ~RobotProtocol();

  void spinOnce(void);
  double get_pcb_version();
  double get_fahrenheit(void);
  double get_degree_centigrade(void);
  double get_battery_voltage(void);
  double get_battery_level(void);
  int get_robot_status(void);
  int get_expression_name(StaticJsonDocument<300> &doc);
  void get_dev_name(char dev_name[]);
  void build_dev_name(char dev_name[]);
  void config_json_init(void);
  void printDoc(StaticJsonDocument<300> &doc);
  void isSys(StaticJsonDocument<300> &doc);
  void parseBasic(StaticJsonDocument<300> &doc);
  void parseJson(StaticJsonDocument<300> &doc);
  void send_status(void);
  void send_heartbeat(void);
  void save_wifi_info_json(void);
  void json_test(char *json_arr);
  void set_test_number(StaticJsonDocument<300> &doc);
  void test_log_output(void);
private:
  uint8_t *_now_buf;
  uint8_t *_old_buf;
  uint8_t _len;
  void UART_WriteBuf(void);
  int checkBufRefresh(void);
  void save_config_json(void);
  void json_is_sys_wifi(StaticJsonDocument<300> &doc);
  void json_is_sys_web_socket_server(StaticJsonDocument<300> &doc);
  void json_is_sys_set_name(StaticJsonDocument<300> &doc);
  void json_is_sys_show_expression(StaticJsonDocument<300> &doc);
  void json_is_sys_send_expression(int send_channel);
  void json_is_sys_set_cloud_token(StaticJsonDocument<300> &doc);
  void json_is_sys_set_openai_token(StaticJsonDocument<300> &doc);
  void calibrate_servo(void);
};

extern int BAT_PIN;        // select the input pin for the ADC
extern esp_adc_cal_characteristics_t adc_chars;

extern Wrobot wrobot;
extern RobotProtocol rp;
