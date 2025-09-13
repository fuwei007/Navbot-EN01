
#include "robot.h"
#include <ArduinoJson.h>
#include "wifi.h"
#include "eeprom_util.h"
#include "Servo_STS3032.h"
#include "SPIFFS.h"
#include <vector>
#include "cpu0_task.h"
#include "ble.h"
#include "web_socket_client_util.h"

int uncontrolable = 0;
int BAT_PIN = 35;
esp_adc_cal_characteristics_t adc_chars;
Wrobot wrobot;
RobotProtocol rp(20);

RobotProtocol::RobotProtocol(uint8_t len) {
  _len = len;
  _now_buf = new uint8_t[_len];
  _old_buf = new uint8_t[_len];

  for (int i = 0; i < _len; i++) {
    _now_buf[i] = 0;
  }

  _now_buf[0] = 0xAA;
  _now_buf[1] = 0x55;
}

RobotProtocol::~RobotProtocol() {
  delete[] _now_buf;
  delete[] _old_buf;
}

void RobotProtocol::spinOnce(void) {
  int flag = checkBufRefresh();
  if (flag) {
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

void RobotProtocol::UART_WriteBuf(void) {
  for (int i = 0; i < _len; i++) {
    Serial.write(_now_buf[i]);
  }
}

int RobotProtocol::checkBufRefresh(void) {
  int ret = 0;
  for (int i = 0; i < _len; i++) {
    if (_now_buf[i] != _old_buf[i]) {
      ret = 1;
      break;
    } else {
      ret = 0;
    }
  }

  for (int i = 0; i < _len; i++) {
    _old_buf[i] = _now_buf[i];
  }
  return ret;
}
// This undocumented internal function must be declared
extern "C" {
  uint8_t temprature_sens_read();
}

double RobotProtocol::get_pcb_version() {
  int sensorValue = analogRead(A0);
  pcb_version = sensorValue * (3.30 / 4096) + 1;
  Serial.print("pcb_version:");
  Serial.println(pcb_version);
  return pcb_version;
}

double RobotProtocol::get_fahrenheit() {
  return temprature_sens_read();
}

double RobotProtocol::get_degree_centigrade() {
  double fahrenheit = get_fahrenheit();
  // Convert to Celsius (unit: Fahrenheit)
  return (fahrenheit - 32) / 1.8;
}

double RobotProtocol::get_battery_voltage() {
  return battery_voltage;
}

double RobotProtocol::get_battery_level() {
  double currentVoltage = battery_voltage;
  double maxVoltage = 8.3;
  double minVoltage = 7.0;

  // Ensure the voltage is within a reasonable range

  if (currentVoltage >= maxVoltage) return 100.0;
  if (currentVoltage <= minVoltage) return 0.0;

  // Calculate the percentage
  double battery_percentage = (currentVoltage - minVoltage) / (maxVoltage - minVoltage) * 100.0;

  battery_percentage = round(battery_percentage) / 1;  // Format to retain two significant digits (rounded to 2 decimal places)

  // Battery logic correction (prevent out-of-bounds and display of 0 data)
  if (battery_percentage > 100) {
    battery_percentage = 100;
  } else if (battery_percentage < 1) {
    battery_percentage = 1;
  }
  battery_level = battery_percentage;
  return battery_percentage;
}

int RobotProtocol::get_robot_status() {
  // 0 - Normal state, 1 - Primary out-of-control state (tilting), 2 - Recovery waiting state
  int robot_status = -1;
  if (uncontrolable == 0) {
    robot_status = 0;
  } else if (uncontrolable == 1) {
    robot_status = 1;
  } else if (uncontrolable > 1) {
    robot_status = 2;
  }
  return robot_status;
}


void RobotProtocol::printDoc(StaticJsonDocument<300> &doc) {
  char receive_command[300];
  serializeJson(doc, receive_command);
  Serial.printf("receive_command:%s \r\n", receive_command);
}

bool create_and_write_file(const char *filePath, const char *content) {
  // Open the file in write mode (if the file does not exist, it will be created)
  File file = SPIFFS.open(filePath, FILE_WRITE);
  // Write the content to the file
  size_t bytesWritten = file.print(content);
  file.close();
  if (bytesWritten > 0) {
    Serial.printf("Successfully wrote %d bytes to the file\n", bytesWritten);
    return true;
  } else {
    Serial.println("An error occurred while writing to the file.");
    return false;
  }
}
void RobotProtocol::save_config_json(void) {

  char json_arr[CONFIG_JSON_SIZE + 1] = { 0 };
  serializeJson(rp.config_json, json_arr);

  //seve data
  eeprom_util.write(&EepromParam.ADDR_JSON, json_arr);

  //read data
  Serial.print("read config json :");Serial.println(eeprom_util.read(&EepromParam.ADDR_JSON));
}
void RobotProtocol::save_wifi_info_json(void) {
  
  char json_arr[WIFI_INFO_JSON_SIZE + 1] = { 0 };
  serializeJson(rp.wifi_info_json, json_arr);

  //seve data
  eeprom_util.write(&EepromParam.ADDR_WIFI_INFO_JSON, json_arr);

  //read data
  Serial.print("read wifi info json :");Serial.println(eeprom_util.read(&EepromParam.ADDR_WIFI_INFO_JSON));

}
void RobotProtocol::json_is_sys_wifi(StaticJsonDocument<300> &doc) {

  wifi_info_json[WIFI_INFO_KEY.STATE]     = doc[WIFI_INFO_KEY.STATE];
  if (doc[WIFI_INFO_KEY.STATE] == WIFI_STATE.CLIENT)
  {
    wifi_info_json[WIFI_INFO_KEY.SSID]      = doc[WIFI_INFO_KEY.SSID];
    wifi_info_json[WIFI_INFO_KEY.PASSWORD]  = doc[WIFI_INFO_KEY.PASSWORD];
  }

  // save data
  String jsonStr;
  serializeJson(wifi_info_json, jsonStr);
  eeprom_util.write(&EepromParam.ADDR_WIFI_INFO_JSON, jsonStr);

  // read data
  Serial.print("READ SAVE WEB SOCKET HOST:");
  Serial.println(eeprom_util.read(&EepromParam.ADDR_WIFI_INFO_JSON));
  wifi_restart();

}
void RobotProtocol::json_is_sys_web_socket_server(StaticJsonDocument<300> &doc) {
  String host = doc["host"];
  uint16_t port = doc["port"];
  String path = doc["path"];

  // save data
  eeprom_util.write(&EepromParam.ADDR_WEB_SOCKET_HOST, host);
  eeprom_util.writeUint16T(&EepromParam.ADDR_WEB_SOCKET_PORT, port);
  eeprom_util.write(&EepromParam.ADDR_WEB_SOCKET_PATH, path);

  // read data
  Serial.print("READ SAVE WEB SOCKET HOST:");
  Serial.println(eeprom_util.read(&EepromParam.ADDR_WEB_SOCKET_HOST));
  Serial.print("READ SAVE WEB SOCKET PORT:");
  Serial.println(eeprom_util.readToUint16T(&EepromParam.ADDR_WEB_SOCKET_PORT));
  Serial.print("READ SAVE WEB SOCKET PATH:");
  Serial.println(eeprom_util.read(&EepromParam.ADDR_WEB_SOCKET_PATH));
}
void RobotProtocol::json_is_sys_set_name(StaticJsonDocument<300> &doc) {
  if (doc["name"].isNull() == true) {
    Serial.println("Invalid json data.");
  } else {
    String name_str = doc["name"];
    if (name_str.length() > 20) {
      Serial.println("The name is too long. Please limit it to 20 characters or less.");
      return;
    }
    config_json[CONFIG_KEY.NAME] = doc["name"];
    save_config_json();
  }
}

// Interrupt function - This function will be called when the time is up.
void ARDUINO_ISR_ATTR show_expression_time_callback()
{
  rp.show_expression_time = -1;
}
void RobotProtocol::json_is_sys_show_expression(StaticJsonDocument<300> &doc) {
  if (doc["file"].isNull() == true) {
    Serial.println("Invalid json data.");
  } else {
    String file_str = doc["file"].as<const char *>();
    show_expression = "/" + file_str;
    if (doc["time"].isNull() == true) {
      show_expression_time = 0;
    } else {
      int temp = doc["time"];
      show_expression_time = temp < 0 ? 0 : temp * 1;
      if(show_expression_time>0)
      {
        // Use 1st timer of 4 (counted from zero).
        // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
        // info).
        hw_timer_t * timer = timerBegin(0, 80, true);
        // Attach onTimer function to our timer.
        timerAttachInterrupt(timer, &show_expression_time_callback, true);
        // Set alarm to call onTimer function every second (value in microseconds).
        // Repeat the alarm (third parameter)
        timerAlarmWrite(timer, show_expression_time*1000000, false);
        // Start an alarm
        timerAlarmEnable(timer);
      }
    }
    Serial.print("assign:");
    Serial.println(show_expression);
    Serial.print("time: ");
    Serial.print(show_expression_time);
    Serial.println("s");
  }
}

std::vector<String> getBinFiles() {
  std::vector<String> binFiles;

  if (!SPIFFS.begin(true)) {
    Serial.println("--error!! SPIFFS mounting failed");
    return binFiles;
  }

  File root = SPIFFS.open("/");
  if (!root) {
    Serial.println("--error!! Cannot open the root directory");
    return binFiles;
  }

  if (!root.isDirectory()) {
    Serial.println("--error!! The root directory is not a folder.");
    return binFiles;
  }

  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      String fileName = String(file.name());
      // Check if the file extension is .bin.
      if (fileName.endsWith(".bin") || fileName.endsWith(".BIN")) {
        //Remove the leading slash (if any)
        if (fileName.startsWith("/")) {
          fileName = fileName.substring(1);
        }
        binFiles.push_back(fileName);
      }
    }
    file = root.openNextFile();
  }
  root.close();
  return binFiles;
}
int RobotProtocol::get_expression_name(StaticJsonDocument<300> &doc) {
  int count = 0;
  JsonArray array = doc.createNestedArray("expression");
  std::vector<String> binFiles = getBinFiles();
  for (const auto &str : binFiles) {
    array.add(str);
    count++;
  }
  return count;
}
void RobotProtocol::json_is_sys_send_expression(int send_channel = FEEDBACK_CHANNEL.BLE) {
  // Serialize to string
  String expression_str;
  // Create static JSON document (allocate memory pool size)
  StaticJsonDocument<300> doc;
  doc["type"] = "get_expression";
  get_expression_name(doc);
  serializeJson(doc, expression_str);

  Serial.print("expression file:");
  Serial.println(expression_str);

  if (FEEDBACK_CHANNEL.ALL == send_channel) {
    web_sockets_client_send_message(expression_str);
    ble_tx_add_string(expression_str);
  } else if (FEEDBACK_CHANNEL.BLE == send_channel) {
    ble_tx_add_string(expression_str);
  } else if (FEEDBACK_CHANNEL.WEB_SOCKET_CLIENT == send_channel) {
    web_sockets_client_send_message(expression_str);
  }
}
void RobotProtocol::json_is_sys_set_cloud_token(StaticJsonDocument<300> &doc) {
  if (doc["token"].isNull() == true) {
    Serial.println("Invalid json data.");
  } else {
    config_json[CONFIG_KEY.CLOUD_TOKEN] = doc["token"];
    save_config_json();
    wss_reset();
  }
}
void RobotProtocol::json_is_sys_set_openai_token(StaticJsonDocument<300> &doc) {
  if (doc["token"].isNull() == true) {
    Serial.println("Invalid json data.");
  } else {
    config_json[CONFIG_KEY.OPENAI_TOKEN] = doc["token"];
    save_config_json();
  }
}
void set_setvo_id1()
{
  sms_sts.set_servo_eeprom_lock(2,0);
  delay(2);
  sms_sts.set_servo_id(2,1);
  // delay(2);
  // sms_sts.set_servo_eeprom_lock(1,1);
}
void set_setvo_id2()
{
  sms_sts.set_servo_eeprom_lock(1,0);
  delay(2);
  sms_sts.set_servo_id(1,2);
  // delay(2);
  // sms_sts.set_servo_eeprom_lock(2,1);
}
void RobotProtocol::calibrate_servo(void)
{
  sms_sts.calibrate_all_servo();
  delay(2);
  sms_sts.on_all_servo();
}
void RobotProtocol::isSys(StaticJsonDocument<300> &doc) {
  String type = doc["type"];
  Serial.print("type:");
  Serial.println(type);
  yield();
  if (type == MESSAGE_TYPE.SYS_WIFI) {
    json_is_sys_wifi(doc);
  } else if (type == MESSAGE_TYPE.SYS_WEB_SOCKET_SERVER) {
    json_is_sys_web_socket_server(doc);
  } else if (type == MESSAGE_TYPE.SYS_RESTART) {
    ESP.restart();
  } else if (type == MESSAGE_TYPE.GET_DEVICE_INFO) {
    feedback_util_send_message(FEEDBACK_CHANNEL.ALL);
  } else if (type == MESSAGE_TYPE.OFF_SERVO) {
    sms_sts.off_all_servo();
  } else if (type == MESSAGE_TYPE.ON_SERVO) {
    sms_sts.on_all_servo();
  } else if (type == MESSAGE_TYPE.CALIBRATION_SERVO) {
    calibrate_servo();
  } else if (type == MESSAGE_TYPE.SET_SERVO_ID1) {
    set_setvo_id1();
  } else if (type == MESSAGE_TYPE.SET_SERVO_ID2) {
    set_setvo_id2();
  } else if (type == MESSAGE_TYPE.SET_NAME) {
    json_is_sys_set_name(doc);
  } else if (type == MESSAGE_TYPE.SHOW_EXPRESSION) {
    json_is_sys_show_expression(doc);
  } else if (type == MESSAGE_TYPE.GET_EXPRESSION) {
    json_is_sys_send_expression(FEEDBACK_CHANNEL.ALL);
  } else if (type == MESSAGE_TYPE.SET_CLOUD_TOKEN) {
    json_is_sys_set_cloud_token(doc);
  } else if (type == MESSAGE_TYPE.SET_OPENAI_TOKEN) {
    json_is_sys_set_openai_token(doc);
  } else {
    Serial.println("Invalid json keyworeds.\r\n");
  }
}
void RobotProtocol::parseJson(StaticJsonDocument<300> &doc) {
  if (doc["type"].isNull() == true) {
    Serial.println("JSON type is null ");
    yield();
    parseBasic(doc);
  } else {
    Serial.println("JSON type is sys");
    isSys(doc);
  }
}
void RobotProtocol::get_dev_name(char dev_name[]) {
  String name_str = config_json["name"];
  unsigned int name_len = name_str.length() + 1;
  name_len = name_len > 20 ? 20 : name_len;
  name_str.toCharArray(dev_name, name_len);
  Serial.print("BLE name :");
  Serial.println(dev_name);
  dev_name[19] = 0;
}
void RobotProtocol::build_dev_name(char dev_name[]) {
  char basc[] = "navbot_en01-";
  uint8_t mac[7];
  esp_read_mac(mac, ESP_MAC_BT);
  mac[6] = 0;
  char i;

  for (i = 0; i < 6; i++)  //Convert the mac address to contain only 0-9/a-z
  {
    mac[i] = mac[i] % 36;  // 10+26=36

    if (mac[i] <= 9) mac[i] = mac[i] + '0';             //0-9
    else if (mac[i] <= 35) mac[i] = mac[i] - 10 + 'a';  //a-z
  }
  sprintf(dev_name, "%s%s", basc, mac);
}
void RobotProtocol::config_json_init(void) {
  eeprom_util.init();
  String json_str = eeprom_util.read(&EepromParam.ADDR_JSON);
  DeserializationError error = deserializeJson(config_json, json_str);
  if (error) {
    Serial.println("The file was not found 'config.json'.");
    char name[20];
    build_dev_name(name);
    String name_str = name;
    config_json["name"] = name_str;
    save_config_json();
  }
  printDoc(config_json);
}
/*
  When the status changes, it actively sends the status synchronization to the upper computer via Bluetooth.
This function is called once every 10 milliseconds.
*/
void RobotProtocol::send_status(void)
{
  if(send_status_flag == true && ble_connected == true)
  {
    send_status_flag = false;
    // ble_tx_add_json(status_json);
  }
}
/*
Fixed heartbeat packet
*/
void RobotProtocol::send_heartbeat(void)
{

}
void RobotProtocol::json_test(char *json_arr) {
  Serial.print("json test: ");
  Serial.println(json_arr);
  String payload_str = String(json_arr);
  StaticJsonDocument<300> doc;
  DeserializationError error = deserializeJson(doc, payload_str);
  if (error) {
    Serial.println("json data error");
  } else {
    parseJson(doc);
  }
}














































void RobotProtocol::parseBasic(StaticJsonDocument<300> &doc) {
  // printDoc(doc);
  // isSys(doc);

  delay(0);
  _now_buf[2] = BASIC;

  String dir = doc["dir"];
  if (dir == "stop") {
    _now_buf[3] = STOP;
    wrobot.dir = STOP;
  } else {
    if (dir == "jump") {
      _now_buf[3] = JUMP;
      wrobot.dir = JUMP;
    } else if (dir == "forward") {
      _now_buf[3] = FORWARD;
      wrobot.dir = FORWARD;
    } else if (dir == "back") {
      _now_buf[3] = BACK;
      wrobot.dir = BACK;
    } else if (dir == "left") {
      _now_buf[3] = LEFT;
      wrobot.dir = LEFT;
    } else if (dir == "right") {
      _now_buf[3] = RIGHT;
      wrobot.dir = RIGHT;
    } else {
      _now_buf[3] = STOP;
      wrobot.dir = STOP;
    }
  }

  int height = doc["height"];
  _now_buf[4] = height;
  wrobot.height = height;

  int roll = doc["roll"];
  rp.offset_roll = roll;

  if (roll >= 0) {
    _now_buf[5] = 0;
  } else {
    _now_buf[5] = 1;
  }
  _now_buf[6] = abs(roll);

  int linear = doc["linear"];
  if(linear < 0){
    linear = 0;
  }
  wrobot.linear = linear;
  if (linear >= 0) {
    _now_buf[7] = 0;
  } else {
    _now_buf[7] = 1;
  }
  _now_buf[8] = abs(linear);

  int angular = doc["angular"];
  if(angular < 0){
    angular = 0;
  }
  wrobot.angular = angular;
  if (angular >= 0) {
    _now_buf[9] = 0;
  } else {
    _now_buf[9] = 1;
  }
  _now_buf[10] = abs(angular);


  int stable = doc["stable"];
  wrobot.go = stable;
  if (stable) {
    _now_buf[11] = 1;
  } else {
    _now_buf[11] = 0;
  }

  int joy_x = doc["joy_x"];
  wrobot.joyx = joy_x;// * angular / 100;
  if (joy_x >= 0) {
    _now_buf[12] = 0;
  } else {
    _now_buf[12] = 1;
  }
  _now_buf[13] = abs(joy_x);

  int joy_y = doc["joy_y"];
  wrobot.joyy = joy_y;// * linear / 100;
  if (joy_y >= 0) {
    _now_buf[14] = 0;
  } else {
    _now_buf[14] = 1;
  }
  _now_buf[15] = abs(joy_y);
}
