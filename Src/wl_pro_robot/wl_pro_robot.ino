//Robot controlled header file
#include <MPU6050_tockn.h>
#include "Servo_STS3032.h"
#include <SimpleFOC.h>
#include <Arduino.h>

//WiFi transmission header file
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WiFi.h>
#include <FS.h>
#include "basic_web.h"
#include "robot.h"
#include "wifi.h"
#include "esp_adc_cal.h"

/************Instance definition*************/

//Electromotor instance
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);
BLDCDriver3PWM driver2  = BLDCDriver3PWM(26,27,14,12);

//Encoder instance
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

//PID instance
PIDController pid_angle     {.P = 1,    .I = 0,   .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_gyro      {.P = 0.06, .I = 0,   .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_distance  {.P = 0.5,  .I = 0,   .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_speed     {.P = 0.7,  .I = 0,   .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_yaw_angle {.P = 1.0,  .I = 0,   .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_yaw_gyro  {.P = 0.04, .I = 0,   .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_lqr_u     {.P = 1,    .I = 15,  .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_zeropoint {.P = 0.002,.I = 0,   .D = 0, .ramp = 100000, .limit = 4};
PIDController pid_roll_angle{.P = 8,    .I = 0,   .D = 0, .ramp = 100000, .limit = 450};

//Low pass filter instance
LowPassFilter lpf_joyy{.Tf = 0.2};
LowPassFilter lpf_zeropoint{.Tf = 0.1};
LowPassFilter lpf_roll{.Tf = 0.3};

// Commander communicate instance
Commander command = Commander(Serial);

void StabAngle(char* cmd)     { command.pid(&pid_angle, cmd);     }
void StabGyro(char* cmd)      { command.pid(&pid_gyro, cmd);      }
void StabDistance(char* cmd)  { command.pid(&pid_distance, cmd);  }
void StabSpeed(char* cmd)     { command.pid(&pid_speed, cmd);     }
void StabYawAngle(char* cmd)  { command.pid(&pid_yaw_angle, cmd); }
void StabYawGyro(char* cmd)   { command.pid(&pid_yaw_gyro, cmd);  }
void lpfJoyy(char* cmd)       { command.lpf(&lpf_joyy, cmd);      }
void StabLqrU(char* cmd)      { command.pid(&pid_lqr_u, cmd);     }
void StabZeropoint(char* cmd) { command.pid(&pid_zeropoint, cmd); }
void lpfZeropoint(char* cmd)  { command.lpf(&lpf_zeropoint, cmd); }
void StabRollAngle(char* cmd) { command.pid(&pid_roll_angle, cmd);}
void lpfRoll(char* cmd)       { command.lpf(&lpf_roll, cmd);      }

//void Stabtest_zeropoint(char* cmd) { command.pid(&test_zeropoint, cmd); }

//WebServer instance
WebServer webserver; // server
WebSocketsServer websocket = WebSocketsServer(81); // Define a webSocket server to process messages sent by clients
RobotProtocol rp(20);
int joystick_value[2];

//STS steering engine instance
SMS_STS sms_sts;

//MPU6050 instance
MPU6050 mpu6050(I2Ctwo);

/************parameter definition*************/
#define pi 3.1415927

//LQR Self-balancing controller parameters
float LQR_angle = 0;
float LQR_gyro  = 0;
float LQR_speed = 0;
float LQR_distance = 0;
float angle_control   = 0;
float gyro_control    = 0;
float speed_control   = 0;
float distance_control = 0;
float LQR_u = 0;
float angle_zeropoint = 4.02;
float distance_zeropoint = -256.0;//Wheel position shift zero offset \
 (-256 is an impossible displacement value, use it as a sign that it is not refreshed)


//YAW axis control data
float YAW_gyro = 0;
float YAW_angle = 0;
float YAW_angle_last = 0;
float YAW_angle_total = 0;
float YAW_angle_zero_point = -10;
float YAW_output = 0;

//Leg steering gear control data
byte ID[2];
s16 Position[2];
u16 Speed[2];
byte ACC[2];

//Logical processing flag bit
float robot_speed = 0;          //Record the current wheel speed
float robot_speed_last = 0;     //Record the wheel speed at the previous time
int wrobot_move_stop_flag = 0;  //Record the sign that the rocker stops
int jump_flag = 0;              //Jump period identification
float leg_position_add = 0;     //roll axis balance control quantity
int uncontrolable = 0;          //Too much tilt and loss of control

//Voltage detection
uint16_t bat_check_num = 0;
int BAT_PIN = 35;    // select the input pin for the ADC
static esp_adc_cal_characteristics_t adc_chars;
static const adc1_channel_t channel = ADC1_CHANNEL_7;     
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

//Power display LED Pin
#define LED_BAT 13

void setup() {
  Serial.begin(115200);
  Serial2.begin(1000000);
  WiFi_SetAP();
  // set_sta_wifi();      // ESP-01S STA mode
  webserver.begin();
  webserver.on("/", HTTP_GET, basicWebCallback);
  websocket.begin();
  websocket.onEvent(webSocketEventCallback);

  //Steering gear initialization
  //Steering gear effective stroke 450
  //Left-hand steering gear[2048+12+50,2048+12+450]
  //Right-hand steering gear[2048-12-50,2048-12-450]
  sms_sts.pSerial = &Serial2;
  ID[0] = 1;
  ID[1] = 2;
  ACC[0] = 30;
  ACC[1] = 30;
  Speed[0] = 300;
  Speed[1] = 300;  
  Position[0] = 2148;
  Position[1] = 1948;
  //The steering gear (ID1/ID2) runs to their respective positions at maximum speed V=2400 steps/SEC and \
  acceleration A=50(50*100 steps/SEC ^2)
  sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

  //Voltage detection
  adc_calibration_init();
  adc1_config_width(width);
  adc1_config_channel_atten(channel, atten);
  esp_adc_cal_characterize(unit, atten, width, 0, &adc_chars);

  //Voltage detection LED
  pinMode(LED_BAT,OUTPUT);
  
  // Encoder setup
  I2Cone.begin(19,18, 400000UL); 
  I2Ctwo.begin(23,5, 400000UL); 
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);

  //mpu6050 setup
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  //Connect the motor object to the encoder object
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  //Speed loop PID parameter
  motor1.PID_velocity.P = 0.05;
  motor1.PID_velocity.I = 1;
  motor1.PID_velocity.D = 0;

  motor2.PID_velocity.P = 0.05;
  motor2.PID_velocity.I = 1;
  motor2.PID_velocity.D = 0;

  // Motor driver setup
  motor1.voltage_sensor_align = 6;
  motor2.voltage_sensor_align = 6;
  driver1.voltage_power_supply = 8;
  driver2.voltage_power_supply = 8;
  driver1.init();
  driver2.init();

  //Connect the motor object to the drive object
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  motor1.torque_controller = TorqueControlType::voltage;
  motor2.torque_controller = TorqueControlType::voltage;   
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;
  
  // Motor-related Settings
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);
  //init
  motor1.init();
  motor1.initFOC(); 
  motor2.init();
  motor2.initFOC();

  // Map motor to commander
    command.add('A', StabAngle, "pid angle");
    command.add('B', StabGyro, "pid gyro");
    command.add('C', StabDistance, "pid distance");
    command.add('D', StabSpeed, "pid speed");
    command.add('E', StabYawAngle, "pid yaw angle");
    command.add('F', StabYawGyro, "pid yaw gyro");
    command.add('G', lpfJoyy, "lpf joyy");
    command.add('H', StabLqrU, "pid lqr u");
    command.add('I', StabZeropoint, "pid zeropoint");
    command.add('J', lpfZeropoint, "lpf zeropoint");
    command.add('K', StabRollAngle, "pid roll angle");
    command.add('L', lpfRoll, "lpf roll");

    //command.add('M', Stabtest_zeropoint, "test_zeropoint");



  delay(500);
}

void loop() {
  bat_check();        //Voltage detection
  web_loop();         //Web data update
  mpu6050.update();   //IMU data update
  lqr_balance_loop(); //lqr self-balancing control
  yaw_loop();         //yaw axis steering control
  leg_loop();         //Leg motion control
  
  //The self-balancing calculated output torque is assigned to the motor
  motor1.target = (-0.5)*(LQR_u + YAW_output);
  motor2.target = (-0.5)*(LQR_u - YAW_output);

  //Shut down output after falling out of control
  if( abs(LQR_angle) > 25.0f  )
  {
    uncontrolable = 1;
  }
  if( uncontrolable != 0 )//Delay recovery after lifting
  {
    if( abs(LQR_angle) < 10.0f  )
    {
      uncontrolable++;
    }
    if( uncontrolable > 200 )//The delay time of 200 program cycles
    {
      uncontrolable = 0;
    }
  }
  
  //Turn off output (remote control stop or Angle is too large out of control)
  if(wrobot.go==0 || uncontrolable!=0)
  {
    motor1.target = 0;
    motor2.target = 0;
    leg_position_add = 0;
  }
  
  //Record the last remote control data
  wrobot.dir_last  = wrobot.dir;
  wrobot.joyx_last = wrobot.joyx;
  wrobot.joyy_last = wrobot.joyy;
  
  //The FOC phase voltage is computed iteratively
  motor1.loopFOC();
  motor2.loopFOC();
  
  //Set the wheel motor output
  motor1.move();
  motor2.move();
  
  command.run();
  }

//lqr self-balancing control
void lqr_balance_loop(){
  /*
  * LQR balance formula, in order to facilitate the adjustment of parameters in actual use, 
  * the formula is decomposed into 4 P controls, 
  * using the PIDController method in commander real-time debugging
  */
  //QR_u = LQR_k1*(LQR_angle - angle_zeropoint) + LQR_k2*LQR_gyro + LQR_k3*(LQR_distance - distance_zeropoint) + LQR_k4*LQR_speed;

  //The negative value is given because, according to the current motor wiring, the positive torque will turn backwards
  LQR_distance  = (-0.5) *(motor1.shaft_angle + motor2.shaft_angle);
  LQR_speed     = (-0.5) *(motor1.shaft_velocity + motor2.shaft_velocity);
  LQR_angle = (float)mpu6050.getAngleY();
  LQR_gyro  = (float)mpu6050.getGyroY(); 
  //Serial.println(LQR_distance); 
  // Serial.println(LQR_angle);
  // Serial.print("-->");
  // Serial.println(angle_control);
  //Calculate self-balancing output
  angle_control     = pid_angle(LQR_angle - angle_zeropoint);
  gyro_control      = pid_gyro(LQR_gyro);

  //Motion detail optimization processing
  if(wrobot.joyy != 0)//Handling when there are forward and backward direction motion instructions
  {
    distance_zeropoint = LQR_distance;//Displacement zero reset
    pid_lqr_u.error_prev = 0;         //The output integral is cleared to zero
  }

  if( (wrobot.joyx_last!=0 && wrobot.joyx==0) || \
      (wrobot.joyy_last!=0 && wrobot.joyy==0) )//Stop in place processing when motion instruction returns to zero
  {
    wrobot_move_stop_flag = 1;
  }
  if( (wrobot_move_stop_flag==1) && (abs(LQR_speed)<0.5) )
  {
    distance_zeropoint = LQR_distance;//Displacement zero reset
    wrobot_move_stop_flag = 0;
  }

  if( abs(LQR_speed)>15 )//Stop in place treatment when being pushed rapidly
  {
    distance_zeropoint = LQR_distance;//Displacement zero reset
  }

  //Calculate displacement control output
  distance_control  = pid_distance(LQR_distance - distance_zeropoint);
  speed_control     = pid_speed(LQR_speed- 0.1*lpf_joyy(wrobot.joyy) );

  //Wheel lift detection
  robot_speed_last = robot_speed; //Record two consecutive wheel speeds
  robot_speed = LQR_speed;
  if( abs(robot_speed-robot_speed_last) > 10 || \
      abs(robot_speed) > 50 || (jump_flag != 0))  /*If the angular speed and angular acceleration of the 
                                                    wheel are too large or in the recovery period after jumping, 
                                                    it is considered that the wheel is off the 
                                                    ground and needs special treatment.
                                                  */
  {   
    distance_zeropoint = LQR_distance;    //Displacement zero point reset
    LQR_u = angle_control + gyro_control; //When the wheel part is off the ground, \
                                          the quantity of the opposite wheel part is not output.\
                                          Conversely, under normal conditions, the balanced torque is fully output
    pid_lqr_u.error_prev = 0; //The output integral is reset to zero
  }
  else
  {
    LQR_u = angle_control + gyro_control + distance_control + speed_control; 
  }
  
  //Trigger conditions: No signal input from the remote control, \
    normal intervention of the wheel position movement control, \
    and not in the recovery period after jumping
  if( abs(LQR_u)<5 && wrobot.joyy == 0 && abs(distance_control)<4 && (jump_flag == 0))
  {
    
    LQR_u = pid_lqr_u(LQR_u);//Compensate for the nonlinearity of small torque
    //Serial.println(LQR_u);
    angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control));//Center of gravity adaptive
  }
  else
  {
    pid_lqr_u.error_prev = 0; //The output integral is reset to zero
  }

  //The balance control parameters are adaptive
  if(wrobot.height < 50)
  {
    pid_speed.P = 0.7;
  }
  else if(wrobot.height < 64)
  {
    pid_speed.P = 0.6;
  }
  else
  {
    pid_speed.P = 0.5;
  }
}

//Leg movement control
void leg_loop(){
  jump_loop();
  if(jump_flag == 0)//Not in a jumping state
  {
    //Adaptive control of the body height
    ACC[0] = 8;
    ACC[1] = 8;
    Speed[0] = 200;
    Speed[1] = 200;
    float roll_angle  = (float)mpu6050.getAngleX() + 2.0;
    //leg_position_add += pid_roll_angle(roll_angle);
    leg_position_add = pid_roll_angle(lpf_roll(roll_angle));//test
    Position[0] = 2048 + 12 + 8.4*(wrobot.height-32) - leg_position_add;
    Position[1] = 2048 - 12 - 8.4*(wrobot.height-32) - leg_position_add;
    if( Position[0]<2110 )
      Position[0]=2110;
    else if( Position[0]>2510 )
      Position[0]=2510;
    if( Position[1]<1586 )
      Position[1]=1586;
    else if( Position[1]>1986 )
      Position[1]=1986;
    sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
  }  
}

//Jump control
void jump_loop(){
  if( (wrobot.dir_last == 5) && (wrobot.dir == 4) && (jump_flag == 0) )
  {
      ACC[0] = 0;
      ACC[1] = 0;
      Speed[0] = 0;
      Speed[1] = 0;
      Position[0] = 2048 + 12 + 8.4*(80-32);
      Position[1] = 2048 - 12 - 8.4*(80-32);
      sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

      jump_flag = 1;
  }
  if( jump_flag > 0 )
  {
    jump_flag++;
    if( (jump_flag > 30) && (jump_flag < 35) )
    {
      ACC[0] = 0;
      ACC[1] = 0;
      Speed[0] = 0;
      Speed[1] = 0;
      Position[0] = 2048 + 12 + 8.4*(40-32);
      Position[1] = 2048 - 12 - 8.4*(40-32);
      sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

      jump_flag = 40;
    }
    if(jump_flag > 200)
    {
      jump_flag = 0;//Ready to jump again
    }
  }
}

//yaw axis steering control
void yaw_loop(){
  //YAW_output = 0.03*(YAW_Kp*YAW_angle_total + YAW_Kd*YAW_gyro);
  yaw_angle_addup();
  
  YAW_angle_total += wrobot.joyx*0.002;
  float yaw_angle_control = pid_yaw_angle(YAW_angle_total);
  float yaw_gyro_control  = pid_yaw_gyro(YAW_gyro);
  YAW_output = yaw_angle_control + yaw_gyro_control;  
}

//Web Data Update
void web_loop(){
  webserver.handleClient();
  websocket.loop();
  rp.spinOnce();//Update the control information returned by the web end
}

//The yaw axis Angle accumulation function
void yaw_angle_addup() {
  YAW_angle  = (float)mpu6050.getAngleZ();;
  YAW_gyro   = (float)mpu6050.getGyroZ();

  if(YAW_angle_zero_point == (-10))
  {
    YAW_angle_zero_point = YAW_angle;
  }

  float yaw_angle_1,yaw_angle_2,yaw_addup_angle;
  if(YAW_angle > YAW_angle_last)
  {
    yaw_angle_1 = YAW_angle - YAW_angle_last;
    yaw_angle_2 = YAW_angle - YAW_angle_last - 2*PI;
  }
  else
  {
    yaw_angle_1 = YAW_angle - YAW_angle_last;
    yaw_angle_2 = YAW_angle - YAW_angle_last + 2*PI;
  }

  if(abs(yaw_angle_1)>abs(yaw_angle_2))
  {
    yaw_addup_angle=yaw_angle_2;
  }
  else
  {
    yaw_addup_angle=yaw_angle_1;
  }

  YAW_angle_total = YAW_angle_total + yaw_addup_angle;
  YAW_angle_last = YAW_angle;
}

void basicWebCallback(void)
{
  webserver.send(300, "text/html", basic_web);
}

void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  if(type == WStype_TEXT)
  {
    String payload_str = String((char*) payload);   
    StaticJsonDocument<300> doc;  
    DeserializationError error = deserializeJson(doc, payload_str);

    String mode_str = doc["mode"];
    if(mode_str == "basic")
    {
      rp.parseBasic(doc);
    }
  }
}

//Voltage detection initialization
void adc_calibration_init()
{
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

//Voltage detection
void bat_check()
{
  if(bat_check_num > 1000)
  {
    //Voltage reading
    uint32_t sum = 0;
    sum= analogRead(BAT_PIN);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(sum, &adc_chars);
    double battery=(voltage*3.97)/1000.0;

    Serial.println(battery);
    //Battery display
    if(battery>7.8)
      digitalWrite(LED_BAT,HIGH);
    else
      digitalWrite(LED_BAT,LOW);

    bat_check_num = 0;
  }
  else
    bat_check_num++;
}
