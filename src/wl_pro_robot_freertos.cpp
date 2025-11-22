// -----------------------------------------------------------------------------
// Copyright (c) 2024 Mu Shibo
// FreeRTOS多任务优化版本 - 分离Web任务提高控制稳定性
// -----------------------------------------------------------------------------

#include <Arduino.h>
#include <MPU6050_tockn.h>
#include "Servo_STS3032.h"
#include <SimpleFOC.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <FS.h>
#include "basic_web.h"
#include "robot.h"
#include "wifi_config.h"
#include "esp_adc_cal.h"

/************函数前向声明*************/
void lqr_balance_loop();
void leg_loop();
void jump_loop();
void yaw_loop();
void yaw_angle_addup();
void basicWebCallback(void);
void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void adc_calibration_init();

/************实例定义*************/

// 电机实例
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(26,27,14,12);

// 编码器实例
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

// PID控制器实例
PIDController pid_angle     {.P = 0.5,    .I = 0.40,   .D = 0, .ramp = 100000, .limit = 10};
PIDController pid_gyro      {.P = 0.06, .I = 0,   .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_distance  {.P = 0.5,  .I = 0,   .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_speed     {.P = 0.7,  .I = 0,   .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_yaw_angle {.P = 1.0,  .I = 0,   .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_yaw_gyro  {.P = 0.04, .I = 0,   .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_lqr_u     {.P = 1,    .I = 15,  .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_zeropoint {.P = 0.002,.I = 0,   .D = 0, .ramp = 100000, .limit = 4};
PIDController pid_roll_angle{.P = 8,    .I = 0,   .D = 0, .ramp = 100000, .limit = 450};

// 低通滤波器实例
LowPassFilter lpf_joyy{.Tf = 0.2};
LowPassFilter lpf_zeropoint{.Tf = 0.1};
LowPassFilter lpf_roll{.Tf = 0.3};

// Commander通信实例
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

// WebServer实例
WebServer webserver;
WebSocketsServer websocket = WebSocketsServer(81);
RobotProtocol rp(20);

// STS舵机实例
SMS_STS sms_sts;

// MPU6050实例
MPU6050 mpu6050(I2Ctwo);

/************参数定义*************/
#define pi 3.1415927

// LQR自平衡控制器参数
float LQR_angle = 0;
float LQR_gyro  = 0;
float LQR_speed = 0;
float LQR_distance = 0;
float angle_control   = 0;
float gyro_control    = 0;
float speed_control   = 0;
float distance_control = 0;
float LQR_u = 0;
float angle_zeropoint = -0.8;
float distance_zeropoint = -256.0;

// YAW轴控制数据
float YAW_gyro = 0;
float YAW_angle = 0;
float YAW_angle_last = 0;
float YAW_angle_total = 0;
float YAW_angle_zero_point = -10;
float YAW_output = 0;

// 腿部舵机控制数据
byte ID[2];
s16 Position[2];
u16 Speed[2];
byte ACC[2];

// 逻辑处理标志位
float robot_speed = 0;
float robot_speed_last = 0;
int wrobot_move_stop_flag = 0;
int jump_flag = 0;
float leg_position_add = 0;
int uncontrolable = 0;

// 电压检测
uint16_t bat_check_num = 0;
int BAT_PIN = 35;
static esp_adc_cal_characteristics_t adc_chars;
static const adc1_channel_t channel = ADC1_CHANNEL_7;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_12;
static const adc_unit_t unit = ADC_UNIT_1;
#define LED_BAT 13

// FreeRTOS任务句柄
TaskHandle_t TaskMotorControlHandle = NULL;
TaskHandle_t TaskWebServerHandle = NULL;
TaskHandle_t TaskMonitorHandle = NULL;

// 任务间共享数据(使用volatile确保多线程可见性)
volatile float control_freq = 0;
volatile float pitch_angle = 0;  // Pitch角度监控

// ============================================================================
// 任务1: 高速电机控制任务 (核心1, 最高优先级, 8KB栈)
// ============================================================================
void TaskMotorControl(void *pvParameters) {
  Serial.println("[任务1] 电机控制任务启动 (核心1)");
  
  unsigned long loop_count = 0;
  unsigned long freq_timer = millis();
  
  for(;;) {
    // 控制频率统计
    loop_count++;
    if (millis() - freq_timer >= 1000) {
      control_freq = loop_count / ((millis() - freq_timer) / 1000.0);
      loop_count = 0;
      freq_timer = millis();
    }
    
    // IMU数据更新
    mpu6050.update();
    
    // 更新共享变量
    pitch_angle = mpu6050.getAngleY();
    
    // LQR自平衡控制
    lqr_balance_loop();
    
    // YAW轴转向控制
    yaw_loop();
    
    // 腿部动作控制
    leg_loop();
    
    // 将自平衡计算输出转矩赋给电机
    motor1.target = (-0.5)*(LQR_u + YAW_output);
    motor2.target = (-0.5)*(LQR_u - YAW_output);

    // 倒地失控后关闭输出
    if(abs(LQR_angle) > 50.0f) {
      uncontrolable = 1;
    }
    if(uncontrolable != 0) {
      if(abs(LQR_angle) < 10.0f) {
        uncontrolable++;
      }
      if(uncontrolable > 100) {
        uncontrolable = 0;
      }
    }
    
    // 关停输出（遥控停止或角度过大失控）
    if(wrobot.go==0 || uncontrolable!=0) {
      motor1.target = (-0.5)*(angle_control + gyro_control);
      motor2.target = (-0.5)*(angle_control + gyro_control);
      leg_position_add = 0;
    }
    
    // 记录上一次的遥控数据
    wrobot.dir_last  = wrobot.dir;
    wrobot.joyx_last = wrobot.joyx;
    wrobot.joyy_last = wrobot.joyy;
    
    // 迭代计算FOC相电压
    motor1.loopFOC();
    motor2.loopFOC();
    
    // 设置轮部电机输出
    motor1.move();
    motor2.move();
    
    // Commander命令处理
    command.run();
    
    // 不添加延迟,让任务以最高频率运行
    taskYIELD(); // 让出CPU给其他同优先级任务
  }
}

// ============================================================================
// 任务2: Web服务器任务 (核心0, 低优先级, 4KB栈)
// ============================================================================
void TaskWebServer(void *pvParameters) {
  Serial.println("[任务2] Web服务器任务启动 (核心0)");
  
  for(;;) {
    webserver.handleClient();
    websocket.loop();
    rp.spinOnce(); // 更新web端回传的控制信息
    
    // Web任务以较低频率运行(约100Hz)
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ============================================================================
// 任务3: 监控任务 (核心0, 最低优先级, 2KB栈)
// ============================================================================
void TaskMonitor(void *pvParameters) {
  Serial.println("[任务3] 监控任务启动 (核心0)");
  
  unsigned long last_print = 0;
  
  for(;;) {
    // 电压检测
    if(bat_check_num > 1000) {
      uint32_t sum = analogRead(BAT_PIN);
      uint32_t voltage = esp_adc_cal_raw_to_voltage(sum, &adc_chars);
      double battery = (voltage*3.97)/1000.0;
      
      if(battery > 7.8)
        digitalWrite(LED_BAT, HIGH);
      else
        digitalWrite(LED_BAT, LOW);
      
      bat_check_num = 0;
    } else {
      bat_check_num++;
    }
    
    // 每500ms打印一次频率和Pitch角度
    if(millis() - last_print >= 500) {
      Serial.print("控制频率: ");
      Serial.print(control_freq, 1);
      Serial.print(" Hz | Pitch角度: ");
      Serial.print(pitch_angle, 2);
      Serial.println("°");
      
      last_print = millis();
    }
    
    // 监控任务以2Hz运行
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// ============================================================================
// Setup函数
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== 轮腿机器人控制系统 (FreeRTOS版本) ===");
  
  // 串口2初始化(舵机通信)
  Serial2.begin(1000000);

  // WiFi初始化
  Serial.println("[初始化] WiFi AP模式启动...");
  WiFi_SetAP();
  webserver.begin();
  webserver.on("/", HTTP_GET, basicWebCallback);
  websocket.begin();
  websocket.onEvent(webSocketEventCallback);
  Serial.println("[初始化] WiFi启动完成");

  // 舵机初始化
  Serial.println("[初始化] 舵机初始化...");
  sms_sts.pSerial = &Serial2;
  ID[0] = 1;
  ID[1] = 2;
  ACC[0] = 30;
  ACC[1] = 30;
  Speed[0] = 300;
  Speed[1] = 300;
  Position[0] = 2148;
  Position[1] = 1948;
  sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
  Serial.println("[初始化] 舵机初始化完成");

  // 电压检测初始化
  adc_calibration_init();
  adc1_config_width(width);
  adc1_config_channel_atten(channel, atten);
  esp_adc_cal_characterize(unit, atten, width, 0, &adc_chars);
  pinMode(LED_BAT, OUTPUT);

  // 编码器初始化
  Serial.println("[初始化] I2C编码器初始化...");
  I2Cone.begin(19, 18, 400000UL);
  I2Ctwo.begin(23, 5, 400000UL);
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);
  Serial.println("[初始化] 编码器初始化完成");

  // MPU6050初始化
  Serial.println("[初始化] MPU6050初始化...");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("[初始化] MPU6050初始化完成");

  // 电机初始化
  Serial.println("[初始化] 电机和驱动器初始化...");
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  motor1.PID_velocity.P = 0.05;
  motor1.PID_velocity.I = 1;
  motor1.PID_velocity.D = 0;
  motor2.PID_velocity.P = 0.05;
  motor2.PID_velocity.I = 1;
  motor2.PID_velocity.D = 0;

  motor1.voltage_sensor_align = 6;
  motor2.voltage_sensor_align = 6;
  driver1.voltage_power_supply = 8;
  driver2.voltage_power_supply = 8;
  driver1.init();
  driver2.init();

  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  motor1.torque_controller = TorqueControlType::voltage;
  motor2.torque_controller = TorqueControlType::voltage;
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;

  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);
  motor1.init();
  motor1.initFOC();
  motor2.init();
  motor2.initFOC();
  Serial.println("[初始化] 电机初始化完成");

  // Commander映射
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
  command.verbose = VerboseMode::user_friendly;

  Serial.println("\n=== 创建FreeRTOS任务 ===");
  
  // 创建任务1: 高速电机控制 (核心1, 优先级3, 8KB栈)
  xTaskCreatePinnedToCore(
    TaskMotorControl,           // 任务函数
    "MotorControl",             // 任务名称
    8192,                       // 栈大小(字节)
    NULL,                       // 参数
    3,                          // 优先级(0-24,数字越大优先级越高)
    &TaskMotorControlHandle,    // 任务句柄
    1                           // 核心1
  );
  Serial.println("✓ 任务1创建: 电机控制 (核心1, 优先级3, 8KB栈)");

  // 创建任务2: Web服务器 (核心0, 优先级1, 4KB栈)
  xTaskCreatePinnedToCore(
    TaskWebServer,
    "WebServer",
    4096,
    NULL,
    1,
    &TaskWebServerHandle,
    0
  );
  Serial.println("✓ 任务2创建: Web服务器 (核心0, 优先级1, 4KB栈)");

  // 创建任务3: 监控打印 (核心0, 优先级0, 2KB栈)
  xTaskCreatePinnedToCore(
    TaskMonitor,
    "Monitor",
    2048,
    NULL,
    0,
    &TaskMonitorHandle,
    0
  );
  Serial.println("✓ 任务3创建: 监控 (核心0, 优先级0, 2KB栈)");

  Serial.println("\n=== 系统启动完成 ===\n");
  delay(500);
}

// ============================================================================
// Loop函数(空,使用FreeRTOS任务替代)
// ============================================================================
void loop() {
  // 空循环,所有功能由FreeRTOS任务处理
  vTaskDelete(NULL); // 删除默认loop任务
}

// ============================================================================
// LQR自平衡控制
// ============================================================================
void lqr_balance_loop() {
  // 给负值是因为按照当前的电机接线,正转矩会向后转
  LQR_distance = (-0.5) * (motor1.shaft_angle + motor2.shaft_angle);
  LQR_speed = (-0.5) * (motor1.shaft_velocity + motor2.shaft_velocity);
  LQR_angle = (float)mpu6050.getAngleY();
  LQR_gyro = (float)mpu6050.getGyroY();

  // 计算自平衡输出
  angle_control = pid_angle(LQR_angle - angle_zeropoint);
  gyro_control = pid_gyro(LQR_gyro);

  // 运动细节优化处理
  if(wrobot.joyy != 0) {
    distance_zeropoint = LQR_distance;
    pid_lqr_u.reset();
  }

  if((wrobot.joyx_last!=0 && wrobot.joyx==0) || (wrobot.joyy_last!=0 && wrobot.joyy==0)) {
    wrobot_move_stop_flag = 1;
  }
  if((wrobot_move_stop_flag==1) && (abs(LQR_speed)<0.5)) {
    distance_zeropoint = LQR_distance;
    wrobot_move_stop_flag = 0;
  }

  if(abs(LQR_speed) > 15) {
    distance_zeropoint = LQR_distance;
  }

  // 计算位移控制输出
  distance_control = pid_distance(LQR_distance - distance_zeropoint);
  speed_control = pid_speed(LQR_speed - 0.1*lpf_joyy(wrobot.joyy));

  // 轮部离地检测
  robot_speed_last = robot_speed;
  robot_speed = LQR_speed;
  if(abs(robot_speed-robot_speed_last) > 10 || abs(robot_speed) > 50 || (jump_flag != 0)) {
    distance_zeropoint = LQR_distance;
    LQR_u = angle_control + gyro_control;
    pid_lqr_u.reset();
  } else {
    LQR_u = angle_control + gyro_control + distance_control + speed_control;
  }

  // 触发条件：遥控器无信号输入、轮部位移控制正常介入、不处于跳跃后的恢复时期
  if(abs(LQR_u)<5 && wrobot.joyy == 0 && abs(distance_control)<4 && (jump_flag == 0)) {
    LQR_u = pid_lqr_u(LQR_u);
    angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control));
  } else {
    pid_lqr_u.reset();
  }

  // 平衡控制参数自适应
  if(wrobot.height < 50) {
    pid_speed.P = 0.7;
  } else if(wrobot.height < 64) {
    pid_speed.P = 0.6;
  } else {
    pid_speed.P = 0.5;
  }
}

// ============================================================================
// 腿部动作控制
// ============================================================================
void leg_loop() {
  jump_loop();
  if(jump_flag == 0) {
    ACC[0] = 8;
    ACC[1] = 8;
    Speed[0] = 200;
    Speed[1] = 200;
    float roll_angle = (float)mpu6050.getAngleX() - 2.0;
    leg_position_add = pid_roll_angle(lpf_roll(roll_angle));
    Position[0] = 2048 + 12 + 8.4*(wrobot.height-32) - leg_position_add;
    Position[1] = 2048 - 12 - 8.4*(wrobot.height-32) - leg_position_add;
    
    if(Position[0] < 2110) Position[0] = 2110;
    else if(Position[0] > 2510) Position[0] = 2510;
    if(Position[1] < 1586) Position[1] = 1586;
    else if(Position[1] > 1986) Position[1] = 1986;
    
    sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
  }
}

// ============================================================================
// 跳跃控制
// ============================================================================
void jump_loop() {
  if((wrobot.dir_last == 5) && (wrobot.dir == 4) && (jump_flag == 0)) {
    ACC[0] = 0;
    ACC[1] = 0;
    Speed[0] = 0;
    Speed[1] = 0;
    Position[0] = 2048 + 12 + 8.4*(80-32);
    Position[1] = 2048 - 12 - 8.4*(80-32);
    sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
    jump_flag = 1;
  }
  if(jump_flag > 0) {
    jump_flag++;
    if((jump_flag > 30) && (jump_flag < 35)) {
      ACC[0] = 0;
      ACC[1] = 0;
      Speed[0] = 0;
      Speed[1] = 0;
      Position[0] = 2048 + 12 + 8.4*(40-32);
      Position[1] = 2048 - 12 - 8.4*(40-32);
      sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
      jump_flag = 40;
    }
    if(jump_flag > 200) {
      jump_flag = 0;
    }
  }
}

// ============================================================================
// YAW轴转向控制
// ============================================================================
void yaw_loop() {
  yaw_angle_addup();
  YAW_angle_total += wrobot.joyx * 0.002;
  float yaw_angle_control = pid_yaw_angle(YAW_angle_total);
  float yaw_gyro_control = pid_yaw_gyro(YAW_gyro);
  YAW_output = yaw_angle_control + yaw_gyro_control;
}

// ============================================================================
// YAW轴角度累加
// ============================================================================
void yaw_angle_addup() {
  YAW_angle = (float)mpu6050.getAngleZ();
  YAW_gyro = (float)mpu6050.getGyroZ();

  if(YAW_angle_zero_point == (-10)) {
    YAW_angle_zero_point = YAW_angle;
  }

  float yaw_angle_1, yaw_angle_2, yaw_addup_angle;
  if(YAW_angle > YAW_angle_last) {
    yaw_angle_1 = YAW_angle - YAW_angle_last;
    yaw_angle_2 = YAW_angle - YAW_angle_last - 2*PI;
  } else {
    yaw_angle_1 = YAW_angle - YAW_angle_last;
    yaw_angle_2 = YAW_angle - YAW_angle_last + 2*PI;
  }

  if(abs(yaw_angle_1) > abs(yaw_angle_2)) {
    yaw_addup_angle = yaw_angle_2;
  } else {
    yaw_addup_angle = yaw_angle_1;
  }

  YAW_angle_total = YAW_angle_total + yaw_addup_angle;
  YAW_angle_last = YAW_angle;
}

// ============================================================================
// Web回调函数
// ============================================================================
void basicWebCallback(void) {
  webserver.send(300, "text/html", basic_web);
}

void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if(type == WStype_TEXT) {
    String payload_str = String((char*) payload);
    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, payload_str);

    String mode_str = doc["mode"];
    if(mode_str == "basic") {
      rp.parseBasic(doc);
    }
  }
}

// ============================================================================
// 电压检测初始化
// ============================================================================
void adc_calibration_init() {
  if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
    Serial.println("eFuse Two Point: Supported");
  } else {
    Serial.println("eFuse Two Point: NOT supported");
  }
  if(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
    Serial.println("eFuse Vref: Supported");
  } else {
    Serial.println("eFuse Vref: NOT supported");
  }
}
