// 硬件基础测试程序 - 替换 wl_pro_robot.cpp 使用
#include <Arduino.h>
#include <MPU6050_tockn.h>
#include "Servo_STS3032.h"
#include <SimpleFOC.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "basic_web.h"
#include "wifi_config.h"
#include "esp_adc_cal.h"

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
MPU6050 mpu6050(I2Ctwo);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
SMS_STS sms_sts;

// 电机和驱动器实例
BLDCMotor motor1 = BLDCMotor(7);
BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32,33,25,22);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(26,27,14,12);

// Web服务器
WebServer webserver;
WebSocketsServer websocket = WebSocketsServer(81);

// 电压检测
int BAT_PIN = 35;
static esp_adc_cal_characteristics_t adc_chars;
static const adc1_channel_t channel = ADC1_CHANNEL_7;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_12;
static const adc_unit_t unit = ADC_UNIT_1;
#define LED_BAT 13

void adc_calibration_init() {
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        Serial.println("eFuse Two Point: Supported");
    }
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        Serial.println("eFuse Vref: Supported");
    }
}

void basicWebCallback(void) {
  webserver.send(200, "text/html", basic_web);
}

void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if(type == WStype_TEXT) {
    Serial.printf("WebSocket消息: %s\n", payload);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(1000000);
  delay(2000);
  Serial.println("=== 硬件测试开始 ===");

  // 测试1: WiFi
  Serial.println("\n[1] 初始化WiFi...");
  WiFi_SetAP();
  webserver.begin();
  webserver.on("/", HTTP_GET, basicWebCallback);
  websocket.begin();
  websocket.onEvent(webSocketEventCallback);
  Serial.println("✓ WiFi热点启动: ESP32_AP");
  Serial.println("✓ Web地址: http://192.168.4.1");

  // 测试2: 电压检测
  Serial.println("\n[2] 初始化电压检测...");
  pinMode(LED_BAT, OUTPUT);
  adc_calibration_init();
  adc1_config_width(width);
  adc1_config_channel_atten(channel, atten);
  esp_adc_cal_characterize(unit, atten, width, 0, &adc_chars);
  Serial.println("✓ ADC初始化成功");

  // 测试3: IMU(MPU6050)
  Serial.println("\n[3] 测试IMU...");
  I2Ctwo.begin(23, 5, 400000UL);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("✓ IMU初始化成功");

  // 测试4: 编码器(AS5600)
  Serial.println("\n[4] 测试编码器...");
  I2Cone.begin(19, 18, 400000UL);
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);
  
  // 检测编码器是否连接
  delay(100);
  float test_angle1 = sensor1.getAngle();
  float test_angle2 = sensor2.getAngle();
  
  if (test_angle1 == 0 && test_angle2 == 0) {
    Serial.println("⚠ 警告: 编码器可能未连接或I2C地址冲突!");
  } else {
    Serial.printf("✓ 编码器初始化成功 (初始角度: %.2f, %.2f)\n", test_angle1, test_angle2);
  }

  // 测试5: 舵机通信
  Serial.println("\n[5] 初始化舵机通信...");
  sms_sts.pSerial = &Serial2;
  Serial.println("✓ 舵机串口配置完成");

  // 测试6: 电机FOC初始化
  Serial.println("\n[6] 初始化电机驱动...");
  
  // 连接编码器
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);
  
  // 速度环PID参数
  motor1.PID_velocity.P = 0.05;
  motor1.PID_velocity.I = 0.6;
  motor1.PID_velocity.D = 0;
  motor2.PID_velocity.P = 0.05;
  motor2.PID_velocity.I = 0.6;
  motor2.PID_velocity.D = 0;
  
  // 驱动器设置
  driver1.voltage_power_supply = 8;
  driver2.voltage_power_supply = 8;
  driver1.init();
  driver2.init();
  
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);
  
  // 设置为速度模式
  motor1.controller = MotionControlType::velocity;
  motor2.controller = MotionControlType::velocity;
  
  motor1.init();
  motor1.initFOC();
  motor2.init();
  motor2.initFOC();
  
  Serial.println("✓ 电机FOC初始化成功");
  Serial.println("提示: 3秒后电机将以低速转动");
  delay(3000);

  Serial.println("\n=== 初始化完成,开始实时监控 ===");
}

void loop() {
  // 控制频率统计
  static unsigned long loop_count = 0;
  static unsigned long freq_timer = 0;
  static float control_freq = 0;
  
  loop_count++;
  if (millis() - freq_timer >= 1000) {
    control_freq = loop_count / ((millis() - freq_timer) / 1000.0);
    loop_count = 0;
    freq_timer = millis();
  }

  // Web服务器处理
  webserver.handleClient();
  websocket.loop();

  // 更新传感器数据
  mpu6050.update();
  
  // 电机匀速转动测试 (2 rad/s 约等于 19 RPM)
  motor1.target = 3.0;  // 目标速度 2 rad/s
  motor2.target = 3.0;
  
  motor1.loopFOC();
  motor2.loopFOC();
  motor1.move();
  motor2.move();

  // 读取电压
  static uint16_t bat_count = 0;
  static double battery = 0;
  if (bat_count > 500) {
    uint32_t adc_reading = analogRead(BAT_PIN);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
    battery = (voltage * 3.97) / 1000.0;
    
    // 电量LED控制
    if (battery > 7.8)
      digitalWrite(LED_BAT, HIGH);
    else
      digitalWrite(LED_BAT, LOW);
    
    bat_count = 0;
  }
  bat_count++;

  // 串口输出(每200ms一次)
  static unsigned long last_print = 0;
  if (millis() - last_print > 200) {
    // 手动读取编码器原始数据
    sensor1.update();
    sensor2.update();
    
    float angle1 = sensor1.getAngle();
    float angle2 = sensor2.getAngle();
    float vel1 = sensor1.getVelocity();
    float vel2 = sensor2.getVelocity();
    
    Serial.println("========================================");
    Serial.printf("控制频率: %.1f Hz | 周期: %.2f ms\n", 
                  control_freq, 1000.0/control_freq);
    Serial.printf("电压: %.2fV | LED: %s\n", battery, battery > 7.8 ? "亮" : "灭");
    Serial.printf("IMU Y轴角度: %.2f° | Z轴角度: %.2f°\n", 
                  mpu6050.getAngleY(), mpu6050.getAngleZ());
    Serial.printf("编码器1: %.4f rad (%.4f rad/s) | 目标: %.1f rad/s\n", 
                  angle1, vel1, motor1.target);
    Serial.printf("编码器2: %.4f rad (%.4f rad/s) | 目标: %.1f rad/s\n", 
                  angle2, vel2, motor2.target);
    
    // 读取舵机位置
    s16 servo1_pos = sms_sts.ReadPos(1);
    s16 servo2_pos = sms_sts.ReadPos(2);
    if(servo1_pos >= 0 && servo2_pos >= 0) {
      Serial.printf("舵机位置: ID1=%d | ID2=%d\n", servo1_pos, servo2_pos);
    } else {
      Serial.println("舵机: 读取失败(检查接线/ID/供电)");
    }
    
    // I2C总线扫描(每5秒一次)
    static uint8_t scan_count = 0;
    if (scan_count > 25) {
      Serial.println("\n--- I2C设备扫描 ---");
      for (uint8_t addr = 1; addr < 127; addr++) {
        I2Cone.beginTransmission(addr);
        if (I2Cone.endTransmission() == 0) {
          Serial.printf("I2C1发现设备: 0x%02X\n", addr);
        }
        I2Ctwo.beginTransmission(addr);
        if (I2Ctwo.endTransmission() == 0) {
          Serial.printf("I2C2发现设备: 0x%02X\n", addr);
        }
      }
      scan_count = 0;
    }
    scan_count++;
    
    last_print = millis();
  }

  delay(1);  // 避免看门狗触发
}
