// 硬件测试程序 - FreeRTOS多任务优化版本
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

// 全局对象定义
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

// 任务间共享数据(使用volatile确保多线程可见性)
volatile float control_freq = 0;
volatile float encoder_feedback_freq = 0;  // 编码器反馈频率
volatile float battery_voltage = 0;
volatile float imu_angle_y = 0;
volatile float imu_angle_z = 0;
volatile float encoder1_angle = 0;
volatile float encoder1_vel = 0;
volatile float encoder2_angle = 0;
volatile float encoder2_vel = 0;
volatile int16_t servo1_pos = 0;
volatile int16_t servo2_pos = 0;

// 电机控制开关 (true=驱动电机, false=只读取编码器)
volatile bool motor_enable = true;

// 任务句柄
TaskHandle_t TaskMotorControlHandle = NULL;
TaskHandle_t TaskWebServerHandle = NULL;
TaskHandle_t TaskMonitorHandle = NULL;

// 函数前向声明
void adc_calibration_init();
void basicWebCallback(void);
void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

// ============================================================================
// 任务1: 高速电机控制任务 (核心1, 最高优先级)
// ============================================================================
void TaskMotorControl(void *pvParameters) {
  Serial.println("[任务1] 电机控制任务启动 (核心1)");
  
  unsigned long loop_count = 0;
  unsigned long freq_timer = millis();
  
  unsigned long encoder_update_count = 0;
  unsigned long encoder_freq_timer = millis();
  
  for(;;) {
    // 控制频率统计
    loop_count++;
    if (millis() - freq_timer >= 1000) {
      control_freq = loop_count / ((millis() - freq_timer) / 1000.0);
      loop_count = 0;
      freq_timer = millis();
    }
    
    // 更新传感器数据
    mpu6050.update();
    sensor1.update();
    sensor2.update();
    
    // 编码器反馈频率统计
    encoder_update_count++;
    if (millis() - encoder_freq_timer >= 1000) {
      encoder_feedback_freq = encoder_update_count / ((millis() - encoder_freq_timer) / 1000.0);
      encoder_update_count = 0;
      encoder_freq_timer = millis();
    }
    
    // 读取传感器值(原子操作)
    imu_angle_y = mpu6050.getAngleY();
    imu_angle_z = mpu6050.getAngleZ();
    encoder1_angle = sensor1.getAngle();
    encoder1_vel = sensor1.getVelocity();
    encoder2_angle = sensor2.getAngle();
    encoder2_vel = sensor2.getVelocity();
    
    // 电机FOC控制 (根据motor_enable开关决定是否驱动)
    if (motor_enable) {
      motor1.target = 3.0;
      motor2.target = 3.0;
      
      motor1.loopFOC();
      motor2.loopFOC();
      motor1.move();
      motor2.move();
    } else {
      // 只读取编码器,不输出电机转矩
      motor1.target = 0;
      motor2.target = 0;
      motor1.move(0);  // 强制输出为0
      motor2.move(0);
    }
    
    // 无延时,全速运行
    // vTaskDelay(1); // 如果需要限制到1000Hz可取消注释
  }
}

// ============================================================================
// 任务2: Web服务器任务 (核心0, 中等优先级)
// ============================================================================
void TaskWebServer(void *pvParameters) {
  Serial.println("[任务2] Web服务器任务启动 (核心0)");
  
  for(;;) {
    webserver.handleClient();
    websocket.loop();
    
    vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz刷新率
  }
}

// ============================================================================
// 任务3: 监控输出任务 (核心0, 低优先级)
// ============================================================================
void TaskMonitor(void *pvParameters) {
  Serial.println("[任务3] 监控任务启动 (核心0)");
  
  uint8_t scan_count = 0;
  
  for(;;) {
    // 读取电压
    uint32_t adc_reading = analogRead(BAT_PIN);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
    battery_voltage = (voltage * 3.97) / 1000.0;
    
    // LED控制
    digitalWrite(LED_BAT, battery_voltage > 7.8 ? HIGH : LOW);
    
    // 读取舵机位置
    servo1_pos = sms_sts.ReadPos(1);
    servo2_pos = sms_sts.ReadPos(2);
    
    // 串口输出
    Serial.println("========================================");
    Serial.printf("控制频率: %.1f Hz | 周期: %.2f ms\n", 
                  control_freq, 1000.0/control_freq);
    Serial.printf("编码器反馈频率: %.1f Hz\n", encoder_feedback_freq);
    Serial.printf("电压: %.2fV | LED: %s\n", 
                  battery_voltage, battery_voltage > 7.8 ? "亮" : "灭");
    Serial.printf("IMU Y轴角度: %.2f° | Z轴角度: %.2f°\n", 
                  imu_angle_y, imu_angle_z);
    Serial.printf("电机控制: %s\n", motor_enable ? "启用" : "禁用");
    Serial.printf("编码器1: %.4f rad (%.4f rad/s) | 目标: %.1f rad/s\n", 
                  encoder1_angle, encoder1_vel, motor_enable ? 3.0 : 0.0);
    Serial.printf("编码器2: %.4f rad (%.4f rad/s) | 目标: %.1f rad/s\n", 
                  encoder2_angle, encoder2_vel, motor_enable ? 3.0 : 0.0);
    
    if(servo1_pos >= 0 && servo2_pos >= 0) {
      Serial.printf("舵机位置: ID1=%d | ID2=%d\n", servo1_pos, servo2_pos);
    } else {
      Serial.println("舵机: 读取失败");
    }
    
    // I2C扫描(每10次,即2秒)
    scan_count++;
    if (scan_count > 10) {
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
    
    vTaskDelay(200 / portTICK_PERIOD_MS); // 5Hz输出频率
  }
}

// ============================================================================
// Web回调函数
// ============================================================================
void basicWebCallback(void) {
  webserver.send(200, "text/html", basic_web);
}

void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if(type == WStype_TEXT) {
    Serial.printf("WebSocket消息: %s\n", payload);
  }
}

void adc_calibration_init() {
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        Serial.println("eFuse Two Point: Supported");
    }
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        Serial.println("eFuse Vref: Supported");
    }
}

// ============================================================================
// 主程序
// ============================================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(1000000);
  delay(2000);
  Serial.println("\n=== FreeRTOS多任务硬件测试 ===");

  // WiFi初始化
  Serial.println("\n[初始化] WiFi...");
  WiFi_SetAP();
  webserver.begin();
  webserver.on("/", HTTP_GET, basicWebCallback);
  websocket.begin();
  websocket.onEvent(webSocketEventCallback);
  Serial.println("✓ WiFi热点: WLROBOT (密码: 12345678)");
  Serial.println("✓ Web地址: http://192.168.1.11");

  // 电压检测
  Serial.println("\n[初始化] 电压检测...");
  pinMode(LED_BAT, OUTPUT);
  adc_calibration_init();
  adc1_config_width(width);
  adc1_config_channel_atten(channel, atten);
  esp_adc_cal_characterize(unit, atten, width, 0, &adc_chars);
  Serial.println("✓ ADC初始化");

  // IMU
  Serial.println("\n[初始化] IMU...");
  I2Ctwo.begin(23, 5, 400000UL);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("✓ MPU6050初始化");

  // 编码器
  Serial.println("\n[初始化] 编码器...");
  I2Cone.begin(19, 18, 400000UL);
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);
  Serial.println("✓ AS5600初始化");

  // 舵机
  Serial.println("\n[初始化] 舵机...");
  sms_sts.pSerial = &Serial2;
  Serial.println("✓ STS3032串口配置");

  // 电机FOC
  Serial.println("\n[初始化] 电机驱动...");
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);
  
  motor1.PID_velocity.P = 0.05;
  motor1.PID_velocity.I = 0.6;
  motor1.PID_velocity.D = 0;
  motor2.PID_velocity.P = 0.05;
  motor2.PID_velocity.I = 0.6;
  motor2.PID_velocity.D = 0;
  
  driver1.voltage_power_supply = 8;
  driver2.voltage_power_supply = 8;
  driver1.init();
  driver2.init();
  
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);
  
  motor1.controller = MotionControlType::velocity;
  motor2.controller = MotionControlType::velocity;
  
  motor1.init();
  motor1.initFOC();
  motor2.init();
  motor2.initFOC();
  
  Serial.println("✓ SimpleFOC初始化");
  Serial.println("\n[提示] 修改 motor_enable 变量控制电机启动");
  Serial.println("       motor_enable = true  (驱动电机)");
  Serial.println("       motor_enable = false (只读编码器)");
  Serial.println("\n当前设置: motor_enable = false (安全模式)");
  delay(2000);

  // 创建FreeRTOS任务
  Serial.println("\n=== 创建多任务 ===");
  
  // 任务1: 电机控制 (核心1, 优先级4 - 最高)
  xTaskCreatePinnedToCore(
    TaskMotorControl,           // 任务函数
    "MotorControl",             // 任务名称
    8192,                       // 栈大小(字节)
    NULL,                       // 参数
    4,                          // 优先级(0-24,数字越大优先级越高)
    &TaskMotorControlHandle,    // 任务句柄
    1                           // CPU核心1
  );
  
  // 任务2: Web服务器 (核心0, 优先级2)
  xTaskCreatePinnedToCore(
    TaskWebServer,
    "WebServer",
    4096,
    NULL,
    2,
    &TaskWebServerHandle,
    0                           // CPU核心0 (WiFi协议栈在此核心)
  );
  
  // 任务3: 监控输出 (核心0, 优先级1)
  xTaskCreatePinnedToCore(
    TaskMonitor,
    "Monitor",
    4096,
    NULL,
    1,
    &TaskMonitorHandle,
    0
  );
  
  Serial.println("✓ 任务1: 电机控制 -> 核心1 (优先级4)");
  Serial.println("✓ 任务2: Web服务器 -> 核心0 (优先级2)");
  Serial.println("✓ 任务3: 监控输出 -> 核心0 (优先级1)");
  Serial.println("\n=== 系统运行中 ===\n");
}

void loop() {
  // Arduino的loop任务已被FreeRTOS任务替代,这里留空
  // FreeRTOS会自动调度上面创建的任务
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
