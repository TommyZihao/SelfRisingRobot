#include <M5Unified.h>
#include <Kalman.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <math.h>
#include "policy_network.h"

// V3: lower GPIO1 +/-90 deg; upper GPIO2 +/-120 deg.
const char* POLICY_VERSION = "V3.1";

// ==================== 显示布局 ====================
#define SCREEN_W 128
#define SCREEN_H 128

// 姿态区域 (顶部到底部，全屏)
#define POSE_X      0
#define POSE_Y      0
#define POSE_W      128
#define POSE_H      108

// IP 地址区域 (底部)
#define IP_X        0
#define IP_Y        110
#define IP_W        128
#define IP_H        18

// 颜色定义
#define C_BG        0x0000  // 黑
#define C_TEXT      0xFFFF  // 白
#define C_ROLL_POS  0xF800  // 红 - 右侧倒
#define C_ROLL_NEG  0x001F  // 蓝 - 左侧倒
#define C_PITCH_POS 0x07E0  // 绿 - 前倒
#define C_PITCH_NEG 0xFFE0  // 黄 - 后倒
#define C_UPRIGHT   0x07FF  // 青 - 直立
#define C_GETUP     0xF81F  // 紫 - 起身中
#define C_DONE      0x07E0  // 绿 - 完成
#define C_SERVO1    0xFD20  // 橙
#define C_SERVO2    0x07FF  // 青

// ==================== 全局变量 ====================
WebServer server(80);
DNSServer dnsServer;

const char ssid[] = "SelfRisingRobot";
const char pass[] = "";

const IPAddress ip(192,168,42,1);
const IPAddress subnet(255,255,255,0);

Servo servo1, servo2;

int servo1_pin = 1;
int servo2_pin = 2;

int ang1 = 0, offset1 = 0, pulse1 = 1000;
int ang2 = 0, offset2 = 0, pulse2 = 1000;

float pitch, roll;
float servo1_target = 0.0f, servo2_target = 0.0f;

float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float theta_X = 0.0, theta_Y = 0.0;
float kalAngleX, kalAngleDotX, kalAngleY, kalAngleDotY;
Kalman kalmanX, kalmanY;

unsigned long oldTime = 0, loopTime, nowTime;
float dt;

Preferences preferences;

enum RobotMode {
  MODE_MANUAL,
  MODE_GETUP,
  MODE_GETUP_DONE,
  MODE_GETUP_ABORT,
};

volatile RobotMode robotMode = MODE_MANUAL;
volatile bool autoGetupEnabled = true;

// 策略暂停标志（按键切换）
volatile bool policyPaused = false;

// RGB LED 状态颜色
uint8_t ledR = 0, ledG = 0, ledB = 0;

// 历史轨迹缓存（最近100个点）
#define HISTORY_LEN 100
float rollHistory[HISTORY_LEN];
float pitchHistory[HISTORY_LEN];
float servo1History[HISTORY_LEN];
float servo2History[HISTORY_LEN];
int historyIdx = 0;

// 数据推送控制（HTTP 轮询替代 WebSocket）
unsigned long lastDataMs = 0;
const unsigned long DATA_PERIOD_MS = 50;  // 20Hz

const float TARGET_DELTA_RAD = 0.08f;
const float TARGET1_LIMIT_RAD = PI / 2.0f;
const float TARGET2_LIMIT_RAD = 2.0f * PI / 3.0f;
const unsigned long GETUP_PERIOD_US = 20000;
const float GETUP_DONE_TILT_RAD = 0.25f;
const unsigned long GETUP_DONE_HOLD_MS = 700;

unsigned long lastPolicyUs = 0;
unsigned long uprightStartMs = 0;

// 显示刷新控制
unsigned long lastDisplayMs = 0;
const unsigned long DISPLAY_PERIOD_MS = 100;  // 10Hz 刷新

// WebSocket 发送控制
unsigned long lastWsMs = 0;
const unsigned long WS_PERIOD_MS = 50;  // 20Hz

// 按键状态
bool lastBtnState = false;
unsigned long btnPressMs = 0;
const unsigned long BTN_DEBOUNCE_MS = 50;

// ==================== 工具函数 ====================
float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// ==================== RGB LED ====================
void updateLed() {
  // 根据模式设置 RGB LED 颜色
  switch (robotMode) {
    case MODE_GETUP:
      ledR = 128; ledG = 0; ledB = 128;  // 紫
      break;
    case MODE_GETUP_DONE:
      ledR = 0; ledG = 128; ledB = 0;    // 绿
      break;
    case MODE_GETUP_ABORT:
      ledR = 128; ledG = 0; ledB = 0;    // 红
      break;
    default:
      if (policyPaused) {
        ledR = 128; ledG = 128; ledB = 0;  // 黄 - 暂停
      } else if (autoGetupEnabled) {
        ledR = 0; ledG = 128; ledB = 128;  // 青 - AUTO ON
      } else {
        ledR = 64; ledG = 64; ledB = 64;   // 灰 - AUTO OFF
      }
      break;
  }
  M5.Led.setAllColor(ledR, ledG, ledB);
  M5.Led.display();
}

// ==================== 舵机控制 ====================
// pulse1/pulse2 are half-span microseconds around calibrated center.
// Nominal endpoints remain the existing 500..2500 us; physical angle calibration pending.
void writeServoTargetsRad(float target1, float target2) {
  target1 = clampf(target1, -TARGET1_LIMIT_RAD, TARGET1_LIMIT_RAD);
  target2 = clampf(target2, -TARGET2_LIMIT_RAD, TARGET2_LIMIT_RAD);
  int us1 = (int)roundf(1500.0f + offset1 - target1 / TARGET1_LIMIT_RAD * pulse1);
  int us2 = (int)roundf(1500.0f + offset2 - target2 / TARGET2_LIMIT_RAD * pulse2);
  servo1.writeMicroseconds(us1);
  servo2.writeMicroseconds(us2);
}

// ==================== 模式控制 ====================
void startGetupMode() {
  noInterrupts();
  autoGetupEnabled = true;
  // Preserve commanded positions when resuming; no forced zero jump.
  robotMode = MODE_GETUP;
  policyPaused = false;
  interrupts();

  uprightStartMs = 0;
  lastPolicyUs = 0;
  writeServoTargetsRad(servo1_target, servo2_target);
}

void stopGetupMode(RobotMode nextMode = MODE_MANUAL) {
  noInterrupts();
  robotMode = nextMode;
  interrupts();
}

const char *modeName() {
  switch (robotMode) {
    case MODE_GETUP: return "GETUP";
    case MODE_GETUP_DONE: return "DONE";
    case MODE_GETUP_ABORT: return "ABORT";
    default: return "STAND";
  }
}

// ==================== 自动起身 ====================
void updateAutoGetup() {
  if (!autoGetupEnabled || policyPaused) return;
  // Both GETUP and DONE run the policy; DONE is only a display status.
  if (robotMode != MODE_GETUP && robotMode != MODE_GETUP_DONE) startGetupMode();
}

// ==================== 策略推理 ====================
void runGetupPolicy() {
  unsigned long nowUs = micros();
  if (lastPolicyUs != 0 && nowUs - lastPolicyUs < GETUP_PERIOD_US) {
    return;
  }
  lastPolicyUs = nowUs;

  float obs[OBS_DIM] = {
    roll,
    pitch,
    servo1_target,
    servo2_target,
  };
  float action[ACTION_DIM];
  forward_policy(obs, action);

  action[0] = clampf(action[0], -1.0f, 1.0f);
  action[1] = clampf(action[1], -1.0f, 1.0f);

  servo1_target = clampf(servo1_target + action[0] * TARGET_DELTA_RAD,
                         -TARGET1_LIMIT_RAD, TARGET1_LIMIT_RAD);
  servo2_target = clampf(servo2_target + action[1] * TARGET_DELTA_RAD,
                         -TARGET2_LIMIT_RAD, TARGET2_LIMIT_RAD);

  writeServoTargetsRad(servo1_target, servo2_target);

  unsigned long nowMs = millis();
  bool upright = fabsf(roll) < GETUP_DONE_TILT_RAD && fabsf(pitch) < GETUP_DONE_TILT_RAD;
  if (upright) {
    if (uprightStartMs == 0) uprightStartMs = nowMs;
    if (nowMs - uprightStartMs > GETUP_DONE_HOLD_MS) {
      robotMode = MODE_GETUP_DONE;
    }
  } else {
    uprightStartMs = 0;
    robotMode = MODE_GETUP;
  }
}

// ==================== IMU ====================
void get_theta(){
  M5.Imu.getAccelData(&accX,&accY,&accZ);
  theta_X  = atan2(accY, sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  theta_Y  = -atan2(-accX, -accZ) * RAD_TO_DEG;
}

void get_gyro(){
  M5.Imu.getGyroData(&gyroX,&gyroY,&gyroZ);
  gyroX = -gyroX;
}

// ==================== 显示：Peter Griffin 表情 ====================
void drawPeterFace(int cx, int cy, uint16_t color, int expression) {
  // expression: 0=normal, 1=happy, 2=surprised, 3=angry, 4=sleeping
  // 脸
  M5.Display.fillCircle(cx, cy, 16, 0xFFE0); // 肤色
  // 眼镜框
  M5.Display.drawRect(cx - 12, cy - 6, 10, 8, color);
  M5.Display.drawRect(cx + 2, cy - 6, 10, 8, color);
  M5.Display.drawLine(cx - 2, cy - 2, cx + 2, cy - 2, color);
  // 眼睛
  if (expression == 3) { // angry
    M5.Display.fillCircle(cx - 7, cy - 2, 2, 0x0000);
    M5.Display.fillCircle(cx + 7, cy - 2, 2, 0x0000);
    // 眉毛
    M5.Display.drawLine(cx - 10, cy - 8, cx - 4, cy - 5, 0x0000);
    M5.Display.drawLine(cx + 10, cy - 8, cx + 4, cy - 5, 0x0000);
  } else if (expression == 4) { // sleeping
    M5.Display.drawLine(cx - 9, cy - 2, cx - 5, cy - 2, 0x0000);
    M5.Display.drawLine(cx + 5, cy - 2, cx + 9, cy - 2, 0x0000);
  } else {
    M5.Display.fillCircle(cx - 7, cy - 2, 2, 0x0000);
    M5.Display.fillCircle(cx + 7, cy - 2, 2, 0x0000);
  }
  // 嘴巴
  if (expression == 1) { // happy
    M5.Display.drawArc(cx, cy + 6, 6, 4, 0, 180, 0x0000);
  } else if (expression == 2) { // surprised
    M5.Display.fillCircle(cx, cy + 8, 4, 0x0000);
  } else if (expression == 3) { // angry
    M5.Display.drawArc(cx, cy + 10, 6, 4, 180, 360, 0x0000);
  } else if (expression == 4) { // sleeping
    M5.Display.drawArc(cx, cy + 8, 4, 2, 0, 180, 0x0000);
  } else { // normal
    M5.Display.drawLine(cx - 4, cy + 8, cx + 4, cy + 8, 0x0000);
  }
}

// ==================== 显示：姿态可视化 ====================
void drawPose() {
  M5.Display.fillRect(POSE_X, POSE_Y, POSE_W, POSE_H, C_BG);

  float absRoll = fabsf(roll);
  float absPitch = fabsf(pitch);
  float maxTilt = max(absRoll, absPitch);

  int cx = POSE_X + POSE_W / 2;
  int cy = POSE_Y + POSE_H / 2;

  if (robotMode == MODE_GETUP) {
    drawPeterFace(cx, cy - 4, C_GETUP, 2); // surprised
    // Continuous mode: bar shows tilt, not a countdown to stopping.
    int progress = (int)(clampf(maxTilt / (PI / 2.0f), 0.0f, 1.0f) * 100);
    M5.Display.drawRect(cx - 20, cy + 18, 40, 4, C_TEXT);
    M5.Display.fillRect(cx - 20, cy + 18, progress * 40 / 100, 4, C_GETUP);
    return;
  }

  if (robotMode == MODE_GETUP_DONE) {
    drawPeterFace(cx, cy, C_DONE, 1); // happy
    return;
  }

  if (robotMode == MODE_GETUP_ABORT) {
    drawPeterFace(cx, cy, C_ROLL_POS, 3); // angry
    return;
  }

  if (maxTilt < 0.3f) {
    drawPeterFace(cx, cy, C_UPRIGHT, 0); // normal
  } else if (absRoll > absPitch) {
    if (roll > 0) {
      drawPeterFace(cx, cy, C_ROLL_POS, 2); // surprised
    } else {
      drawPeterFace(cx, cy, C_ROLL_NEG, 2); // surprised
    }
  } else {
    if (pitch > 0) {
      drawPeterFace(cx, cy, C_PITCH_POS, 2); // surprised
    } else {
      drawPeterFace(cx, cy, C_PITCH_NEG, 4); // sleeping
    }
  }
}

// ==================== 显示：主刷新 ====================
void updateDisplay() {
  unsigned long nowMs = millis();
  if (nowMs - lastDisplayMs < DISPLAY_PERIOD_MS) {
    return;
  }
  lastDisplayMs = nowMs;

  drawPose();
  // 屏幕下方显示 IP
  M5.Display.fillRect(IP_X, IP_Y, IP_W, IP_H, C_BG);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(C_TEXT, C_BG);
  M5.Display.setCursor(IP_X + 10, IP_Y + 5);
  M5.Display.print("192.168.42.1");
}

// ==================== 按键处理 ====================
void checkButton() {
  bool btnState = M5.BtnA.isPressed();
  unsigned long nowMs = millis();

  if (btnState && !lastBtnState) {
    btnPressMs = nowMs;
  } else if (!btnState && lastBtnState) {
    if (nowMs - btnPressMs > BTN_DEBOUNCE_MS) {
      policyPaused = !policyPaused;
      if (policyPaused) {
        // Pause holds the last command; do not snap to center.
        stopGetupMode(MODE_MANUAL);
      } else {
        startGetupMode();
      }
    }
  }
  lastBtnState = btnState;
}

// ==================== 数据记录 ====================
void recordData() {
  unsigned long nowMs = millis();
  if (nowMs - lastDataMs < DATA_PERIOD_MS) {
    return;
  }
  lastDataMs = nowMs;

  rollHistory[historyIdx] = roll;
  pitchHistory[historyIdx] = pitch;
  servo1History[historyIdx] = servo1_target;
  servo2History[historyIdx] = servo2_target;
  historyIdx = (historyIdx + 1) % HISTORY_LEN;
}

// ==================== Web 页面 ====================
void handleRoot() {
  String temp = "<!DOCTYPE html>\n<html lang=\"zh-CN\">";
  temp += "<head>";
  temp += "<meta charset=\"utf-8\">";
  temp += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  temp += "<title>勃勃生机</title>";
  temp += "<style>";
  temp += "*{box-sizing:border-box;margin:0;padding:0}";
  temp += "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:#1a1a2e;color:#eee;min-height:100vh;padding:20px}";
  temp += ".container{max-width:500px;margin:0 auto}";
  temp += "h1{font-size:1.5rem;margin-bottom:20px;text-align:center;color:#0ff}";
  temp += ".card{background:#16213e;border-radius:12px;padding:16px;margin-bottom:16px;box-shadow:0 4px 6px rgba(0,0,0,0.3)}";
  temp += ".card-title{font-size:0.9rem;color:#888;margin-bottom:12px;text-transform:uppercase;letter-spacing:1px}";
  temp += ".value{font-size:1.5rem;font-weight:bold;text-align:center;padding:8px;border-radius:8px;background:#0f3460;margin:4px 0}";
  temp += ".btn-group{display:flex;gap:8px;flex-wrap:wrap;justify-content:center;margin:8px 0}";
  temp += ".btn{flex:1;min-width:80px;padding:12px 16px;border:none;border-radius:8px;font-size:1rem;font-weight:bold;cursor:pointer;transition:all 0.2s;text-decoration:none;display:inline-block;text-align:center;color:#fff}";
  temp += ".btn-primary{background:#0f3460;border:2px solid #0ff}";
  temp += ".btn-danger{background:#ff6b6b}";
  temp += ".btn-success{background:#4ecdc4;color:#000}";
  temp += ".btn-warning{background:#ffe66d;color:#000}";
  temp += ".status{display:inline-block;padding:4px 12px;border-radius:20px;font-size:0.85rem;font-weight:bold}";
  temp += ".status.on{background:#4ecdc4;color:#000}";
  temp += ".status.off{background:#666;color:#fff}";
  temp += ".grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}";
  temp += ".servo-bar{height:24px;background:#333;border-radius:12px;overflow:hidden;margin:4px 0;position:relative;display:flex;justify-content:center}";
  temp += ".servo-fill{height:100%;border-radius:12px;transition:width 0.1s;position:absolute}";
  temp += ".servo-fill.s1{background:linear-gradient(90deg,#ff6b6b,#ff8e53)}";
  temp += ".servo-fill.s2{background:linear-gradient(90deg,#4ecdc4,#44a08d)}";
  temp += ".servo-center{position:absolute;left:50%;top:0;width:2px;height:100%;background:#fff;opacity:0.5;transform:translateX(-50%)}";
  temp += "canvas{width:100%;height:150px;background:#0f3460;border-radius:8px;margin:8px 0}";
  temp += ".adv-card{background:#0d1b2a;border:1px solid #333}";
  temp += "input[type=number]{width:80px;padding:8px;border-radius:6px;border:1px solid #333;background:#1a1a2e;color:#fff;text-align:center}";
  temp += ".param-row{display:flex;justify-content:space-between;align-items:center;margin:8px 0;padding:8px;background:#16213e;border-radius:8px}";
  temp += ".param-label{color:#888}";
  temp += ".param-value{color:#0ff;font-weight:bold}";
  temp += ".led-indicator{width:20px;height:20px;border-radius:50%;display:inline-block;margin-right:8px;vertical-align:middle}";
  temp += ".switch{position:relative;display:inline-block;width:50px;height:24px}";
  temp += ".switch input{opacity:0;width:0;height:0}";
  temp += ".slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background-color:#333;transition:.3s;border-radius:24px}";
  temp += ".slider:before{position:absolute;content:'';height:18px;width:18px;left:3px;bottom:3px;background-color:#fff;transition:.3s;border-radius:50%}";
  temp += "input:checked + .slider{background-color:#4ecdc4}";
  temp += "input:checked + .slider:before{transform:translateX(26px)}";
  temp += "</style>";
  temp += "</head>";
  temp += "<body>";
  temp += "<div class=\"container\">";
  temp += "<h1>勃勃生机，万物竞发</h1>";
  temp += "<p style=\"text-align:center;color:#888;margin-bottom:20px\">同济子豪兄</p>";

  // 状态卡片
  temp += "<div class=\"card\">";
  temp += "<div class=\"card-title\">状态</div>";
  temp += "<div style=\"display:flex;align-items:center;margin-bottom:8px\">";
  temp += "<span class=\"led-indicator\" id=\"ledDot\" style=\"background:#666\"></span>";
  temp += "<span id=\"modeText\">" + String(modeName()) + "</span>";
  temp += "</div>";
  temp += "<div style=\"display:flex;justify-content:space-between;align-items:center\">";
  temp += "<span>AUTO GETUP</span>";
  temp += "<span class=\"status " + String(autoGetupEnabled ? "on" : "off") + "\" id=\"autoStatus\">" + String(autoGetupEnabled ? "ON" : "OFF") + "</span>";
  temp += "</div>";
  temp += "</div>";

  // 姿态卡片
  temp += "<div class=\"card\">";
  temp += "<div class=\"card-title\">姿态 (rad)</div>";
  temp += "<div class=\"grid\">";
  temp += "<div class=\"value\" style=\"color:#ff6b6b\">Roll<br><span id=\"rollVal\">" + String(roll, 2) + "</span></div>";
  temp += "<div class=\"value\" style=\"color:#4ecdc4\">Pitch<br><span id=\"pitchVal\">" + String(pitch, 2) + "</span></div>";
  temp += "</div>";
  temp += "<div id=\"cubeContainer\" style=\"width:100%;height:200px;background:#0f3460;border-radius:8px\"></div>";
  temp += "</div>";

  // 舵机卡片
  temp += "<div class=\"card\">";
  temp += "<div class=\"card-title\">舵机目标 (rad)</div>";
  temp += "<div class=\"servo-bar\"><div class=\"servo-fill s1\" id=\"s1Bar\" style=\"width:0%\"></div><div class=\"servo-center\"></div></div>";
  temp += "<div style=\"display:flex;justify-content:space-between;font-size:0.85rem;color:#888\"><span>Servo1</span><span id=\"s1Val\">" + String(servo1_target, 2) + "</span></div>";
  temp += "<div class=\"servo-bar\"><div class=\"servo-fill s2\" id=\"s2Bar\" style=\"width:0%\"></div><div class=\"servo-center\"></div></div>";
  temp += "<div style=\"display:flex;justify-content:space-between;font-size:0.85rem;color:#888\"><span>Servo2</span><span id=\"s2Val\">" + String(servo2_target, 2) + "</span></div>";
  temp += "<canvas id=\"servoChart\"></canvas>";
  temp += "</div>";

  // 控制卡片
  temp += "<div class=\"card\">";
  temp += "<div class=\"card-title\">控制</div>";
  temp += "<div class=\"btn-group\">";
  temp += "<button onclick=\"action('getupStart')\" class=\"btn btn-success\">站立一次</button>";
  temp += "<button onclick=\"action('getupStop')\" class=\"btn btn-danger\">急停</button>";
  temp += "</div>";
  temp += "<div style=\"margin-top:12px;display:flex;align-items:center;justify-content:space-between\">";
  temp += "<span style=\"color:#888\">自动站立</span>";
  temp += "<label class=\"switch\"><input type=\"checkbox\" id=\"autoSwitch\" " + String(autoGetupEnabled ? "checked" : "") + " onchange=\"toggleAuto()\"><span class=\"slider round\"></span></label>";
  temp += "</div>";
  temp += "</div>";

  // 高级设置卡片
  temp += "<div class=\"card adv-card\">";
  temp += "<div class=\"card-title\">高级设置</div>";
  temp += "<form action=\"/setParams\" method=\"GET\">";
  temp += "<div class=\"param-row\"><span class=\"param-label\">ang1</span><input type=\"number\" name=\"ang1\" value=\"" + String(ang1) + "\" min=\"-120\" max=\"120\"></div>";
  temp += "<div class=\"param-row\"><span class=\"param-label\">offset1</span><input type=\"number\" name=\"offset1\" value=\"" + String(offset1) + "\" min=\"-300\" max=\"300\"></div>";
  temp += "<div class=\"param-row\"><span class=\"param-label\">pulse1</span><input type=\"number\" name=\"pulse1\" value=\"" + String(pulse1) + "\" min=\"0\" max=\"1200\"></div>";
  temp += "<div class=\"param-row\"><span class=\"param-label\">ang2</span><input type=\"number\" name=\"ang2\" value=\"" + String(ang2) + "\" min=\"-120\" max=\"120\"></div>";
  temp += "<div class=\"param-row\"><span class=\"param-label\">offset2</span><input type=\"number\" name=\"offset2\" value=\"" + String(offset2) + "\" min=\"-300\" max=\"300\"></div>";
  temp += "<div class=\"param-row\"><span class=\"param-label\">pulse2</span><input type=\"number\" name=\"pulse2\" value=\"" + String(pulse2) + "\" min=\"0\" max=\"1200\"></div>";
  temp += "<div class=\"btn-group\" style=\"margin-top:12px\">";
  temp += "<button type=\"submit\" class=\"btn btn-warning\">保存参数</button>";
  temp += "</div>";
  temp += "</form>";
  temp += "</div>";

  temp += "</div>";

  // JavaScript
  temp += "<script>";
  temp += "let rollData = [], pitchData = [], s1Data = [], s2Data = [];";
  temp += "const maxLen = 50;";
  temp += "function fetchData() {";
  temp += "  fetch('/data').then(r => r.json()).then(d => {";
  temp += "    document.getElementById('rollVal').textContent = d.roll.toFixed(2);";
  temp += "    document.getElementById('pitchVal').textContent = d.pitch.toFixed(2);";
  temp += "    document.getElementById('s1Val').textContent = d.s1.toFixed(2);";
  temp += "    document.getElementById('s2Val').textContent = d.s2.toFixed(2);";
  temp += "    document.getElementById('modeText').textContent = d.mode;";
  temp += "    document.getElementById('autoSwitch').checked = d.auto;";
  temp += "    const ledColor = 'rgb(' + d.led[0] + ',' + d.led[1] + ',' + d.led[2] + ')';";
  temp += "    document.getElementById('ledDot').style.background = ledColor;";
  temp += "    document.getElementById('s1Bar').style.width = Math.abs(d.s1) / 1.55 * 50 + '%';";
  temp += "    document.getElementById('s1Bar').style.left = d.s1 >= 0 ? '50%' : (50 - Math.abs(d.s1) / 1.55 * 50) + '%';";
  temp += "    document.getElementById('s2Bar').style.width = Math.abs(d.s2) / 1.55 * 50 + '%';";
  temp += "    document.getElementById('s2Bar').style.left = d.s2 >= 0 ? '50%' : (50 - Math.abs(d.s2) / 1.55 * 50) + '%';";
  temp += "    rollData.push(d.roll); pitchData.push(d.pitch);";
  temp += "    s1Data.push(d.s1); s2Data.push(d.s2);";
  temp += "    if (rollData.length > maxLen) { rollData.shift(); pitchData.shift(); s1Data.shift(); s2Data.shift(); }";
  temp += "    drawCube(d.roll, d.pitch);";
  temp += "    drawChart('servoChart', s1Data, s2Data, '#ff8e53', '#44a08d', 'S1', 'S2');";
  temp += "  }).catch(e => console.log(e));";
  temp += "}";
  temp += "function action(cmd) { fetch('/' + cmd).then(() => fetchData()); }";
  temp += "function toggleAuto() {";
  temp += "  const on = document.getElementById('autoSwitch').checked;";
  temp += "  fetch(on ? '/autoOn' : '/autoOff').then(() => fetchData());";
  temp += "}";
  temp += "setInterval(fetchData, 50);";
  temp += "function drawCube(roll, pitch) {";
  temp += "  const container = document.getElementById('cubeContainer');";
  temp += "  container.innerHTML = '';";
  temp += "  const canvas = document.createElement('canvas');";
  temp += "  canvas.width = container.offsetWidth;";
  temp += "  canvas.height = container.offsetHeight;";
  temp += "  container.appendChild(canvas);";
  temp += "  const ctx = canvas.getContext('2d');";
  temp += "  const w = canvas.width, h = canvas.height;";
  temp += "  const cx = w / 2, cy = h / 2;";
  temp += "  const size = 50;";
  temp += "  ctx.clearRect(0, 0, w, h);";
  temp += "  // 3D projection";
  temp += "  const cosR = Math.cos(roll), sinR = Math.sin(roll);";
  temp += "  const cosP = Math.cos(pitch), sinP = Math.sin(pitch);";
  temp += "  const vertices = [[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]];";
  temp += "  const edges = [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]];";
  temp += "  const projected = vertices.map(v => {";
  temp += "    let x = v[0], y = v[1], z = v[2];";
  temp += "    // Rotate around X (pitch)";
  temp += "    let y1 = y * cosP - z * sinP;";
  temp += "    let z1 = y * sinP + z * cosP;";
  temp += "    // Rotate around Y (roll)";
  temp += "    let x2 = x * cosR + z1 * sinR;";
  temp += "    let z2 = -x * sinR + z1 * cosR;";
  temp += "    // Perspective projection";
  temp += "    const scale = 300 / (300 + z2 * 50);";
  temp += "    return [cx + x2 * size * scale, cy + y1 * size * scale];";
  temp += "  });";
  temp += "  ctx.strokeStyle = '#4ecdc4';";
  temp += "  ctx.lineWidth = 2;";
  temp += "  edges.forEach(e => {";
  temp += "    ctx.beginPath();";
  temp += "    ctx.moveTo(projected[e[0]][0], projected[e[0]][1]);";
  temp += "    ctx.lineTo(projected[e[1]][0], projected[e[1]][1]);";
  temp += "    ctx.stroke();";
  temp += "  });";
  temp += "  ctx.fillStyle = '#fff';";
  temp += "  ctx.font = '12px sans-serif';";
  temp += "  ctx.fillText('Roll: ' + roll.toFixed(2), 10, 20);";
  temp += "  ctx.fillText('Pitch: ' + pitch.toFixed(2), 10, 36);";
  temp += "}";
  temp += "function drawChart(id, d1, d2, c1, c2, l1, l2) {";
  temp += "  const canvas = document.getElementById(id);";
  temp += "  const ctx = canvas.getContext('2d');";
  temp += "  canvas.width = canvas.offsetWidth;";
  temp += "  canvas.height = canvas.offsetHeight;";
  temp += "  const w = canvas.width, h = canvas.height;";
  temp += "  ctx.clearRect(0, 0, w, h);";
  temp += "  const min = Math.min(...d1, ...d2, -1.6);";
  temp += "  const max = Math.max(...d1, ...d2, 1.6);";
  temp += "  const range = max - min || 1;";
  temp += "  const y = v => h - (v - min) / range * h;";
  temp += "  const x = i => i / (maxLen - 1) * w;";
  temp += "  ctx.strokeStyle = c1; ctx.lineWidth = 2; ctx.beginPath();";
  temp += "  d1.forEach((v, i) => { if (i === 0) ctx.moveTo(x(i), y(v)); else ctx.lineTo(x(i), y(v)); });";
  temp += "  ctx.stroke();";
  temp += "  ctx.strokeStyle = c2; ctx.beginPath();";
  temp += "  d2.forEach((v, i) => { if (i === 0) ctx.moveTo(x(i), y(v)); else ctx.lineTo(x(i), y(v)); });";
  temp += "  ctx.stroke();";
  temp += "  ctx.fillStyle = c1; ctx.font = '10px sans-serif'; ctx.fillText(l1, 5, 12);";
  temp += "  ctx.fillStyle = c2; ctx.fillText(l2, 5, 24);";
  temp += "}";
  temp += "</script>";
  temp += "</body></html>";
  server.send(200, "text/HTML", temp);
}

// ==================== JSON 数据接口 ====================
void handleData() {
  String json = "{";
  json += "\"policyVersion\":\"" + String(POLICY_VERSION) + "\",\"roll\":" + String(roll, 3);
  json += ",\"pitch\":" + String(pitch, 3);
  json += ",\"s1\":" + String(servo1_target, 3);
  json += ",\"s2\":" + String(servo2_target, 3);
  json += ",\"mode\":\"" + String(modeName()) + "\"";
  json += ",\"auto\":" + String(autoGetupEnabled ? "true" : "false");
  json += ",\"paused\":" + String(policyPaused ? "true" : "false");
  json += ",\"led\":[" + String(ledR) + "," + String(ledG) + "," + String(ledB) + "]";
  json += "}";
  server.send(200, "application/json", json);
}

// ==================== 参数保存 ====================
void setParamsHttp() {
  // An explicit manual angle request is the only path that enables manual output.
  if (server.hasArg("ang1") || server.hasArg("ang2")) {
    autoGetupEnabled = false;
    policyPaused = false;
    stopGetupMode(MODE_MANUAL);
  }
  if (server.hasArg("ang1")) ang1 = server.arg("ang1").toInt();
  if (server.hasArg("offset1")) {
    offset1 = server.arg("offset1").toInt();
    preferences.putInt("offset1", offset1);
  }
  if (server.hasArg("pulse1")) {
    pulse1 = server.arg("pulse1").toInt();
    preferences.putInt("pulse1", pulse1);
  }
  if (server.hasArg("ang2")) ang2 = server.arg("ang2").toInt();
  if (server.hasArg("offset2")) {
    offset2 = server.arg("offset2").toInt();
    preferences.putInt("offset2", offset2);
  }
  if (server.hasArg("pulse2")) {
    pulse2 = server.arg("pulse2").toInt();
    preferences.putInt("pulse2", pulse2);
  }

  // 重新 attach 舵机
  servo1.detach();
  servo1.attach(servo1_pin, 1500 + offset1 - pulse1, 1500 + offset1 + pulse1);
  servo2.detach();
  servo2.attach(servo2_pin, 1500 + offset2 - pulse2, 1500 + offset2 + pulse2);

  handleRoot();
}

// ==================== HTTP 处理函数 ====================
void getupStartHttp() {
  policyPaused = false;
  startGetupMode();
  server.send(200, "text/plain", "OK");
}
void getupStopHttp() {
  autoGetupEnabled = false;
  policyPaused = true;
  stopGetupMode(MODE_MANUAL);
  server.send(200, "text/plain", "OK");
}
void autoOnHttp() {
  startGetupMode();
  server.send(200, "text/plain", "OK");
}
void autoOffHttp() {
  autoGetupEnabled = false;
  policyPaused = true;
  stopGetupMode(MODE_MANUAL);
  server.send(200, "text/plain", "OK");
}

// ==================== Setup ====================
void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  cfg.internal_imu = true;
  cfg.fallback_board = m5::board_t::board_M5AtomS3R;
  M5.begin(cfg);

  // 初始化显示
  M5.Display.setRotation(0);
  M5.Display.setBrightness(128);
  M5.Display.fillScreen(C_BG);
  M5.Display.setTextColor(C_TEXT, C_BG);

  // 显示启动画面（2秒）
  M5.Display.setTextSize(1);
  M5.Display.setCursor(10, 30);
  M5.Display.print("Self-Rising Robot");
  M5.Display.setCursor(10, 45);
  M5.Display.print("V3.1 HomeMadeGarbage");
  M5.Display.setCursor(10, 60);
  M5.Display.print("TommyZihao Feisuo");
  delay(2000);
  M5.Display.fillScreen(C_BG);

  // 检查 IMU
  if (!M5.Imu.isEnabled()) {
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 60);
    M5.Display.print("IMU ERROR!");
    while(1) delay(100);
  }

  // 轴序：S3R 固定映射
  M5.Imu.setAxisOrder(m5::IMU_Class::axis_x_neg,
                     m5::IMU_Class::axis_y_pos,
                     m5::IMU_Class::axis_z_neg);

  // 舵机初始化（用户要求 pulse 范围可调大，±120°）
  servo1.setTimerWidth(14);
  servo2.setTimerWidth(14);
  servo1.attach(servo1_pin, 1500 + offset1 - pulse1, 1500 + offset1 + pulse1);
  servo2.attach(servo2_pin, 1500 + offset2 - pulse2, 1500 + offset2 + pulse2);

  // 读取保存的参数
  preferences.begin("robo", false);
  offset1 = preferences.getInt("offset1", 0);
  pulse1 = preferences.getInt("pulse1", 1000);
  offset2 = preferences.getInt("offset2", 0);
  pulse2 = preferences.getInt("pulse2", 1000);
  preferences.end();

  // WiFi AP（无密码）
  WiFi.softAP(ssid);
  WiFi.softAPConfig(ip, ip, subnet);

  // DNS 劫持：所有域名都重定向到本机，实现 Captive Portal 自动弹窗
  dnsServer.start(53, "*", ip);

  // HTTP 路由
  server.on("/", handleRoot);
  server.on("/getupStart", getupStartHttp);
  server.on("/getupStop", getupStopHttp);
  server.on("/autoOn", autoOnHttp);
  server.on("/autoOff", autoOffHttp);
  server.on("/setParams", setParamsHttp);
  server.on("/data", handleData);
  server.begin();

  // Kalman 初始化
  get_theta();
  kalmanX.setAngle(theta_X);
  kalmanY.setAngle(theta_Y);

  Serial.printf("robo03 S3R Zihao: board=%d IMU=%d\n",
                M5.getBoard(), M5.Imu.isEnabled());
  Serial.printf("Policy: %s\n", POLICY_VERSION);
  Serial.printf("Servo pins: %d, %d\n", servo1_pin, servo2_pin);
  oldTime = micros();
}

// ==================== Loop ====================
void loop() {
  M5.update();

  // 按键检测
  checkButton();

  // IMU 读取
  get_theta();
  get_gyro();

  // 时间计算
  nowTime = micros();
  loopTime = nowTime - oldTime;
  oldTime = nowTime;
  dt = loopTime / 1000000.0f;

  // Kalman 滤波
  kalAngleX = kalmanX.getAngle(theta_X, gyroX, dt);
  kalAngleY = kalmanY.getAngle(theta_Y, gyroY, dt);
  kalAngleDotX = kalmanX.getRate();
  kalAngleDotY = kalmanY.getRate();
  pitch = kalAngleX * DEG_TO_RAD;
  roll = -kalAngleY * DEG_TO_RAD;

  // 自动起身检测
  updateAutoGetup();

  // 策略执行（GETUP 模式且未暂停）
  if (autoGetupEnabled && !policyPaused) {
    runGetupPolicy();
  } else if (robotMode == MODE_MANUAL && !policyPaused) {
    // Explicit manual control; synchronize the policy's next observation.
    servo1_target = clampf(ang1 * DEG_TO_RAD, -TARGET1_LIMIT_RAD, TARGET1_LIMIT_RAD);
    servo2_target = clampf(ang2 * DEG_TO_RAD, -TARGET2_LIMIT_RAD, TARGET2_LIMIT_RAD);
    writeServoTargetsRad(servo1_target, servo2_target);
  }

  // RGB LED 更新
  updateLed();

  // 显示刷新
  updateDisplay();

  // 数据记录
  recordData();

  // HTTP 服务
  server.handleClient();
  dnsServer.processNextRequest();

  // 非阻塞延时：GETUP 模式全速跑，其他模式轻微让步
  if (robotMode != MODE_GETUP) {
    delay(1);
  }
}
