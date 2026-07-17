#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

includn
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "model.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

/* WIFI + MQTT */
const char* ssid = "Tenchiduoc";
const char* password = "trungdeptrai";

const char* mqtt_server = "la4fdf16.ala.us-east-1.emqxsl.com";
const int mqtt_port = 8883;

const char* mqtt_user = "esp32";
const char* mqtt_pass = "12345678";
const char* topic = "esp32/gps_imu";

WiFiClientSecure espClient;
PubSubClient client(espClient);

/* GPS */
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

/* MPU6050 */
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68

/* CÔNG TẮC / NÚT NHẤN */
#define THEFT_SWITCH_PIN 4
#define SOS_BUTTON_PIN   5

/* GPS FILTER */
const double GPS_ALPHA = 0.2;
const int MIN_SAT = 5;
bool gpsFilteredReady = false;
double filteredLat = 0;
double filteredLng = 0;

/* MODE HỆ THỐNG */
enum SystemMode {
  MODE_NORMAL = 0,
  MODE_ANTI_THEFT = 1
};

/* TRẠNG THÁI CẢNH BÁO */
enum AlertState {
  NONE  = 1,
  CRASH = 2,
  LOST1 = 3,
  LOST2 = 4,
  SOS   = 5
};

struct GPSData {
  double lat;
  double lng;
  int sat;
  bool valid;
};

struct IMUData {
  float ax;
  float ay;
  float az;
  float accTotal;
  float accChange;
  bool crash;
};

struct ButtonData {
  bool antiTheft;
  bool sos;
};

struct SystemData {
  SystemMode mode;
  AlertState alertState;

  double lat;
  double lng;
  int sat;

  float ax;
  float ay;
  float az;
  float accTotal;
  float accChange;

  bool antiTheft;
  double lockLat;
  double lockLng;
  double distance;
  int lostLevel;
  bool sos;
  bool crash;
};

QueueHandle_t queueGPS;
QueueHandle_t queueIMU;
QueueHandle_t queueButton;
QueueHandle_t queueMQTT;
SemaphoreHandle_t serialMutex;
SemaphoreHandle_t mqttMutex;

/* TINYML - MPU6050 WINDOW MODEL */
#define WINDOW_SIZE 20
#define FEATURE_SIZE 5

float imuWindow[WINDOW_SIZE][FEATURE_SIZE];
int imuIndex = 0;
bool imuWindowReady = false;

// Thay đúng theo scaler.mean_ và scaler.scale_ từ Colab nếu cần.
float scalerMean[FEATURE_SIZE] = {0.02075613,0.03541806,0.71290792,1.22031479,0.39152778};
float scalerStd[FEATURE_SIZE]  = {0.77457809,0.65052061,0.49680159,0.53918334,0.53583921};

constexpr int kTensorArenaSize = 20 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

const tflite::Model* aiModel = nullptr;
tflite::MicroInterpreter* aiInterpreter = nullptr;
TfLiteTensor* aiInput = nullptr;
TfLiteTensor* aiOutput = nullptr;

void safePrint(const String &s) {
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.print(s);
    xSemaphoreGive(serialMutex);
  }
}

void safePrintln(const String &s) {
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.println(s);
    xSemaphoreGive(serialMutex);
  }
}

/* WIFI */
void setup_wifi() {
  WiFi.begin(ssid, password);
  Serial.print("Dang ket noi WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Da ket noi WiFi");
}

/* MQTT */
void reconnect() {
  static unsigned long lastReconnectAttempt = 0;

  if (client.connected()) {
    return;
  }

  if (millis() - lastReconnectAttempt < 3000) {
    return;
  }

  lastReconnectAttempt = millis();

  Serial.print("Dang ket noi MQTT...");

  String clientId = "ESP32-" + String(random(0xffff), HEX);

  if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
    Serial.println("Thanh cong");
  } else {
    Serial.print("That bai, rc=");
    Serial.println(client.state());
  }
}

/* MPU6050 */
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t read16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 2);
  if (Wire.available() >= 2) {
    return (Wire.read() << 8) | Wire.read();
  }
  return 0;
}

/* TINYML SETUP */
void setupTinyML() {
  aiModel = tflite::GetModel(model);

  static tflite::MicroMutableOpResolver<10> resolver;
  resolver.AddConv2D();
  resolver.AddMaxPool2D();
  resolver.AddMean();
  resolver.AddFullyConnected();
  resolver.AddRelu();
  resolver.AddLogistic();
  resolver.AddReshape();
  resolver.AddExpandDims();
  resolver.AddSqueeze();

  static tflite::MicroInterpreter staticInterpreter(
    aiModel,
    resolver,
    tensor_arena,
    kTensorArenaSize
  );

  aiInterpreter = &staticInterpreter;

  if (aiInterpreter->AllocateTensors() != kTfLiteOk) {
    Serial.println("TinyML AllocateTensors failed");
    while (1);
  }

  aiInput = aiInterpreter->input(0);
  aiOutput = aiInterpreter->output(0);
  Serial.println("TinyML window model ready");
}

float normalizeFeature(float value, int featureIndex) {
  if (scalerStd[featureIndex] == 0) return 0;
  return (value - scalerMean[featureIndex]) / scalerStd[featureIndex];
}

bool predictAccidentWindow() {
  if (!imuWindowReady) return false;

  int inputIndex = 0;

  for (int step = 0; step < WINDOW_SIZE; step++) {
    int realIndex = (imuIndex + step) % WINDOW_SIZE;

    for (int feature = 0; feature < FEATURE_SIZE; feature++) {
      float rawValue = imuWindow[realIndex][feature];
      float scaledValue = normalizeFeature(rawValue, feature);
      aiInput->data.f[inputIndex++] = scaledValue;
    }
  }

  if (aiInterpreter->Invoke() != kTfLiteOk) {
    safePrintln("TinyML Invoke failed");
    return false;
  }

  float result = aiOutput->data.f[0];
  return result > 0.5;
}

/* TÍNH KHOẢNG CÁCH GPS BẰNG HAVERSINE */
double calcDistance(double lat1, double lng1, double lat2, double lng2) {
  const double R = 6371000.0;
  double phi1 = lat1 * PI / 180.0;
  double phi2 = lat2 * PI / 180.0;
  double dPhi = (lat2 - lat1) * PI / 180.0;
  double dLambda = (lng2 - lng1) * PI / 180.0;

  double a = sin(dPhi / 2) * sin(dPhi / 2) +
             cos(phi1) * cos(phi2) *
             sin(dLambda / 2) * sin(dLambda / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

void TaskGPS(void *pv) {
  GPSData gpsData = {0, 0, 0, false};

  while (1) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }

    gpsData.sat = gps.satellites.value();

    if (gps.location.isValid() && gpsData.sat >= MIN_SAT) {
      double newLat = gps.location.lat();
      double newLng = gps.location.lng();

      if (!gpsFilteredReady) {
        filteredLat = newLat;
        filteredLng = newLng;
        gpsFilteredReady = true;
      } else {
        filteredLat = GPS_ALPHA * newLat + (1 - GPS_ALPHA) * filteredLat;
        filteredLng = GPS_ALPHA * newLng + (1 - GPS_ALPHA) * filteredLng;
      }

      gpsData.lat = filteredLat;
      gpsData.lng = filteredLng;
      gpsData.valid = true;
    } else {
      gpsData.valid = false;
    }

    xQueueOverwrite(queueGPS, &gpsData);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void TaskIMU(void *pv) {
  IMUData imuData = {0, 0, 0, 0, 0, false};

  static float lastAcc_g = 0;
  static bool firstRead = true;
  int crashCount = 0;
  unsigned long lastCrashTime = 0;
  const unsigned long CRASH_HOLD_TIME = 5000;

  while (1) {
    int16_t ax_raw = read16(0x3B);
    int16_t ay_raw = read16(0x3D);
    int16_t az_raw = read16(0x3F);

    float ax_g = ax_raw / 16384.0;
    float ay_g = ay_raw / 16384.0;
    float az_g = az_raw / 16384.0;

    float accTotal_g = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    float accChange_g = 0;

    if (firstRead) {
      lastAcc_g = accTotal_g;
      accChange_g = 0;
      firstRead = false;
    } else {
      accChange_g = abs(accTotal_g - lastAcc_g);
      lastAcc_g = accTotal_g;
    }

    imuData.ax = ax_g * 9.81;
    imuData.ay = ay_g * 9.81;
    imuData.az = az_g * 9.81;
    imuData.accTotal = accTotal_g * 9.81;
    imuData.accChange = accChange_g * 9.81;

    imuWindow[imuIndex][0] = ax_g;
    imuWindow[imuIndex][1] = ay_g;
    imuWindow[imuIndex][2] = az_g;
    imuWindow[imuIndex][3] = accTotal_g;
    imuWindow[imuIndex][4] = accChange_g;

    imuIndex++;
    if (imuIndex >= WINDOW_SIZE) {
      imuIndex = 0;
      imuWindowReady = true;
    }

    bool crashCondition = predictAccidentWindow();

    if (crashCondition) {
      crashCount++;
    } else {
      crashCount = 0;
    }

    if (crashCount >= 2) {
      imuData.crash = true;
      lastCrashTime = millis();
    }

    if (imuData.crash && !crashCondition && millis() - lastCrashTime > CRASH_HOLD_TIME) {
      imuData.crash = false;
    }

    xQueueOverwrite(queueIMU, &imuData);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void TaskButton(void *pv) {
  ButtonData buttonData = {false, false};
  bool lastAntiTheft = false;
  bool lastSosButton = false;

  while (1) {
    bool currentAntiTheft = (digitalRead(THEFT_SWITCH_PIN) == LOW);
    bool currentSosButton = (digitalRead(SOS_BUTTON_PIN) == LOW);

    if (currentAntiTheft != lastAntiTheft) {
      buttonData.antiTheft = currentAntiTheft;
      safePrintln(buttonData.antiTheft ? "BAT CHE DO CHONG TROM" : "TAT CHE DO CHONG TROM");
      lastAntiTheft = currentAntiTheft;
    }

    if (currentSosButton && !lastSosButton) {
      buttonData.sos = !buttonData.sos;
      safePrintln(buttonData.sos ? "SOS DUOC KICH HOAT" : "SOS DA TAT");
      vTaskDelay(pdMS_TO_TICKS(200));
    }

    lastSosButton = currentSosButton;
    xQueueOverwrite(queueButton, &buttonData);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void TaskProcess(void *pv) {
  GPSData gpsData = {0, 0, 0, false};
  IMUData imuData = {0, 0, 0, 0, 0, false};
  ButtonData buttonData = {false, false};
  SystemData sysData;

  bool lastAntiTheft = false;
  double lockLat = 0;
  double lockLng = 0;
  double distanceD = 0;
  int lostLevel = 0;

  while (1) {
    xQueueReceive(queueButton, &buttonData, 0);
    xQueueReceive(queueIMU, &imuData, 0);
    xQueueReceive(queueGPS, &gpsData, 0);

    if (buttonData.antiTheft != lastAntiTheft) {
      if (buttonData.antiTheft) {
        lockLat = gpsData.lat;
        lockLng = gpsData.lng;
        distanceD = 0;
        lostLevel = 0;

        safePrint("Luu vi tri khoa L1: ");
        safePrint(String(lockLat, 6));
        safePrint(", ");
        safePrintln(String(lockLng, 6));
      } else {
        distanceD = 0;
        lostLevel = 0;
      }
      lastAntiTheft = buttonData.antiTheft;
    }

    if (buttonData.antiTheft) {
      if (gpsData.valid && gpsData.lat != 0 && gpsData.lng != 0 && lockLat != 0 && lockLng != 0) {
        distanceD = calcDistance(lockLat, lockLng, gpsData.lat, gpsData.lng);

        if (distanceD < 10) {
          lostLevel = 0;
        } else if (distanceD >= 10 && distanceD < 50) {
          lostLevel = 1;
        } else {
          lostLevel = 2;
        }
      }
    } else {
      distanceD = 0;
      lostLevel = 0;
    }

    AlertState alertState = NONE;
    if (buttonData.sos) {
      alertState = SOS;
    } else if (imuData.crash) {
      alertState = CRASH;
    } else if (buttonData.antiTheft && lostLevel == 2) {
      alertState = LOST2;
    } else if (buttonData.antiTheft && lostLevel == 1) {
      alertState = LOST1;
    } else {
      alertState = NONE;
    }

    sysData.mode = buttonData.antiTheft ? MODE_ANTI_THEFT : MODE_NORMAL;
    sysData.alertState = alertState;
    sysData.lat = gpsData.lat;
    sysData.lng = gpsData.lng;
    sysData.sat = gpsData.sat;
    sysData.ax = imuData.ax;
    sysData.ay = imuData.ay;
    sysData.az = imuData.az;
    sysData.accTotal = imuData.accTotal;
    sysData.accChange = imuData.accChange;
    sysData.antiTheft = buttonData.antiTheft;
    sysData.lockLat = lockLat;
    sysData.lockLng = lockLng;
    sysData.distance = distanceD;
    sysData.lostLevel = lostLevel;
    sysData.sos = buttonData.sos;
    sysData.crash = imuData.crash;

    xQueueOverwrite(queueMQTT, &sysData);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void TaskMQTT(void *pv) {
  SystemData sysData;
  unsigned long lastSend = 0;

  while (1) {
    if (mqttMutex != NULL && xSemaphoreTake(mqttMutex, portMAX_DELAY) == pdTRUE) {
      if (!client.connected()) {
        reconnect();
      }
      client.loop();
      xSemaphoreGive(mqttMutex);
    }

    if (xQueueReceive(queueMQTT, &sysData, 0) == pdTRUE) {
      if (millis() - lastSend > 1000) {
        lastSend = millis();

        char payload[650];
        snprintf(payload, sizeof(payload),
                 "{"
                 "\"mode\":%d,"
                 "\"alertState\":%d,"
                 "\"lat\":%.6f,"
                 "\"lng\":%.6f,"
                 "\"sat\":%d,"
                 "\"ax\":%.2f,"
                 "\"ay\":%.2f,"
                 "\"az\":%.2f,"
                 "\"accTotal\":%.2f,"
                 "\"accChange\":%.2f,"
                 "\"antiTheft\":%d,"
                 "\"lockLat\":%.6f,"
                 "\"lockLng\":%.6f,"
                 "\"distance\":%.2f,"
                 "\"lostLevel\":%d,"
                 "\"sos\":%d,"
                 "\"crash\":%d"
                 "}",
                 (int)sysData.mode,
                 (int)sysData.alertState,
                 sysData.lat,
                 sysData.lng,
                 sysData.sat,
                 sysData.ax,
                 sysData.ay,
                 sysData.az,
                 sysData.accTotal,
                 sysData.accChange,
                 sysData.antiTheft,
                 sysData.lockLat,
                 sysData.lockLng,
                 sysData.distance,
                 sysData.lostLevel,
                 sysData.sos,
                 sysData.crash);

        if (mqttMutex != NULL && xSemaphoreTake(mqttMutex, portMAX_DELAY) == pdTRUE) {
          client.publish(topic, payload);
          xSemaphoreGive(mqttMutex);
        }
        safePrintln(payload);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(THEFT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);

  serialMutex = xSemaphoreCreateMutex();
  mqttMutex = xSemaphoreCreateMutex();

  queueGPS = xQueueCreate(1, sizeof(GPSData));
  queueIMU = xQueueCreate(1, sizeof(IMUData));
  queueButton = xQueueCreate(1, sizeof(ButtonData));
  queueMQTT = xQueueCreate(1, sizeof(SystemData));

  if (queueGPS == NULL || queueIMU == NULL || queueButton == NULL || queueMQTT == NULL ||
      serialMutex == NULL || mqttMutex == NULL) {
    Serial.println("LOI TAO QUEUE/MUTEX");
    while (1);
  }

  setup_wifi();
  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Wire.begin(SDA_PIN, SCL_PIN);
  writeReg(0x6B, 0x00);

  setupTinyML();
z
  xTaskCreatePinnedToCore(TaskGPS,     "TaskGPS",     4096,  NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskIMU,     "TaskIMU",     12288, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskButton,  "TaskButton",  2048,  NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskProcess, "TaskProcess", 4096,  NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(TaskMQTT,    "TaskMQTT",    6144,  NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
