#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

const char* ssid = "MINH DUNG 3";
const char* password = "minhdung68686868";

const char* mqtt_server = "la4fdf16.ala.us-east-1.emqxsl.com";

const int mqtt_port = 8883;

const char* mqtt_user = "esp32";
const char* mqtt_pass = "12345678";

const char* topic = "esp32/gps_imu";

WiFiClientSecure espClient;
PubSubClient client(espClient);

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68

#define THEFT_SWITCH_PIN 4
#define SOS_BUTTON_PIN   5

enum SystemMode {
  MODE_NORMAL = 0,
  MODE_ANTI_THEFT = 1
};

enum AlertState {
  NONE  = 1,
  CRASH = 2,
  LOST1 = 3,
  LOST2 = 4,
  SOS   = 5
};

SystemMode mode = MODE_NORMAL;
AlertState alertState = NONE;

float gps_lat = 0;
float gps_lng = 0;
int gps_sat = 0;

float ax = 0;
float ay = 0;
float az = 0;

float acc_total = 0;
float accChange = 0;
float last_acc = 0;
bool crash = false;

int crashCount = 0;
unsigned long lastCrashTime = 0;
const unsigned long CRASH_HOLD_TIME = 5000; 

bool antiTheft = false;
bool lastAntiTheft = false;

double lockLat = 0;
double lockLng = 0;

double distanceD = 0;
int lostLevel = 0;

bool sosPressed = false;
bool lastSosButton = false;

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

void reconnect() {
  while (!client.connected()) {
    Serial.print("Dang ket noi MQTT...");

    String clientId = "ESP32-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("Thanh cong");
    } else {
      Serial.print("That bai, rc=");
      Serial.println(client.state());
      delay(2000);
      yield();
    }
  }
}

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

void checkAntiTheftSwitch() {
  antiTheft = digitalRead(THEFT_SWITCH_PIN) == LOW;

  if (antiTheft != lastAntiTheft) {
    if (antiTheft) {
      mode = MODE_ANTI_THEFT;

      lockLat = gps_lat;
      lockLng = gps_lng;

      distanceD = 0;
      lostLevel = 0;

      Serial.println("BAT CHE DO CHONG TROM");
      Serial.print("Luu vi tri khoa L1: ");
      Serial.print(lockLat, 6);
      Serial.print(", ");
      Serial.println(lockLng, 6);
    } else {
      mode = MODE_NORMAL;

      distanceD = 0;
      lostLevel = 0;

      Serial.println("TAT CHE DO CHONG TROM");
    }

    lastAntiTheft = antiTheft;
  }
}

void checkSOSButton() {
  bool currentSosButton = digitalRead(SOS_BUTTON_PIN) == LOW;

  if (currentSosButton && !lastSosButton) {
    sosPressed = !sosPressed;

    if (sosPressed) {
      Serial.println("SOS DUOC KICH HOAT");
    } else {
      Serial.println("SOS DA TAT");
    }

    delay(200);
  }

  lastSosButton = currentSosButton;
}

void lostDetAlgo() {
  if (!antiTheft) {
    distanceD = 0;
    lostLevel = 0;
    return;
  }

  if (gps_lat == 0 || gps_lng == 0 || lockLat == 0 || lockLng == 0) {
    return;
  }

  distanceD = calcDistance(lockLat, lockLng, gps_lat, gps_lng);

  if (distanceD < 10) {
    lostLevel = 0;
  }
  else if (distanceD >= 10 && distanceD < 50) {
    lostLevel = 1;
  }
  else {
    lostLevel = 2;
  }
}

void updateAlertState() {
  

  if (sosPressed) {
    alertState = SOS;
    return;
  }

  if (crash) {
    alertState = CRASH;
    return;
  }

  if (antiTheft) {
    if (lostLevel == 2) {
      alertState = LOST2;
    }
    else if (lostLevel == 1) {
      alertState = LOST1;
    }
    else {
      alertState = NONE;
    }
  } else {
    alertState = NONE;
  }
}

void TaskGPS(void *pv) {
  while (1) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }

    if (gps.location.isValid()) {
      gps_lat = gps.location.lat();
      gps_lng = gps.location.lng();
    }

    gps_sat = gps.satellites.value();

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void TaskIMU(void *pv) {
  while (1) {
    int16_t ax_raw = read16(0x3B);
    int16_t ay_raw = read16(0x3D);
    int16_t az_raw = read16(0x3F);

    ax = (ax_raw / 16384.0) * 9.81;
    ay = (ay_raw / 16384.0) * 9.81;
    az = (az_raw / 16384.0) * 9.81;

    acc_total = sqrt(ax * ax + ay * ay + az * az);

    static bool firstRead = true;

    if (firstRead) {
      last_acc = acc_total;
      accChange = 0;
      firstRead = false;
    } else {
      accChange = abs(acc_total - last_acc);
      last_acc = acc_total;
    }

    

    float crashAccThreshold;
    float crashChangeThreshold;

    if (mode == MODE_NORMAL) {
      crashAccThreshold = 0.0;
      crashChangeThreshold = 3.0;
    } else {
      crashAccThreshold = 0.0;
      crashChangeThreshold = 3.0;
    }

    
    bool crashCondition = (acc_total > crashAccThreshold && accChange > crashChangeThreshold);

    if (crashCondition) {
      crashCount++;
    } else {
      crashCount = 0;
    }

    
    if (crashCount >= 2) {
      crash = true;
      lastCrashTime = millis();
    }

    
    if (crash && !crashCondition && millis() - lastCrashTime > CRASH_HOLD_TIME) {
      crash = false;
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(THEFT_SWITCH_PIN, INPUT_PULLUP);
  pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);

  setup_wifi();

  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Wire.begin(SDA_PIN, SCL_PIN);

  
  writeReg(0x6B, 0x00);

  xTaskCreatePinnedToCore(TaskGPS, "GPS", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskIMU, "IMU", 4096, NULL, 1, NULL, 1);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  checkAntiTheftSwitch();
  checkSOSButton();
  lostDetAlgo();
  updateAlertState();

  static unsigned long lastSend = 0;

  if (millis() - lastSend > 1000) {
    lastSend = millis();

    char payload[600];

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
        "\"sos\":%d"
      "}",

      mode,
      alertState,

      gps_lat,
      gps_lng,
      gps_sat,

      ax,
      ay,
      az,
      acc_total,
      accChange,

      antiTheft,
      lockLat,
      lockLng,
      distanceD,
      lostLevel,
      sosPressed
    );

    client.publish(topic, payload);

    Serial.println(payload);
  }
}
