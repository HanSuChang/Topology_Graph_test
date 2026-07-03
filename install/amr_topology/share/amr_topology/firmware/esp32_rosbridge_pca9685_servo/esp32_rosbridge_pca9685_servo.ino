#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Change these for your network and PC running rosbridge_websocket.
const char * WIFI_SSID = "hansu";
const char * WIFI_PASSWORD = "06040604";
const char * ROSBRIDGE_HOST = "172.20.10.12";
const uint16_t ROSBRIDGE_PORT = 9090;

const char * SERVO_TOPIC = "/arm_servo_angles";
const char * STATUS_TOPIC = "/esp32_servo_status";
const char * FEEDBACK_TOPIC = "/esp32_servo_feedback";

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
WebSocketsClient webSocket;

#define SDA_PIN 21
#define SCL_PIN 22

#define SERVO_FREQ 50
#define SERVOMIN_DEFAULT 102
#define SERVOMAX_DEFAULT 512
#define SERVOMIN_CH8 150
#define SERVOMAX_CH8 600

#define SERVO_COUNT 5
#define STEP_DELAY 30
#define STATUS_LED_PIN 2

int servoChannels[SERVO_COUNT] = {0, 4, 8, 12, 15};
int angles[SERVO_COUNT] = {90, 90, 90, 90, 90};
bool rosbridgeConnected = false;
bool statusLedState = false;
unsigned long lastStatusLedToggleMs = 0;

void setStatusLed(bool enabled) {
  statusLedState = enabled;
  digitalWrite(STATUS_LED_PIN, enabled ? HIGH : LOW);
}

void updateStatusLed() {
  if (rosbridgeConnected) {
    setStatusLed(true);
    return;
  }

  unsigned long now = millis();
  unsigned long intervalMs = WiFi.status() == WL_CONNECTED ? 700 : 150;

  if (now - lastStatusLedToggleMs >= intervalMs) {
    lastStatusLedToggleMs = now;
    setStatusLed(!statusLedState);
  }
}

uint16_t angleToPulse(int channel, int angle) {
  angle = constrain(angle, 0, 180);

  if (channel == 8) {
    return map(angle, 0, 180, SERVOMIN_CH8, SERVOMAX_CH8);
  }

  return map(angle, 0, 180, SERVOMIN_DEFAULT, SERVOMAX_DEFAULT);
}

void setServoAngle(int channel, int angle) {
  angle = constrain(angle, 0, 180);
  pca9685.setPWM(channel, 0, angleToPulse(channel, angle));
}

void moveAllServos(int targetAngles[]) {
  int maxDelta = 0;

  for (int i = 0; i < SERVO_COUNT; i++) {
    targetAngles[i] = constrain(targetAngles[i], 0, 180);
    int delta = abs(targetAngles[i] - angles[i]);
    if (delta > maxDelta) {
      maxDelta = delta;
    }
  }

  for (int step = 1; step <= maxDelta; step++) {
    for (int i = 0; i < SERVO_COUNT; i++) {
      int currentAngle = angles[i];
      int targetAngle = targetAngles[i];

      if (currentAngle < targetAngle) {
        currentAngle++;
      } else if (currentAngle > targetAngle) {
        currentAngle--;
      }

      angles[i] = currentAngle;
      setServoAngle(servoChannels[i], currentAngle);
    }

    delay(STEP_DELAY);
  }
}

void printCurrentAngles() {
  Serial.print("OK ");

  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.print("CH");
    Serial.print(servoChannels[i]);
    Serial.print("=");
    Serial.print(angles[i]);

    if (i < SERVO_COUNT - 1) {
      Serial.print(",");
    }
  }

  Serial.println();
}

void sendJson(JsonDocument &doc) {
  String payload;
  serializeJson(doc, payload);
  webSocket.sendTXT(payload);
}

void advertiseTopic(const char * topic, const char * type) {
  StaticJsonDocument<256> doc;
  doc["op"] = "advertise";
  doc["topic"] = topic;
  doc["type"] = type;
  sendJson(doc);
}

void publishStatus(const String &status) {
  StaticJsonDocument<384> doc;
  doc["op"] = "publish";
  doc["topic"] = STATUS_TOPIC;
  doc["msg"]["data"] = status;
  sendJson(doc);
}

void publishFeedback() {
  StaticJsonDocument<512> doc;
  doc["op"] = "publish";
  doc["topic"] = FEEDBACK_TOPIC;

  JsonArray data = doc["msg"]["data"].to<JsonArray>();
  for (int i = 0; i < SERVO_COUNT; i++) {
    data.add(angles[i]);
  }

  sendJson(doc);
}

void advertiseDebugTopics() {
  advertiseTopic(STATUS_TOPIC, "std_msgs/String");
  advertiseTopic(FEEDBACK_TOPIC, "std_msgs/Int32MultiArray");
}

void subscribeServoTopic() {
  StaticJsonDocument<256> doc;
  doc["op"] = "subscribe";
  doc["topic"] = SERVO_TOPIC;
  doc["type"] = "std_msgs/Int32MultiArray";
  doc["throttle_rate"] = 0;
  doc["queue_length"] = 1;

  sendJson(doc);

  Serial.print("Subscribed: ");
  Serial.println(SERVO_TOPIC);
  publishStatus(String("subscribed ") + SERVO_TOPIC);
}

bool parseServoAngles(JsonVariant data, int targetAngles[]) {
  if (!data.is<JsonArray>()) {
    return false;
  }

  JsonArray array = data.as<JsonArray>();
  if (array.size() != SERVO_COUNT) {
    return false;
  }

  for (int i = 0; i < SERVO_COUNT; i++) {
    int angle = array[i].as<int>();
    if (angle < 0 || angle > 180) {
      return false;
    }
    targetAngles[i] = angle;
  }

  return true;
}

void handleRosbridgeMessage(uint8_t * payload, size_t length) {
  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.print("ERR JSON: ");
    Serial.println(error.c_str());
    publishStatus(String("json error: ") + error.c_str());
    return;
  }

  const char * op = doc["op"] | "";
  const char * topic = doc["topic"] | "";

  if (strcmp(op, "publish") != 0 || strcmp(topic, SERVO_TOPIC) != 0) {
    return;
  }

  int targetAngles[SERVO_COUNT];
  if (!parseServoAngles(doc["msg"]["data"], targetAngles)) {
    Serial.println("ERR Invalid /arm_servo_angles message");
    publishStatus("invalid /arm_servo_angles message");
    return;
  }

  Serial.print("RX ");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.print(targetAngles[i]);
    if (i < SERVO_COUNT - 1) {
      Serial.print(",");
    }
  }
  Serial.println();

  moveAllServos(targetAngles);
  printCurrentAngles();
  publishStatus("received /arm_servo_angles");
  publishFeedback();
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      rosbridgeConnected = false;
      Serial.println("rosbridge disconnected");
      break;

    case WStype_CONNECTED:
      rosbridgeConnected = true;
      Serial.print("rosbridge connected: ws://");
      Serial.print(ROSBRIDGE_HOST);
      Serial.print(":");
      Serial.println(ROSBRIDGE_PORT);
      advertiseDebugTopics();
      publishStatus(
        String("rosbridge connected, esp32 ip=") + WiFi.localIP().toString()
      );
      publishFeedback();
      subscribeServoTopic();
      break;

    case WStype_TEXT:
      handleRosbridgeMessage(payload, length);
      break;

    default:
      break;
  }
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    setStatusLed(!statusLedState);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("WiFi connected, IP=");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  pinMode(STATUS_LED_PIN, OUTPUT);
  setStatusLed(false);

  Wire.begin(SDA_PIN, SCL_PIN);
  pca9685.begin();
  pca9685.setPWMFreq(SERVO_FREQ);

  delay(500);

  for (int i = 0; i < SERVO_COUNT; i++) {
    setServoAngle(servoChannels[i], angles[i]);
    delay(100);
  }
  printCurrentAngles();

  connectWiFi();

  webSocket.begin(ROSBRIDGE_HOST, ROSBRIDGE_PORT, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(2000);
}

void loop() {
  webSocket.loop();
  updateStatusLed();
}
