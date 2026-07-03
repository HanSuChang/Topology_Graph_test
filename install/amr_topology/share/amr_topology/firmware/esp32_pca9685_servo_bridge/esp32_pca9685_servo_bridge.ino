#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

#define SDA_PIN 21
#define SCL_PIN 22

#define SERVO_FREQ 50
#define SERVOMIN_DEFAULT 102
#define SERVOMAX_DEFAULT 512

#define SERVO_COUNT 5
#define SERVO_STEP_INTERVAL_MS 10
#define SERVO_STEP_DEG 1
#define LINE_BUFFER_SIZE 64
#define COMMAND_ACK_ENABLED 0

int servoChannels[SERVO_COUNT] = {0, 4, 8, 12, 15};
int angles[SERVO_COUNT] = {38, 70, 90, 15, 50};
int targetAngles[SERVO_COUNT] = {38, 70, 90, 15, 50};

char lineBuffer[LINE_BUFFER_SIZE];
size_t lineLength = 0;
unsigned long lastServoStepMs = 0;

uint16_t angleToPulse(int channel, int angle) {
  (void)channel;
  angle = constrain(angle, 0, 180);

  return map(angle, 0, 180, SERVOMIN_DEFAULT, SERVOMAX_DEFAULT);
}

void setServoAngle(int channel, int angle) {
  angle = constrain(angle, 0, 180);
  pca9685.setPWM(channel, 0, angleToPulse(channel, angle));
}

void updateServosTowardLatestTarget() {
  unsigned long now = millis();
  if (now - lastServoStepMs < SERVO_STEP_INTERVAL_MS) {
    return;
  }

  lastServoStepMs = now;

  for (int i = 0; i < SERVO_COUNT; i++) {
    int currentAngle = angles[i];
    int targetAngle = targetAngles[i];

    if (currentAngle < targetAngle) {
      currentAngle += min(SERVO_STEP_DEG, targetAngle - currentAngle);
    } else if (currentAngle > targetAngle) {
      currentAngle -= min(SERVO_STEP_DEG, currentAngle - targetAngle);
    } else {
      continue;
    }

    angles[i] = currentAngle;
    setServoAngle(servoChannels[i], currentAngle);
  }
}

bool parseAngleToken(const char *start, const char *end, int &angle) {
  if (start == end) {
    return false;
  }

  int value = 0;
  for (const char *p = start; p < end; p++) {
    if (*p < '0' || *p > '9') {
      return false;
    }

    value = value * 10 + (*p - '0');
    if (value > 180) {
      return false;
    }
  }

  angle = value;
  return true;
}

bool parseServoCommand(char *command, int outputAngles[]) {
  char *cursor = command;

  for (int i = 0; i < SERVO_COUNT; i++) {
    char *tokenStart = cursor;
    char *tokenEnd = cursor;

    while (*tokenEnd != '\0' && *tokenEnd != ',') {
      tokenEnd++;
    }

    if (!parseAngleToken(tokenStart, tokenEnd, outputAngles[i])) {
      return false;
    }

    if (i < SERVO_COUNT - 1) {
      if (*tokenEnd != ',') {
        return false;
      }
      cursor = tokenEnd + 1;
    } else if (*tokenEnd != '\0') {
      return false;
    }
  }

  return true;
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

void printTargetAngles() {
  Serial.print("OK TARGET ");

  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.print(targetAngles[i]);

    if (i < SERVO_COUNT - 1) {
      Serial.print(",");
    }
  }

  Serial.println();
}

void processCommand(char *command) {
  int parsedAngles[SERVO_COUNT];

  if (!parseServoCommand(command, parsedAngles)) {
    Serial.print("ERR Invalid command: ");
    Serial.println(command);
    Serial.println("FORMAT: angle0,angle1,angle2,angle3,angle4");
    return;
  }

  for (int i = 0; i < SERVO_COUNT; i++) {
    targetAngles[i] = constrain(parsedAngles[i], 0, 180);
  }

  if (COMMAND_ACK_ENABLED) {
    printTargetAngles();
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  pca9685.begin();
  pca9685.setPWMFreq(SERVO_FREQ);

  delay(500);

  Serial.println("ESP32 + PCA9685 Ready");
  Serial.println("Protocol: angle0,angle1,angle2,angle3,angle4");
  Serial.println("Example: 38,70,90,15,50");
  Serial.println("Order: CH0,CH4,CH8,CH12,CH15");

  for (int i = 0; i < SERVO_COUNT; i++) {
    targetAngles[i] = angles[i];
    setServoAngle(servoChannels[i], angles[i]);
    delay(100);
  }

  printCurrentAngles();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      lineBuffer[lineLength] = '\0';
      processCommand(lineBuffer);
      lineLength = 0;
    } else if (c != '\r') {
      if (lineLength < LINE_BUFFER_SIZE - 1) {
        lineBuffer[lineLength++] = c;
      } else {
        lineLength = 0;
        Serial.println("ERR Line too long");
      }
    }
  }

  updateServosTowardLatestTarget();
}
