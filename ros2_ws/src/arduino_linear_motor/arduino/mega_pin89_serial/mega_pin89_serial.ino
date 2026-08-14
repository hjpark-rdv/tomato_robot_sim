// Arduino Mega:
// - Direct digital control of pins 8 and 9 over USB serial.
// - Servo positioning on pin 10 with an optional speed percentage:
//   "ANGLE 10 <10..173> [1..100]". 100% (or omitted) moves immediately;
//   1..99% moves at a software-limited angular rate.
// Digital commands: "PIN 8 0", "PIN 8 1", "PIN 9 0", or "PIN 9 1".

#include <Servo.h>

constexpr uint8_t PIN_8 = 8;
constexpr uint8_t PIN_9 = 9;
constexpr uint8_t SERVO_PIN = 10;
constexpr float SERVO_MAX_SPEED_DEG_PER_SEC = 180.0f;

String input;
Servo pin10Servo;
bool servoAttached = false;
bool servoMoving = false;
float servoCurrentAngleDeg = 90.0f;
float servoTargetAngleDeg = 90.0f;
float servoSpeedDegPerSec = 0.0f;
unsigned long servoLastUpdateMs = 0;

void allLow() {
  digitalWrite(PIN_8, LOW);
  digitalWrite(PIN_9, LOW);
}

void setup() {
  pinMode(PIN_8, OUTPUT);
  pinMode(PIN_9, OUTPUT);
  allLow();  // Safe output state after reset or USB reconnect.

  Serial.begin(115200);
  Serial.println("READY PIN8 PIN9 SERVO10_MANUAL");
}

void commandServo(int angleDeg, int speedPercent) {
  // The first command establishes the only trustworthy software position.
  // Write it before attach so the first PWM pulse already has the target.
  if (!servoAttached) {
    pin10Servo.write(angleDeg);
    pin10Servo.attach(SERVO_PIN);
    servoAttached = true;
    servoCurrentAngleDeg = angleDeg;
    servoTargetAngleDeg = angleDeg;
    servoMoving = false;
    return;
  }

  servoTargetAngleDeg = angleDeg;
  if (speedPercent >= 100) {
    pin10Servo.write(angleDeg);
    servoCurrentAngleDeg = angleDeg;
    servoMoving = false;
    return;
  }
  servoSpeedDegPerSec =
      SERVO_MAX_SPEED_DEG_PER_SEC * static_cast<float>(speedPercent) / 100.0f;
  servoLastUpdateMs = millis();
  servoMoving = true;
}

void updateServoMotion() {
  if (!servoAttached || !servoMoving) return;
  unsigned long now = millis();
  unsigned long elapsedMs = now - servoLastUpdateMs;
  if (elapsedMs == 0) return;
  servoLastUpdateMs = now;

  float remaining = servoTargetAngleDeg - servoCurrentAngleDeg;
  float maximumStep = servoSpeedDegPerSec * elapsedMs / 1000.0f;
  if (abs(remaining) <= maximumStep) {
    servoCurrentAngleDeg = servoTargetAngleDeg;
    servoMoving = false;
  } else {
    servoCurrentAngleDeg += remaining > 0.0f ? maximumStep : -maximumStep;
  }
  pin10Servo.write(static_cast<int>(round(servoCurrentAngleDeg)));
}

void handleCommand(const String& command) {
  int pin = -1;
  int value = -1;
  int servoPin = -1;
  int angleDeg = -1;
  int speedPercent = 100;
  if (sscanf(command.c_str(), "PIN %d %d", &pin, &value) == 2 &&
      (pin == PIN_8 || pin == PIN_9) && (value == 0 || value == 1)) {
    digitalWrite(pin, value == 1 ? HIGH : LOW);
    Serial.print("OK PIN ");
    Serial.print(pin);
    Serial.print(" ");
    Serial.println(value);
  } else if (sscanf(command.c_str(), "ANGLE %d %d %d", &servoPin, &angleDeg,
                    &speedPercent) >= 2 &&
             servoPin == SERVO_PIN && angleDeg >= 10 && angleDeg <= 173 &&
             speedPercent >= 1 && speedPercent <= 100) {
    commandServo(angleDeg, speedPercent);
    Serial.print("OK ANGLE 10 ");
    Serial.print(angleDeg);
    Serial.print(" SPEED ");
    Serial.println(speedPercent);
  } else {
    Serial.println(
        "ERR expected: PIN <8|9> <0|1> or ANGLE 10 <10..173> [1..100]");
  }
}

void loop() {
  while (Serial.available()) {
    char received = static_cast<char>(Serial.read());
    if (received == '\n') {
      input.trim();
      if (input.length() > 0) handleCommand(input);
      input = "";
    } else if (received != '\r') {
      input += received;
      if (input.length() > 48) input = "";  // Reject malformed overlong input.
    }
  }
  updateServoMotion();
}
