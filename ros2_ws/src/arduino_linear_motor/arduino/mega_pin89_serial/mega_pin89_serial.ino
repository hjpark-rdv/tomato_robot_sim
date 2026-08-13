// Arduino Mega:
// - Direct digital control of pins 8 and 9 over USB serial.
// - Manual servo positioning on pin 10 only when an "ANGLE 10 <10..173>"
//   command is received. The servo holds the requested angle afterward.
// Digital commands: "PIN 8 0", "PIN 8 1", "PIN 9 0", or "PIN 9 1".

#include <Servo.h>

constexpr uint8_t PIN_8 = 8;
constexpr uint8_t PIN_9 = 9;
constexpr uint8_t SERVO_PIN = 10;

String input;
Servo pin10Servo;
bool servoAttached = false;

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

void handleCommand(const String& command) {
  int pin = -1;
  int value = -1;
  int servoPin = -1;
  int angleDeg = -1;
  if (sscanf(command.c_str(), "PIN %d %d", &pin, &value) == 2 &&
      (pin == PIN_8 || pin == PIN_9) && (value == 0 || value == 1)) {
    digitalWrite(pin, value == 1 ? HIGH : LOW);
    Serial.print("OK PIN ");
    Serial.print(pin);
    Serial.print(" ");
    Serial.println(value);
  } else if (sscanf(command.c_str(), "ANGLE %d %d", &servoPin, &angleDeg) == 2 &&
             servoPin == SERVO_PIN && angleDeg >= 10 && angleDeg <= 173) {
    // Store the requested pulse width before attach so the first generated
    // PWM pulse already represents the GUI-requested angle.
    pin10Servo.write(angleDeg);
    if (!servoAttached) {
      pin10Servo.attach(SERVO_PIN);
      servoAttached = true;
    }
    Serial.print("OK ANGLE 10 ");
    Serial.println(angleDeg);
  } else {
    Serial.println("ERR expected: PIN <8|9> <0|1> or ANGLE 10 <10..173>");
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
}
