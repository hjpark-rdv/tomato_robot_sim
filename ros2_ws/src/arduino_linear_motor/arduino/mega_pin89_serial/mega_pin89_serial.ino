// Arduino Mega: direct digital control of pins 8 and 9 over USB serial.
// Commands from ROS bridge: "PIN 8 0", "PIN 8 1", "PIN 9 0", or "PIN 9 1".

constexpr uint8_t PIN_8 = 8;
constexpr uint8_t PIN_9 = 9;
String input;

void allLow() {
  digitalWrite(PIN_8, LOW);
  digitalWrite(PIN_9, LOW);
}

void setup() {
  pinMode(PIN_8, OUTPUT);
  pinMode(PIN_9, OUTPUT);
  allLow();  // Safe output state after reset or USB reconnect.
  Serial.begin(115200);
  Serial.println("READY PIN8 PIN9");
}

void handleCommand(const String& command) {
  int pin = -1;
  int value = -1;
  if (sscanf(command.c_str(), "PIN %d %d", &pin, &value) == 2 &&
      (pin == PIN_8 || pin == PIN_9) && (value == 0 || value == 1)) {
    digitalWrite(pin, value == 1 ? HIGH : LOW);
    Serial.print("OK PIN ");
    Serial.print(pin);
    Serial.print(" ");
    Serial.println(value);
  } else {
    Serial.println("ERR expected: PIN <8|9> <0|1>");
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
