#include <Arduino.h>

// const int LED_PIN = 13;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Arduino - Teensy 4.0 Demo....");
  pinMode( LED_BUILTIN, OUTPUT );
}

void loop() {
  static uint32_t state = 0;
  digitalWrite(LED_BUILTIN, state ^= 1);
  Serial.printf("[%06u] LED state: %u\n", millis(), state);
  delay(5000);
}
