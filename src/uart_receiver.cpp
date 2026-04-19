#include <HardwareSerial.h>

HardwareSerial RaspiSerial(1); // Define a Serial for UART1 aka the RaspiSerial
const int RaspiSerialRX = 16;
const int RaspiSerialTX = 17;

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Initialize the RaspiSerial to the pins
  RaspiSerial.begin(115200, SERIAL_8N1, RaspiSerialRX, RaspiSerialTX);

  Serial.println("ESP32 ready");
}

void loop() {
  // Reading RaspiSerial content and print it
  while (RaspiSerial.available() > 0) {
    String msg = RaspiSerial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(msg);
  }
}
