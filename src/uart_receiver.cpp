#include <HardwareSerial.h>

HardwareSerial RaspiSerial(1); // define a Serial for UART1 aka the RaspiSerial
const int RaspiSerialRX = 16;
const int RaspiSerialTX = 17;

void setup() {
  Serial.begin(115200);
  delay(2000);

  // initialize the RaspiSerial to the pins
  RaspiSerial.begin(11500, SERIAL_8N1, RaspiSerialRX, RaspiSerialTX);

  Serial.println("ESP32 ready");
}

void loop() {
  // here we could use our RaspiSerial normally
  while (RaspiSerial.available() > 0) {
    String msg = RaspiSerial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(msg);
  }
}
