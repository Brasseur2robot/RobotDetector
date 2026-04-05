#include <Wire.h>

#define I2C_SLAVE_ADDRESS 0x08

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.println("I2C Slave Ready");
}

void loop() { delay(100); }

void receiveEvent(int numBytes) {
  while (Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
  }
}

void requestEvent() { Wire.write("OK"); }
