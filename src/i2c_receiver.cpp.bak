#include <Arduino.h>
#include <Wire.h>

#define I2C_SLAVE_ADDRESS 0x08

void receiveEvent(int numBytes);
void requestEvent();

void setup() {
  Serial.begin(115200);
  delay(2000);

  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);
  // Wire.onRequest(requestEvent);

  Serial.println("I2C Slave Ready");
}

void loop() { 
  delay(100); 
}

void receiveEvent(int numBytes) {
    if (numBytes == 5) {
      uint8_t cmd = Wire.read();

      uint16_t distance = (Wire.read() << 8) | Wire.read();
      uint16_t angle    = (Wire.read() << 8) | Wire.read();

      // Serial.print("CMD: ");
      // Serial.print(cmd);
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.print(" | Angle: ");
      Serial.println(angle);
    }
}

// void requestEvent() { Wire.write("OK"); }
