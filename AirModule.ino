#include <Wire.h>

const byte FS3000_ADDR = 0x28; // I2C address

#define BUFFER_SIZE 20
uint16_t buffer[BUFFER_SIZE];
int bufferIndex = 0;
float Vel_kph;

void setup() {
  Serial.begin(9600);
  Wire.begin(); // A4 = SDA, A5 = SCL

  // Initialize buffer with zeros
  for (int i = 0; i < BUFFER_SIZE; i++) {
    buffer[i] = 0;
  }
}

void loop() {
  // Request 3 bytes: checksum + 2 data bytes
  Wire.requestFrom((uint8_t)FS3000_ADDR, (uint8_t)3);

  if (Wire.available() == 3) {
    Wire.read();  // checksum (ignored here)
    uint8_t highByte = Wire.read();  // only low 4 bits valid
    uint8_t lowByte  = Wire.read();

    // Combine into 12-bit raw value
    uint16_t raw = ((highByte & 0x0F) << 8) | lowByte;

    // Save reading into buffer
    buffer[bufferIndex] = raw;
    bufferIndex++;

    // If we have 20 samples → average & print
    if (bufferIndex >= BUFFER_SIZE) {
      bufferIndex = 0;

      uint32_t sum = 0;
      for (int i = 0; i < BUFFER_SIZE; i++) {
        sum += buffer[i];
      }
      uint16_t averageRaw = sum / BUFFER_SIZE;

      //Serial.print("Average Raw (20 samples): ");
     //Serial.print(averageRaw); Serial.print(" ");

      // converter para velocidade
      if (averageRaw>2400) { 
        Vel_kph=(0.0029*averageRaw-3.295);
      }
      else {
        Vel_kph=(0.0018*averageRaw-0.06812);
      }
      //Serial.print(averageRaw);   Serial.print(" ");
      Serial.println(Vel_kph);
    }
  }

  delay(5); // sample every 5 ms
}


