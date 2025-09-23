#include <Wire.h>

const byte SDP810_ADDR = 0x25; // I2C address

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }

  Wire.begin(); // A4 = SDA, A5 = SCL; no arduino nano

  // Stop any measurement (needed to change sensor measurement mode/ force it to await orders)
  Wire.beginTransmission(SDP810_ADDR);
  Wire.write(0x3F);
  Wire.write(0xF9);
  Wire.endTransmission();

  delay(20); // datasheet says wait at least 20 ms after stop

  // Start continuous differential pressure measurement (average-till-read mode)
  Wire.beginTransmission(SDP810_ADDR);
  Wire.write(0x36);
  Wire.write(0x15);
  Wire.endTransmission();
  
  delay(50); // give sensor time to start
}

void loop() {
  // Request 9 bytes like datasheet says (3 of DP; 3 of Temp; 3 of scale factor)
  Wire.requestFrom(SDP810_ADDR, (uint8_t)9);
  unsigned long timer = millis();

  if (Wire.available() == 9) {
    // Differential Pressure
    int16_t dp_raw = (Wire.read() << 8) | Wire.read();
    Wire.read(); // ignore CRC 

    // Temperature
    int16_t temp_raw = (Wire.read() << 8) | Wire.read();
    Wire.read(); // ignore CRC 

    // Scale Factor
    int16_t scale_raw = (Wire.read() << 8) | Wire.read();
    Wire.read(); // ignore CRC 

    // Convert to physical units
    float dp = (float)dp_raw / (float)scale_raw;  // [Pa]
    float tempC = (float)temp_raw / 200.0;        // [°C]

    // Ambient density correction
    float AmbientPressure=1016.5; // [mbar]
    float dpCorrected= dp*966/AmbientPressure;

    // Print results
    // Serial.print("DP: ");
    Serial.print(dpCorrected, 2); Serial.print("; ");
    // Serial.print(" Pa  |  Temp: ");
    // Serial.print(tempC, 2);
    // Serial.print(" °C  |  ");


    float AmbientPressurePa = AmbientPressure*100; // convert from mbar to Pa
    float Temperature = 25;  //ºC
    float Dens = 1.292*AmbientPressurePa*273.15/(101325*(273.15+Temperature));  // Calculate ambient density

    // Print velocity
    Serial.println(sqrt(2*dpCorrected/Dens), 2);   //Bernouli formula
    
  }

  while (millis()-timer <100){}; // output every 100 ms
}
