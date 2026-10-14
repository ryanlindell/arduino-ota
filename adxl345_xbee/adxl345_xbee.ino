/*
 * adxl345_xbee.ino - Read an ADXL345 accelerometer and send it out the
 * hardware serial port, where the XBee (transparent mode) radios it to the
 * coordinator.
 *
 * Wiring (Uno):
 *   ADXL345 VCC -> 3V3      ADXL345 GND -> GND
 *   ADXL345 SDA -> A4       ADXL345 SCL -> A5
 *   ADXL345 CS  -> 3V3 (selects I2C)
 *   ADXL345 SDO -> GND (I2C address 0x53)
 *
 * Output: one line per sample, "x,y,z" in milli-g (1000 = 1 g).
 */

#include <Wire.h>

const uint8_t ADXL345_ADDR = 0x53;
const uint8_t REG_DEVID = 0x00;        // always reads 0xE5
const uint8_t REG_POWER_CTL = 0x2D;
const uint8_t REG_DATA_FORMAT = 0x31;
const uint8_t REG_DATAX0 = 0x32;       // first of 6 data bytes: X, Y, Z (low, high)

const unsigned long SAMPLE_PERIOD_MS = 100;   // 10 samples per second

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDR, (uint8_t)1);
  return Wire.read();
}

void setup() {
  // Must match the XBee's BD setting (and the bootloader, for OTA uploads).
  Serial.begin(115200);
  Wire.begin();

  if (readRegister(REG_DEVID) != 0xE5) {
    Serial.println("ERROR: ADXL345 not found, check wiring");
  }
  writeRegister(REG_DATA_FORMAT, 0x08);   // full resolution, +/-2 g: 3.9 mg per count
  writeRegister(REG_POWER_CTL, 0x08);     // start measuring
}

void loop() {
  static unsigned long lastSample = 0;
  if (millis() - lastSample < SAMPLE_PERIOD_MS) {
    return;
  }
  lastSample = millis();

  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(REG_DATAX0);
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDR, (uint8_t)6);
  if (Wire.available() < 6) {
    Serial.println("ERROR: no data from ADXL345");
    return;
  }

  int16_t raw[3];
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t low = Wire.read();
    uint8_t high = Wire.read();
    raw[i] = (int16_t)((high << 8) | low);
  }

  // 3.9 mg per count, done in integers: counts * 39 / 10.
  Serial.print((long)raw[0] * 39 / 10);
  Serial.print(',');
  Serial.print((long)raw[1] * 39 / 10);
  Serial.print(',');
  Serial.println((long)raw[2] * 39 / 10);
}
