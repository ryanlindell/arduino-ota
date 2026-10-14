/*
 * adxl345_lcd_xbee.ino - Read an ADXL345 accelerometer, send it out the
 * hardware serial port (XBee, transparent mode) and show it on a 16x2 I2C LCD.
 *
 * Needs the "LiquidCrystal I2C" library (Frank de Brabander).
 *
 * Wiring (Uno). Both devices share the I2C pins A4 (SDA) and A5 (SCL).
 *   ADXL345 VCC -> 3V3      ADXL345 GND -> GND
 *   ADXL345 SDA -> A4       ADXL345 SCL -> A5
 *   ADXL345 CS  -> 3V3 (selects I2C)
 *   ADXL345 SDO -> GND (I2C address 0x53)
 *
 *   LCD VCC -> 5V           LCD GND -> GND
 *   LCD SDA -> A4           LCD SCL -> A5
 *
 * Serial output: one line per sample, "x,y,z" in milli-g (1000 = 1 g).
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const uint8_t ADXL345_ADDR = 0x53;
const uint8_t REG_DEVID = 0x00;        // always reads 0xE5
const uint8_t REG_POWER_CTL = 0x2D;
const uint8_t REG_DATA_FORMAT = 0x31;
const uint8_t REG_DATAX0 = 0x32;       // first of 6 data bytes: X, Y, Z (low, high)

const unsigned long SAMPLE_PERIOD_MS = 100;   // 10 samples per second over the radio
const unsigned long LCD_PERIOD_MS = 250;      // the screen is slow, so refresh it less often

// LCD backpacks come with one of two addresses depending on the chip fitted.
LiquidCrystal_I2C lcd27(0x27, 16, 2);
LiquidCrystal_I2C lcd3F(0x3F, 16, 2);
LiquidCrystal_I2C *lcd = NULL;         // stays NULL if no screen is connected

bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

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

void lcdShow(const char *line1, const char *line2) {
  if (lcd == NULL) {
    return;
  }
  lcd->setCursor(0, 0);
  lcd->print(line1);
  lcd->setCursor(0, 1);
  lcd->print(line2);
}

void setup() {
  // Must match the XBee's BD setting (and the bootloader, for OTA uploads).
  Serial.begin(115200);
  Wire.begin();

  if (i2cDevicePresent(0x27)) {
    lcd = &lcd27;
  } else if (i2cDevicePresent(0x3F)) {
    lcd = &lcd3F;
  }
  if (lcd != NULL) {
    lcd->init();
    lcd->backlight();
  } else {
    Serial.println("ERROR: LCD not found, check wiring");
  }

  if (readRegister(REG_DEVID) != 0xE5) {
    Serial.println("ERROR: ADXL345 not found, check wiring");
    lcdShow("ADXL345 not     ", "found           ");
  }
  writeRegister(REG_DATA_FORMAT, 0x08);   // full resolution, +/-2 g: 3.9 mg per count
  writeRegister(REG_POWER_CTL, 0x08);     // start measuring
}

void loop() {
  static unsigned long lastSample = 0;
  static unsigned long lastLcd = 0;
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
    lcdShow("ADXL345         ", "no data         ");
    return;
  }

  long mg[3];
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t low = Wire.read();
    uint8_t high = Wire.read();
    // 3.9 mg per count, done in integers: counts * 39 / 10.
    mg[i] = (long)(int16_t)((high << 8) | low) * 39 / 10;
  }

  Serial.print(mg[0]);
  Serial.print(',');
  Serial.print(mg[1]);
  Serial.print(',');
  Serial.println(mg[2]);

  if (millis() - lastLcd >= LCD_PERIOD_MS) {
    lastLcd = millis();
    // Fixed-width fields overwrite the old digits, so no flickery clear().
    char line1[17];
    char line2[17];
    snprintf(line1, sizeof(line1), "X:%-5ld Y:%-5ld ", mg[0], mg[1]);
    snprintf(line2, sizeof(line2), "Z:%-5ld mg      ", mg[2]);
    lcdShow(line1, line2);
  }
}
