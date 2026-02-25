/*
  sensor_test.ino
  Tests MPU-6050 (accelerometer) over I2C and a flex sensor over ADC.

  Wiring:
    MPU-6050:
      GPIO 22 -> SCL
      GPIO 21 -> SDA
      GND     -> ADO  (sets I2C address to 0x68)
      3.3V    -> VCC
      GND     -> GND

    Flex sensor (voltage divider, 10kOhm pull-down):
      3.3V -> [flex sensor] -> GPIO 34 -> [10kOhm] -> GND
*/

#include <Wire.h>

// ── MPU-6050 ──────────────────────────────────────────────────────────────────
#define MPU_ADDR      0x68   // ADO=GND → 0x68
#define REG_WHO_AM_I  0x75
#define REG_PWR_MGMT  0x6B
#define REG_ACCEL_X_H 0x3B   // accel X/Y/Z are 6 consecutive bytes from here

// ── Flex sensor ───────────────────────────────────────────────────────────────
#define FLEX_PIN      34     // GPIO 34: input-only, good ADC pin
#define ADC_MAX       4095   // ESP32 ADC is 12-bit
#define VCC           3.3f

// 10kOhm pull-down resistor in the voltage divider
#define R_DIVIDER     10000.0f

// ─────────────────────────────────────────────────────────────────────────────

// Write one byte to an MPU register
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Read `len` bytes starting at `reg` into `buf`
void mpuRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);          // keep bus active
  Wire.requestFrom((uint8_t)MPU_ADDR, len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== sensor_test booting ===");

  // Start I2C on the correct pins for this wiring
  Wire.begin(21, 22);   // SDA=21, SCL=22

  // ── Check MPU-6050 is responding ──
  Wire.beginTransmission(MPU_ADDR);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.printf("ERROR: No I2C device found at 0x%02X (Wire error %d)\n",
                  MPU_ADDR, err);
    Serial.println("Check wiring. Halting.");
    while (true) delay(1000);
  }

  // ── Verify WHO_AM_I register (should be 0x68) ──
  uint8_t whoami = 0;
  mpuRead(REG_WHO_AM_I, &whoami, 1);
  Serial.printf("WHO_AM_I = 0x%02X (expected 0x68)\n", whoami);
  if (whoami != 0x68) {
    Serial.println("ERROR: Unexpected WHO_AM_I. Check chip / address. Halting.");
    while (true) delay(1000);
  }

  // ── Wake the MPU-6050 (clears sleep bit) ──
  mpuWrite(REG_PWR_MGMT, 0x00);
  delay(100);

  Serial.println("MPU-6050 OK — starting continuous read loop.");
  Serial.println("-------------------------------------------");
  Serial.println("AccelX(g)  AccelY(g)  AccelZ(g)  | Flex(raw)  Flex(V)  FlexR(Ohm)");
  Serial.println("-------------------------------------------");
}

// ─────────────────────────────────────────────────────────────────────────────

void loop() {
  // ── Read raw accelerometer data (6 bytes: XH,XL,YH,YL,ZH,ZL) ──
  uint8_t raw[6];
  mpuRead(REG_ACCEL_X_H, raw, 6);

  int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t az = (int16_t)((raw[4] << 8) | raw[5]);

  // Default full-scale range is ±2g → 16384 LSB/g
  float gx = ax / 16384.0f;
  float gy = ay / 16384.0f;
  float gz = az / 16384.0f;

  // ── Read flex sensor ──
  int flexRaw = analogRead(FLEX_PIN);
  float flexV = (flexRaw / (float)ADC_MAX) * VCC;
  // Voltage divider: V_out = VCC * R_divider / (R_flex + R_divider)
  // → R_flex = R_divider * (VCC/V_out - 1)
  float flexR = (flexV > 0.01f)
                  ? R_DIVIDER * (VCC / flexV - 1.0f)
                  : 99999.0f;   // guard divide-by-zero

  // ── Print ──
  Serial.printf("% 8.3f   % 8.3f   % 8.3f   |  %4d    %5.3fV   %7.0f Ohm\n",
                gx, gy, gz, flexRaw, flexV, flexR);

  delay(100);   // 10 Hz print rate — adjust as needed
}
