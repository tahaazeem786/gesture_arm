#include <Arduino.h>
#include <Wire.h>

HardwareSerial armSerial(1);

static const int ARM_RX = 16;
static const int ARM_TX = 17;

static uint16_t moveTime = 800;

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

uint16_t pos1 = 500, pos2 = 500, pos3 = 500, pos4 = 500, pos5 = 500, pos6 = 500;

// ─────────────────────────────────────────────────────────────────────────────
// SERVO CONTOL PROTOCOL:
// Protocol: 55 55 LEN CMD COUNT TIME_L TIME_H [ID POS_L POS_H]...
// Where LEN = total bytes from LEN through end (INCLUDING LEN itself).
// For count=1: LEN = 5 + 3*1 = 8  (matches your working packet)
void sendMultiMove(uint8_t count, const uint8_t* ids, const uint16_t* positions) {
  if (count == 0 || count > 6) return;

  uint8_t len = (uint8_t)(5 + 3 * count);

  uint8_t frame[2 + 1 + 1 + 1 + 2 + 6 * 3]; // header + max payload
  int idx = 0;

  frame[idx++] = 0x55;
  frame[idx++] = 0x55;
  frame[idx++] = len;
  frame[idx++] = 0x03;       // move command
  frame[idx++] = count;
  frame[idx++] = (uint8_t)(moveTime & 0xFF);
  frame[idx++] = (uint8_t)((moveTime >> 8) & 0xFF);

  for (int i = 0; i < count; i++) {
    uint16_t pos = constrain((int)positions[i], 0, 1000);
    frame[idx++] = ids[i];
    frame[idx++] = (uint8_t)(pos & 0xFF);
    frame[idx++] = (uint8_t)((pos >> 8) & 0xFF);
  }

  armSerial.write(frame, idx);
}

void sendMove(uint8_t id, uint16_t pos) {
  uint8_t ids[1] = { id };
  uint16_t positions[1] = { pos };
  sendMultiMove(1, ids, positions);
}

String readLine() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String out = buf;
      buf = "";
      out.trim();
      return out;
    }
    buf += c;
  }
  return "";
}

void resetAll() {
  uint8_t ids[6] = {1,2,3,4,5,6};
  uint16_t positions[6] = {500,500,500,500,500,500};
  Serial.println("Resetting all servos to 500 (multi)...");
  sendMultiMove(6, ids, positions);
}

void resetAllSequential() {
  Serial.println("Resetting all servos to 500 (sequential)...");
  for (uint8_t id = 1; id <= 6; id++) {
    sendMove(id, 500);
    delay(moveTime + 100);
  }
}

// ─────────────────────────────────────────────────────────────────────────────

// IMU helper functions (MPU-6050)

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);          // keep bus active
  Wire.requestFrom((uint8_t)MPU_ADDR, len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) {
    buf[i] = Wire.read();
  }
}

void mpuSetup() {
    Wire.begin(21, 22);   // SDA=21, SCL=22

    Wire.beginTransmission(MPU_ADDR);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        Serial.printf("ERROR: No I2C device found at 0x%02X (Wire error %d)\n",
                    MPU_ADDR, err);
        Serial.println("Check wiring. Halting.");
        while (true) delay(1000);
    }

    mpuWrite(REG_PWR_MGMT, 0x00);
    delay(100);
}

// Change the function to return void, but take pointers or references
void mpuTranslateData(float &gx, float &gy, float &gz) {
    uint8_t raw[6];
    mpuRead(REG_ACCEL_X_H, raw, 6);

    int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4] << 8) | raw[5]);

    gx = ax / 16384.0f;
    gy = ay / 16384.0f;
    gz = az / 16384.0f;
}



void setup() {
    
    // ARM SETUP
    Serial.begin(115200);
    delay(1500);

    armSerial.begin(9600, SERIAL_8N1, ARM_RX, ARM_TX);
    delay(200);

    Serial.println("\nLeArm Ready");
 

    // IMU SETUP
    mpuSetup();

}

void loop() {
    // get imu data
    float gx, gy, gz;
    mpuTranslateData(gx, gy, gz);
    Serial.printf("Accel: % 8.3fX   % 8.3fY   % 8.3fZ \n",
                gx, gy, gz);
    
    // translate imu data to servo positions
    // if X, then id 6 plus 100
    if(gx > 0.5) {
        pos6 = pos6 + 100;
    } else if (gx < -0.5) {
        pos6 = pos6 - 100;
    }

    // if Y, then id 5 and 4 plus 100
    if(gx > 0.5) {
        pos4 = pos4 + 100;
        pos5 = pos5 + 100;
    } else if (gx < -0.5) {
        pos5 = pos5 - 100;
        pos4 = pos4 - 100;
    }

    // if Z, then id 3 and 2 plus 100
    if(gz > 0.5) {
        pos3 = pos3 + 100;
        pos2 = pos2 + 100;
    } else if (gz < -0.5) {
        pos3 = pos3 - 100;
        pos2 = pos2 - 100;
    }

    // HOW ARE WE MAPPING CLAW??

    Serial.printf("POS: 1:%d   2:%d   3:%d   4:%d   5:%d   6:%d \n",
                pos1, pos2, pos3, pos4, pos5, pos6);
    
    sendMove((uint8_t)1, pos1);
    sendMove((uint8_t)2, pos2);
    sendMove((uint8_t)3, pos3);
    sendMove((uint8_t)4, pos4);
    sendMove((uint8_t)5, pos5);
    sendMove((uint8_t)6, pos6);
}
