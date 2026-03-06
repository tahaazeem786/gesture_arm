#include <Arduino.h>
#include <Wire.h>

HardwareSerial armSerial(1);

#define ARM_RX 16
#define ARM_TX 17
#define PUSH_PIN 39

static uint16_t moveTime = 800;

// ── MPU-6050 ──────────────────────────────────────────────────────────────────
#define MPU_ADDR      0x68   // ADO=GND → 0x68
#define REG_WHO_AM_I  0x75
#define REG_PWR_MGMT  0x6B
#define REG_ACCEL_X_H 0x3B   // accel X/Y/Z are 6 consecutive bytes from here

// ── Flex sensor ───────────────────────────────────────────────────────────────
#define FLEX_PIN      36     // GPIO 36: input-only, good ADC pin
#define ADC_MAX       4095   // ESP32 ADC is 12-bit
#define VCC           3.3f

// 10kOhm pull-down resistor in the voltage divider
#define R_DIVIDER     10000.0f

int16_t pos1 = 500, pos2 = 500, pos3 = 500, pos4 = 500, pos5 = 500, pos6 = 500;
uint8_t flexADC = 0, pushADC = 0; // 0=unflexed, 1=flexe

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
  delay(100);
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


void readFlexSensor(int8_t &flexADC) {
    flexADC = analogRead(FLEX_PIN);
    float voltage = (flexADC / (float)ADC_MAX) * VCC;
    float flexResistance = (R_DIVIDER * voltage) / (VCC - voltage);
    Serial.printf("Flex Sensor: ADC=%d  Voltage=%.2fV  Resistance=%.1fΩ\n",
                flexADC, voltage, flexResistance);


    // ADC sits from 60-90 when unflexed, goes to 0 or ~140 when flexed.
}

void readPushButton(int8_t &pushADC) {
    pushADC = analogRead(PUSH_PIN);
    float voltage = (pushADC / (float)ADC_MAX) * VCC;
    Serial.printf("Push Button: ADC=%d  Voltage=%.2fV\n",
                pushADC, voltage);


    // ADC sits from 1690 when pushed
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

  // default vals x -0.13, y -0.08, z 1.01
  float gx, gy, gz;
  mpuTranslateData(gx, gy, gz);
  readFlexSensor(flexADC);
  readPushButton(pushADC);
  Serial.printf("Accel: % 8.3fX   % 8.3fY   % 8.3fZ \n",
              gx, gy, gz);
  
  
  
  if(flexADC < 90 || flexADC > 60) {
    // unflexed, control 6,5,4 with X,Y

    // X CONTROL WHEN UNFLEXED
    if(gx > 0.4) {
      pos6 = pos6 - 100;
    } else if (gx < -0.5) {
        pos6 = pos6 + 100;
    }

    // Y CONTROL WHEN UNFLEXED
    if(gy > 0.4) {
        pos4 = pos4 - 100;
        pos5 = pos5 - 100;
    } else if (gy < -0.4) {
        pos5 = pos5 + 100;
        pos4 = pos4 + 100;
    }

  } else {
    // flexed, control 2,3 with X,Y

    // X CONTROL WHEN FLEXED
    if(gx > 0.4) {
      pos2 = pos2 - 100;
    } else if (gx < -0.5) {
      pos2 = pos2 + 100;
    }

    // Y CONTROL WHEN FLEXED
    if(gy > 0.4) {
      pos3 = pos3 - 100;
    } else if (gy < -0.4) {
      pos3 = pos3 + 100;
    }
  }



  // if push button, then toggle between 1000 and 0 on id 1
  if (pushADC > 500 && pos1 < 500) {
      pos1 = 600;
  } else if (pushADC > 500 && pos1 > 500) {
      pos1 = 0;
  }

  if (pos6 > 1000) pos6 = 1000;
  if (pos6 < 0) pos6 = 0;
  if (pos5 > 1000) pos5 = 1000;
  if (pos5 < 0) pos5 = 0;
  if (pos4 > 1000) pos4 = 1000;
  if (pos4 < 0) pos4 = 0;

  // mess with pos2 and 3 tuning for claw positioning
  if (pos3 > 1000) pos3 = 1000;
  if (pos3 < 0) pos3 = 0;
  if (pos2 > 1000) pos2 = 1000;
  if (pos2 < 0) pos2 = 0;
  if (pos1 > 1000) pos1 = 1000;
  if (pos1 < 0) pos1 = 0;

  


  Serial.printf("POS: 1:%d   2:%d   3:%d   4:%d   5:%d   6:%d \n",
              pos1, pos2, pos3, pos4, pos5, pos6);
  
  sendMove((uint8_t)1, pos1);
  sendMove((uint8_t)2, pos2);
  sendMove((uint8_t)3, pos3);
  sendMove((uint8_t)4, pos4);
  sendMove((uint8_t)5, pos5);
  sendMove((uint8_t)6, pos6);
}
