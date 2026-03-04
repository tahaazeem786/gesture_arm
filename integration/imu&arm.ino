#include <Arduino.h>
HardwareSerial armSerial(1);

static const int ARM_RX = 16;
static const int ARM_TX = 17;

static uint16_t moveTime = 800;

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

void setup() {
  Serial.begin(115200);
  delay(1500);

  armSerial.begin(9600, SERIAL_8N1, ARM_RX, ARM_TX);
  delay(200);

  Serial.println("\nLeArm Ready");
 
}

void loop() {
  String line = readLine();
  if (line.length() == 0) return;

  if (line == "reset") {
    resetAll();
    return;
  }
  if (line == "reset_seq") {
    resetAllSequential();
    return;
  }
  if (line.startsWith("t ")) {
    int ms = line.substring(2).toInt();
    ms = constrain(ms, 0, 10000);
    moveTime = (uint16_t)ms;
    Serial.print("Move time set to ");
    Serial.println(moveTime);
    return;
  }

  int sp = line.indexOf(' ');
  if (sp < 0) {
    Serial.println("Bad format. Try: 1 800 or reset");
    return;
  }

  int id = line.substring(0, sp).toInt();
  int pos = line.substring(sp + 1).toInt();
  if (id < 1 || id > 6) {
    Serial.println("Servo id must be 1..6");
    return;
  }

  sendMove((uint8_t)id, (uint16_t)pos);
}
