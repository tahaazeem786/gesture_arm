#include "Config.h"
#include "Hiwonder.hpp"
#include "Robot_arm.hpp"
#include "./src/PS2/PS2_CTL.hpp"
#include "./src/PC_BLE/PC_BLE_CTL.hpp"

Buzzer_t buzzer_obj;
Button_t key_obj;
Led_t led_obj;
LeArm_t arm;
PS2_CTL ps2;
PC_BLE_CTL pc_ble_obj;

uint8_t mode_flag = 0;
uint8_t button_2_flag = 0;

void button_change_mode(uint8_t id,  ButtonEventIDEnum event)
{
  if(event == BUTTON_EVENT_PRESSED)
  {
    if(id == 1)
    {
      if(mode_flag == 0)
      {
        // PC mode
        mode_flag = 1;
        led_obj.blink(1000, 1000, 0);
        pc_ble_obj.init(0);
      }else if(mode_flag == 1){
        // PS2 controller mode
        mode_flag = 2;
        led_obj.blink(200, 200, 0);
      }else if(mode_flag == 2){
        // Offline mode
        mode_flag = 3;
        led_obj.blink(50, 50, 0);
      }else{
        // APP mode
        mode_flag = 0;
        led_obj.blink(2000, 2000, 0);
        pc_ble_obj.init(1);
      }
      buzzer_obj.blink(1500 , 100, 100, mode_flag+1);
    }
    if(id ==2)
    {
      if(mode_flag == 3){
        button_2_flag = 1;
        buzzer_obj.blink(1500 , 200, 50, 1);
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  delay(100);
  
  Serial.println("Setup Start");
  delay(1000);
  pinMode(IO_BLE_CTL, OUTPUT);
  digitalWrite(IO_BLE_CTL, LOW);  // Set BLE control pin LOW to cut power to the Bluetooth module

  pinMode(PA4, OUTPUT);
  pinMode(PA5, OUTPUT);
  // pinMode(1, OUTPUT);
  // pinMode(3, OUTPUT);

  // initialize arm first (this sets up Serial1 and servos)
  arm.init();
  
  // Verify servo type
  uint8_t servo_type_detected = arm.get_servo_type();
  Serial.print("Servo type: ");
  Serial.println(servo_type_detected);
  
  led_obj.init(IO_LED);
  buzzer_obj.init(IO_BUZZER);
  key_obj.init();

  Serial.println("BOOT: performing startup sequence");

  // simple beep + LED blink pattern
  led_obj.blink(150, 150, 3);
  buzzer_obj.blink(1500, 120, 120, 3);
  delay(1000);

  // quick reset and small motions to verify control
  arm.reset(800);
  delay(900);
  arm.claw_set(90.0f, 600);
  delay(700);
  arm.roll_set(45.0f, 600);
  delay(700);
  arm.claw_set(0.0f, 600);
  delay(700);

  // Initialize control interfaces
  ps2.init();
  pc_ble_obj.init(1); // 0: select PC control mode

  delay(100);
  key_obj.register_callback(button_change_mode);
  led_obj.blink(2000, 2000, 0);
  if(arm.get_servo_type() == 0){
    buzzer_obj.blink(5000 , 50, 50, 1);
  }else{
    buzzer_obj.blink(5000 , 300, 50, 1);
  }
  Serial.println("begin");
  delay(1000);
}

void loop() {
  switch(mode_flag){
    case 0: // Bluetooth mode
    case 1: // PC mode
      pc_ble_obj.PC_BLE_Task(&arm , &led_obj , &buzzer_obj);
      break;
    case 2: // PS2 controller mode
      ps2.PS2_Task(&arm, &led_obj, &buzzer_obj);
      break;
    case 3: // Offline control mode
      if(button_2_flag != 0){
        arm.action_run(17,1);
        button_2_flag = 0;
      }
      break;
    default:
      break;
  }
  delay(10);
}
