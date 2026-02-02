#include "./../../Hiwonder.hpp"
#include <Arduino.h>
#include "./../../Config.h"

#define BUTTON_TASK_PERIOD  ((float)30) /* Onboard button scan interval (ms) */

static void button_scan(Button_t* obj, uint8_t id)
{
    obj->bt_run[id-1].ticks_count += BUTTON_TASK_PERIOD;

    uint32_t pin = obj->read(id);
    if(pin != obj->bt_run[id-1].last_pin_raw)  { /* If two consecutive IO states differ the button is unstable; save new raw state and return */
        obj->bt_run[id-1].last_pin_raw = pin;
        return;
    }

/* Button state unchanged; state machine won't transition, return */
    if(obj->bt_run[id-1].last_pin_filtered == obj->bt_run[id-1].last_pin_raw && obj->bt_run[id-1].stage == BUTTON_STAGE_NORMAL && obj->bt_run[id-1].combin_counter == 0) { 
        return;
    }

    obj->bt_run[id-1].last_pin_filtered = obj->bt_run[id-1].last_pin_raw; /* Save new filtered button state */
    switch(obj->bt_run[id-1].stage) {
        case BUTTON_STAGE_NORMAL: {
            if(obj->bt_run[id-1].last_pin_filtered) {
                obj->event_callback(id, BUTTON_EVENT_PRESSED); /* Trigger button pressed event */
                if(obj->bt_run[id-1].ticks_count < obj->bt_run[id-1].combin_th && obj->bt_run[id-1].combin_counter > 0) { /* Combo only works when combin_counter != 0 */
                    obj->bt_run[id-1].combin_counter += 1;
                    if(obj->bt_run[id-1].combin_counter == 2) {  /* Double click callback */
                        obj->event_callback(id, BUTTON_EVENT_DOUBLE_CLICK);
                    }
                    if(obj->bt_run[id-1].combin_counter == 3) {  /* Triple click callback */
                        obj->event_callback(id, BUTTON_EVENT_TRIPLE_CLICK);
                    }
                }
                obj->bt_run[id-1].ticks_count = 0;
                obj->bt_run[id-1].stage = BUTTON_STAGE_PRESS;
            } else {
                if(obj->bt_run[id-1].ticks_count > obj->bt_run[id-1].combin_th && obj->bt_run[id-1].combin_counter != 0) {
                    obj->bt_run[id-1].combin_counter = 0;
                    obj->bt_run[id-1].ticks_count = 0;
                }
            }
            break;
		    }
        case BUTTON_STAGE_PRESS: {
            if(obj->bt_run[id-1].last_pin_filtered) {
                if(obj->bt_run[id-1].ticks_count > obj->bt_run[id-1].lp_th) { /* Exceeded long press threshold */
                    obj->event_callback(id, BUTTON_EVENT_LONGPRESS); /* Trigger long press event */
                    obj->bt_run[id-1].ticks_count = 0;
                    obj->bt_run[id-1].stage = BUTTON_STAGE_LONGPRESS; /* State transitions to long press */
                }
            } else { /* Button released */
                obj->event_callback(id, BUTTON_EVENT_RELEASE_FROM_SP); /* Trigger release from short press event */
                obj->event_callback(id, BUTTON_EVENT_CLICK);  /* Trigger click event */
                obj->bt_run[id-1].combin_counter = obj->bt_run[id-1].combin_counter == 0 ? 1 : obj->bt_run[id-1].combin_counter; /* Combo only works when combin_counter != 0 */
                obj->bt_run[id-1].stage = BUTTON_STAGE_NORMAL;
            }
            break;
		    }
        case BUTTON_STAGE_LONGPRESS: {
            if(obj->bt_run[id-1].last_pin_filtered) {
                if(obj->bt_run[id-1].ticks_count > obj->bt_run[id-1].repeat_th)  {
                    obj->event_callback(id, BUTTON_EVENT_LONGPRESS_REPEAT); /* Trigger long press repeat event */
                    obj->bt_run[id-1].ticks_count = 0; /* Reset timer for next repeat */
                }
            } else { /* Button released */
                obj->event_callback(id, BUTTON_EVENT_RELEASE_FROM_LP);  /* Trigger release from long press event */
                obj->bt_run[id-1].combin_counter = 0;                /* Long press cannot combo; disable combo counter */
                obj->bt_run[id-1].ticks_count = obj->bt_run[id-1].combin_th + 1; /* Make combo timer expire immediately */
                obj->bt_run[id-1].stage = BUTTON_STAGE_NORMAL;
            }
            break;
		    }
    }
}

static void button_control_callback(Button_t* obj)
{
  button_scan(obj , 1);
  button_scan(obj , 2);
}

void button_defalut_event_callback(uint8_t id,  ButtonEventIDEnum event)
{
}

void Button_t::init(void)
{
    for(int i = 0 ; i < 2 ; i++){
      bt_run[i].stage = BUTTON_STAGE_NORMAL;
      bt_run[i].last_pin_raw = 0;
      bt_run[i].last_pin_filtered = 0;
      bt_run[i].combin_counter = 0;
      bt_run[i].ticks_count = 0;

      /* config */
      bt_run[i].lp_th = 2000;  
      bt_run[i].repeat_th = 500;
      bt_run[i].combin_th = 300;
      bt_run[i].lp_th = 1500;  
		  bt_run[i].repeat_th = 400;
    }
    event_callback = button_defalut_event_callback; 
    timer_button.attach((BUTTON_TASK_PERIOD/1000), button_control_callback , this);
}

uint8_t Button_t::read(uint8_t id)
{
  int adcValue = analogRead(IO_BUTTON);
  uint8_t button_id = 0;
  if(700 < adcValue && adcValue < 900)
  {
    button_id = 1;
  }else if(1600 < adcValue && adcValue < 2000)
  {
    button_id = 2;
  }else if(500 < adcValue && adcValue < 700)
  {
    button_id = 3;
  }
  if(id == 1 && (button_id == 1 || button_id == 3))
  {
    return 1;
  }else if(id == 2 && (button_id == 2 || button_id == 3))
  {
    return 1;
  }else{
    return 0;
  }
}

void Button_t::register_callback(void (*function)(uint8_t , ButtonEventIDEnum))
{
  event_callback = function;
}

