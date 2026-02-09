#include "./../../Hiwonder.hpp"
#include <Arduino.h>

#define LED_TASK_PERIOD     ((float)30) // LED status refresh interval (ms)

#define BATTERY_TASK_PERIOD ((float)50) // Battery capacity detection interval (ms)

static void led_control_callback(Led_t* obj)
{
    /* Try to take new control data from the queue; if succeeded, reset state machine to start a new control cycle */
    if(obj->new_flag != 0) {
        obj->new_flag = 0;
        obj->stage = LED_STAGE_START_NEW_CYCLE;
    }
    /* State machine processing */
    switch(obj->stage) {
        case LED_STAGE_START_NEW_CYCLE: {
            if(obj->ticks_on > 0) {
                digitalWrite(obj->led_pin,LOW);
                if(obj->ticks_off > 0) { /* If off time > 0 then blinking, else steady on */
                    obj->ticks_count = 0;
                    obj->stage = LED_STAGE_WATTING_OFF; /* Wait until LED on time ends */
                }else{
                  obj->stage = LED_STAGE_IDLE; /* Steady on, enter idle */
                }
            } else { /* If on time is 0 then steady off */
                digitalWrite(obj->led_pin,HIGH);
			        obj->stage = LED_STAGE_IDLE; /* Steady off, enter idle */
            }
            break;
        }
        case LED_STAGE_WATTING_OFF: {
            obj->ticks_count += LED_TASK_PERIOD;
            if(obj->ticks_count >= obj->ticks_on) { /* LED on time ended */
                digitalWrite(obj->led_pin,HIGH);
                obj->stage = LED_STAGE_WATTING_PERIOD_END;
            }
            break;
        }
        case LED_STAGE_WATTING_PERIOD_END: { /* Wait for period end */
            obj->ticks_count += LED_TASK_PERIOD;
            if(obj->ticks_count >= (obj->ticks_off + obj->ticks_on)) {
				        obj->ticks_count -= (obj->ticks_off + obj->ticks_on);
                if(obj->repeat == 1) { /* If remaining repeat count is 1, finish control task */
                    digitalWrite(obj->led_pin,HIGH);
                    obj->stage = LED_STAGE_IDLE;  /* Repeats finished, enter idle */
                } else {
                    digitalWrite(obj->led_pin,LOW);
                    obj->repeat = obj->repeat == 0 ? 0 : obj->repeat - 1;
                    obj->stage = LED_STAGE_WATTING_OFF;
                }
            }
            break;
        }
        case LED_STAGE_IDLE: {
            break;
        }
        default:
            break;
    }
}

void Led_t::init(uint8_t pin)
{
    led_pin = pin;
    stage = LED_STAGE_IDLE;
    ticks_count = 0;
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin,HIGH);
    timer_led.attach((LED_TASK_PERIOD/1000), led_control_callback , this);
}

void Led_t::on_off(uint8_t state)
{
    if(state)
    {
        blink(100 , 0 , 0);
    }else{
        blink(0 , 100 , 0);
    }
}

void Led_t::blink(uint32_t on_time , uint32_t off_time , uint32_t count)
{
    new_flag = 1;
    ticks_on = on_time;
    ticks_off = off_time;
    repeat = count;
}

