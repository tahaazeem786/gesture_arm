#include "./../../Hiwonder.hpp"
#include <Arduino.h>

#define BUZZER_TASK_PERIOD  ((float)30) /* Buzzer status refresh interval (ms) */

static void buzzer_control_callback(Buzzer_t* obj)
{
  /* Try to take new control data from the queue; if succeeded, reset state machine to start a new control cycle */
    if(obj->new_flag != 0) {
        obj->new_flag = 0;
        obj->stage = BUZZER_STAGE_START_NEW_CYCLE;
    }
    /* State machine processing */
    switch(obj->stage) {
        case BUZZER_STAGE_START_NEW_CYCLE: {
            if(obj->ticks_on > 0 && obj->freq > 0) {
                ledcWriteTone(obj->buzzer_channel , obj->freq); /* Sound the buzzer */
                if(obj->ticks_off > 0) {/* If off time > 0 then beeping, else continuous tone */
                    obj->ticks_count = 0;
                    obj->stage = BUZZER_STAGE_WATTING_OFF; /* Wait until sound time ends */
                }else{
					obj->stage = BUZZER_STAGE_IDLE; /* Continuous tone, enter idle */
				}
            } else { /* If sound time is 0 then silent */
                ledcWriteTone(obj->buzzer_channel , 0);
				obj->stage = BUZZER_STAGE_IDLE;  /* Continuous silence, enter idle */
            }
            break;
        }
        case BUZZER_STAGE_WATTING_OFF: {
            obj->ticks_count += BUZZER_TASK_PERIOD;
            if(obj->ticks_count >= obj->ticks_on) { /* Sound time ended */
                ledcWriteTone(obj->buzzer_channel , 0);
                obj->stage = BUZZER_STAGE_WATTING_PERIOD_END;
            }
            break;
        }
        case BUZZER_STAGE_WATTING_PERIOD_END: { /* Wait for period end */
            obj->ticks_count += BUZZER_TASK_PERIOD;
            if(obj->ticks_count >= (obj->ticks_off + obj->ticks_on)) {
                obj->ticks_count -= (obj->ticks_off + obj->ticks_on);
                if(obj->repeat == 1) { /* If remaining repeat count is 1, finish control task */
                    ledcWriteTone(obj->buzzer_channel , 0);
                    obj->stage = BUZZER_STAGE_IDLE;
                } else {
                    ledcWriteTone(obj->buzzer_channel , obj->freq);
                    obj->repeat = obj->repeat == 0 ? 0 : obj->repeat - 1;
                    obj->stage = BUZZER_STAGE_WATTING_OFF;
                }
            }
            break;
        }
        case BUZZER_STAGE_IDLE: {
            break;
        }
        default:
            break;
    }
}


void Buzzer_t::init(uint8_t pin , uint8_t channel , uint16_t frequency)
{
    buzzer_pin = pin;
    stage = BUZZER_STAGE_IDLE;
    ticks_count = 0;
    freq = frequency;
    buzzer_channel = channel;
    ledcSetup(buzzer_channel,frequency,12);
    ledcAttachPin(buzzer_pin, buzzer_channel);
    ledcWrite(buzzer_channel, 2048);
    ledcWriteTone(buzzer_channel , 0);
    timer_buzzer.attach((BUZZER_TASK_PERIOD/1000), buzzer_control_callback , this);
}

void Buzzer_t::on_off(uint8_t state)
{
  if(state != 0)
  {
    blink(1500 , 200 , 400 , 0);
  }else{
    blink(1500 , 0 , 400 , 0);
  }
}

void Buzzer_t::blink(uint16_t frequency , uint16_t on_time , uint16_t off_time , uint16_t count)
{
    new_flag = 1;
    freq = frequency;
    ticks_on = on_time;
    ticks_off = off_time;
    repeat = count;
}
