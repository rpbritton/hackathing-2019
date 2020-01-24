/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== hackathing.c ========
 */

/* Standard header files. */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

/* TI-RTOS header files. */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

/* Driver header files. */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/PWM.h>

/* Driver configuration. */
#include "ti_drivers_config.h"

/* Definitions. */
#define LOOP_DELAY   ( 5 )
#define LOOP_DELAY_TICKS    ( LOOP_DELAY * (Clock_tickPeriod / 1000) )

#define GATE_TOTAL          ( 3000 / LOOP_DELAY )
#define GATE_ENABLE         ( 200 / LOOP_DELAY )

#define ROLLER_TOTAL     ( 30000 / LOOP_DELAY )
#define ROLLER_ENABLE    ( 1000 / LOOP_DELAY )

#define ROLLER_TOGGLER_TOTAL     ( 1500 / LOOP_DELAY )
#define ROLLER_TOGGLER_ENABLE    ( ROLLER_TOGGLER_TOTAL * 0.4 )

#define GATE_SENSOR_LEVEL   ( 8000 )
#define ROLLER_SENSOR_LEVEL ( 15000 )

#define GATE_SPEED          ( 0.85 )
#define ROLLER_SPEED        ( 0.9 )

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* GPIO Driver. */
    GPIO_init();

    /* ADC Driver. */
    ADC_init();
    /* ADC Params. */
    ADC_Params gateSensorADCParams;
    ADC_Params_init(&gateSensorADCParams);
    ADC_Params rollerSensorADCParams;
    ADC_Params_init(&rollerSensorADCParams);
    /* ADC Channels. */
    ADC_Handle gateSensorADC = ADC_open(GATE_SENSOR_ADC, &gateSensorADCParams);
    if (gateSensorADC == NULL) {
        while (1) {}
    }
    ADC_Handle rollerSensorADC = ADC_open(ROLLER_SENSOR_ADC, &rollerSensorADCParams);
    if (rollerSensorADC == NULL) {
        while (1) {}
    }

    /* PWM Driver. */
    PWM_init();
    /* PWM Params. */
    PWM_Params gatePWMParams;
    PWM_Params_init(&gatePWMParams);
    PWM_Params rollerPWMParams;
    PWM_Params_init(&rollerPWMParams);
    /* PWM Channels. */
    PWM_Handle gatePWM = PWM_open(GATE_PWM, &gatePWMParams);
    if (gatePWM == NULL) {
        while (1) {}
    }
    PWM_Handle rollerPWM = PWM_open(ROLLER_PWM, &rollerPWMParams);
    if (rollerPWM == NULL) {
        while (1) {}
    }
    PWM_start(gatePWM);
    PWM_start(rollerPWM);

    int gateCounter = 0;
    int rollerCounter = 0;
    int rollerTogglerCounter = 0;

    while (1) {
        uint32_t loopStart = Clock_getTicks();

        /*
         * Gate Stuff.
         */

        uint16_t gateSensorADCValue;
        int_fast16_t gateSensorADCRes = ADC_convert(gateSensorADC, &gateSensorADCValue);
        if (gateSensorADCRes == ADC_STATUS_SUCCESS) {
            if (gateSensorADCValue < GATE_SENSOR_LEVEL) {
                if (gateCounter < GATE_TOTAL) {
                    gateCounter++;
                }
                GPIO_write(GATE_SENSOR_LED_BLUE, 1);
            }
            else {
                if (gateCounter > 0) {
                    gateCounter--;
                }
                GPIO_write(GATE_SENSOR_LED_BLUE, 0);
            }
        }
        if (gateCounter < GATE_ENABLE) {
            PWM_setDuty(gatePWM, PWM_DUTY_FRACTION_MAX * GATE_SPEED);
            GPIO_write(PICKUP_NOW_PIN, 0);
        } else {
            PWM_setDuty(gatePWM, 0);
            GPIO_write(PICKUP_NOW_PIN, 1);
        }

        /*
         * Roller Stuff.
         */

        uint16_t rollerSensorADCValue;
        int_fast16_t rollerSensorADCRes = ADC_convert(rollerSensorADC, &rollerSensorADCValue);
        if (rollerSensorADCRes == ADC_STATUS_SUCCESS) {
            if (rollerSensorADCValue < ROLLER_SENSOR_LEVEL) {
                if (rollerCounter < ROLLER_TOTAL) {
                    rollerCounter++;
                }
                GPIO_write(ROLLER_SENSOR_LED_GREEN, 1);
            }
            else {
                if (rollerCounter > 0) {
                    rollerCounter--;
                }
                GPIO_write(ROLLER_SENSOR_LED_GREEN, 0);
            }
        }
        rollerTogglerCounter++;
        if (rollerCounter < ROLLER_ENABLE && rollerTogglerCounter < ROLLER_TOGGLER_ENABLE) {
            PWM_setDuty(rollerPWM, PWM_DUTY_FRACTION_MAX * GATE_SPEED);
        } else {
            PWM_setDuty(rollerPWM, 0);
        }
        if (rollerTogglerCounter > ROLLER_TOGGLER_TOTAL) {
            rollerTogglerCounter = 0;
        }

        /*
         * Loop Timing Stuff.
         */

        uint32_t loopLength = Clock_getTicks() - loopStart;

        if (loopLength > LOOP_DELAY_TICKS) {
            GPIO_write(OVERRUN_LED_RED, 1);
        } else {
            GPIO_write(OVERRUN_LED_RED, 0);
        }

        Task_sleep(LOOP_DELAY_TICKS - loopLength);
    }
}
