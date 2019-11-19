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
 *  ======== empty.c ========
 */

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/apps/Button.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/apps/LED.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Motor enables. */
volatile bool enable_roller = false;
volatile bool enable_gate = false;

/* Button Callbacks. */
void gateBtnCb(Button_Handle handle, Button_EventMask events);
void rollerBtnCb(Button_Handle handle, Button_EventMask events);

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* PWM Driver. */
    PWM_Params pwmParams;
    PWM_Handle rollerPwmHandle;
    PWM_Handle gatePwmHandle;
    /* Init the PWM driver. */
    PWM_init();
    /* Init the PWM parameters. */
    PWM_Params_init(&pwmParams);
    pwmParams.periodValue = 2e3; // 2kHz
    /* Open the PWM instances. */
    rollerPwmHandle = PWM_open(CONFIG_PWM_0, &pwmParams);
    if (rollerPwmHandle == NULL) {
        /* Failed to open. */
        while (1);
    }
    gatePwmHandle = PWM_open(CONFIG_PWM_1, &pwmParams);
    if (gatePwmHandle == NULL) {
        /* Failed to open. */
        while (1);
    }
    /* Start the PWM instances. */
    PWM_start(rollerPwmHandle);
    PWM_start(gatePwmHandle);

    /* Button Driver. */
    Button_Params btnParams;
    Button_Handle rollerBtnHandle;
    Button_Handle gateBtnHandle;
    /* Init the button parameters. */
    Button_Params_init(&btnParams);
    /* Open the button instances. */
    rollerBtnHandle = Button_open(CONFIG_BUTTON_1, rollerBtnCb, &btnParams);
    if (rollerBtnHandle == NULL) {
        /* Failed to open. */
        while (1);
    }
    gateBtnHandle = Button_open(CONFIG_BUTTON_0, gateBtnCb, &btnParams);
    if (gateBtnHandle == NULL) {
        /* Failed to open. */
        while (1);
    }

    /* LED Driver. */
    LED_Params ledParams;
    LED_Handle ledHandle;
    /* Init the led driver. */
    LED_init();
    /* Init the led parameters. */
    LED_Params_init(&ledParams);
    /* Open the led instance. */
    ledHandle = LED_open(CONFIG_LED_0, &ledParams);
    if (ledHandle == NULL) {
        /* Failed to open. */
        while (1);
    }

    /* ADC Driver. */
    ADC_Params adcParams;
    ADC_Handle adcHandle;
    /* Init ADC Driver. */
    ADC_init();
    /* Init ADC parameters. */
    ADC_Params_init(&adcParams);
    /* Open the ADC instance. */
    adcHandle = ADC_open(CONFIG_ADC_0, &adcParams);
    if (adcHandle == NULL) {
        /* Failed to open. */
        while (1);
    }

    while (1) {
        if (enable_gate) {
            PWM_setDuty(gatePwmHandle, PWM_DUTY_FRACTION_MAX * 0.4);
        }
        else {
            PWM_setDuty(gatePwmHandle, 0);
        }

        if (enable_roller) {
            PWM_setDuty(rollerPwmHandle, PWM_DUTY_FRACTION_MAX * 0.75);
        }
        else {
            PWM_setDuty(rollerPwmHandle, 0);
        }

        uint16_t adcReading = 0;
        ADC_convert(adcHandle, &adcReading);
        if (adcReading > 100) {
            LED_setOff(ledHandle);
        }
        else {
            LED_setOn(ledHandle, 100);
        }
    }
}

/*
 *  ======== rollerBtnCb ========
 */
void rollerBtnCb(Button_Handle handle, Button_EventMask events) {
    if (events & Button_EV_PRESSED) {
        enable_roller = !enable_roller;
    }
}

/*
 *  ======== gateBtnCb ========
 */
void gateBtnCb(Button_Handle handle, Button_EventMask events) {
    if (events & Button_EV_PRESSED) {
        enable_gate = !enable_gate;
    }
}
