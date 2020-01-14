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

/* Standard Libraries. */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/PWM.h>

/* Driver configuration */
#include "ti_drivers_config.h"

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
    ADC_Params adcParams;
    ADC_Params_init(&adcParams);
    /* ADC Channels. */
    ADC_Handle adc0 = ADC_open(CONFIG_ADC_0, &adcParams);
    if (adc0 == NULL) {
        while (1) {}
    }

    /* PWM Driver. */
    PWM_init();
    /* PWM Params. */
    PWM_Params pwm0Params;
    PWM_Params_init(&pwm0Params);
    PWM_Params pwm1Params;
    PWM_Params_init(&pwm1Params);
    /* PWM Channels. */
    PWM_Handle pwm0 = PWM_open(CONFIG_PWM_0, &pwm0Params);
    if (pwm0 == NULL) {
        while (1) {}
    }
    PWM_Handle pwm1 = PWM_open(CONFIG_PWM_1, &pwm1Params);
    if (pwm1 == NULL) {
        while (1) {}
    }
    PWM_start(pwm0);
    PWM_start(pwm1);

    int gateCounter = 75000;
    int rollerCounter = 0;
    while (1) {
        uint16_t adc0Value;
        int_fast16_t adc0Res = ADC_convert(adc0, &adc0Value);
        if (adc0Res == ADC_STATUS_SUCCESS) {
            if (adc0Value < 6000) {
                if (gateCounter > 0) {
                    gateCounter--;
                }
                GPIO_write(CONFIG_GPIO_0, 1);
            }
            else {
                if (gateCounter < 100000) {
                    gateCounter++;
                }
                GPIO_write(CONFIG_GPIO_0, 0);
            }
        }

        if (gateCounter < 98000) {
            PWM_setDuty(pwm0, 0);
        } else {
            PWM_setDuty(pwm0, PWM_DUTY_FRACTION_MAX * 0.9);
        }

        if (rollerCounter < 20000) {
            PWM_setDuty(pwm1, PWM_DUTY_FRACTION_MAX * 0);
        } else if (rollerCounter < 25000) {
            PWM_setDuty(pwm1, PWM_DUTY_FRACTION_MAX * 0.7);
        } else {
            rollerCounter = 0;
        }
        rollerCounter++;
    }
}
