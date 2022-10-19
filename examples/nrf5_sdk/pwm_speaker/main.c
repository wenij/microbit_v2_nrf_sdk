/**
 * Copyright (c) 2015 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup pwm_example_main main.c
 * @{
 * @ingroup pwm_example
 *
 * @brief PWM Example Application main file.
 *
 * This file contains the source code for a sample application using PWM.
 */

#include <stdio.h>
#include <string.h>
#include "boards.h"

#include "app_util_platform.h"
#include "app_error.h"
#include "app_uart.h"

#include "bsp.h"
#include "nrf_delay.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "nrf_drv_pwm.h"
//#include "nrf_drv_clock.h"


static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
//static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);
//static nrf_drv_pwm_t m_pwm2 = NRF_DRV_PWM_INSTANCE(2);

// This is for tracking PWM instances being used, so we can unintialize only
// the relevant ones when switching from one demo to another.
#define USED_PWM(idx) (1UL << idx)
static uint8_t m_used = 0;





static void demo2(void)
{
    printf("demo2");
   
    /*
     * This demo plays back two concatenated sequences:
     * - Sequence 0: Light intensity is increased in 25 steps during one second.
     * - Sequence 1: LED blinks twice (100 ms off, 100 ms on), then stays off
     *   for 200 ms.
     * The same output is generated on all 4 channels (LED 1 - LED 4).
     * The playback is repeated in a loop.
     */

    enum { // [local constants]
        TOP        = 1136,
        STEP_COUNT = 25
    };

    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            NRF_DRV_PWM_PIN_NOT_USED, // channel 0
            2 | NRF_DRV_PWM_PIN_INVERTED, // channel 1
             NRF_DRV_PWM_PIN_NOT_USED, // channel 2
             NRF_DRV_PWM_PIN_NOT_USED  // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_500kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = TOP,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));
    m_used |= USED_PWM(0);

    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM.
    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).
    static nrf_pwm_values_common_t /*const*/ seq0_values[] =
    {
            568,
            //568,
    };

    nrf_pwm_sequence_t const seq0 =
    {
        .values.p_common = seq0_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq0_values),
        .repeats         = 0,
        .end_delay       = 20, /* 100: 231 ms
                                   10: 23  ms  
                                   20: 48  ms
                               */
    };

    nrf_drv_pwm_simple_playback(&m_pwm0, &seq0, 2,
                                      NRFX_PWM_FLAG_STOP /* NRF_DRV_PWM_FLAG_LOOP */ );
}

void pwm_set_freq(int channel,int gpio_pin,uint32_t nHz)
{
    uint32_t counter=0;

    nrf_drv_pwm_config_t  config0 =
    {
        .output_pins =
        {
            NRF_DRV_PWM_PIN_NOT_USED, // channel 0
            NRF_DRV_PWM_PIN_NOT_USED, // channel 1
             NRF_DRV_PWM_PIN_NOT_USED, // channel 2
             NRF_DRV_PWM_PIN_NOT_USED  // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_500kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        //.top_value    = TOP,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    if (channel <=3 || channel >=0) {
        uint32_t top;
        config0.output_pins[channel]= gpio_pin | NRF_DRV_PWM_PIN_INVERTED ;
        top = 500000U / nHz ;
        counter = top/2;
        config0.top_value = top;
    }
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));
    //m_used |= USED_PWM(0);

    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM.
    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).
    static nrf_pwm_values_common_t /*const*/ seq0_values[1];
    seq0_values[0]=counter;

    nrf_pwm_sequence_t const seq0 =
    {
        .values.p_common = seq0_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq0_values),
        .repeats         = 0,
        .end_delay       = 0, /* 100: 231 ms
                                   10: 23  ms  
                                   20: 48  ms
                               */
    };

    nrf_drv_pwm_simple_playback(&m_pwm0, &seq0, 1, NRF_DRV_PWM_FLAG_LOOP );
}

void pwm_stop()
{
     nrf_drv_pwm_uninit(&m_pwm0);
}
#if 0
static int run=1;
static void bsp_evt_handler(bsp_event_t evt)
{
    

    switch (evt)
    {
        // Button 1 - switch to the previous demo.
        case BSP_EVENT_KEY_0:
            if(run==1) {
                nrfx_pwm_stop(&m_pwm0,true);
                run = 0;
            }

            break;

        // Button 2 - switch to the next demo.
        case BSP_EVENT_KEY_1:
           
            break;

        default:
            return;
    }

}
#endif
static void init_bsp()
{
    bsp_board_init(BSP_INIT_LEDS);
    #if 0 /* following code will cause pin0 not work. */
    APP_ERROR_CHECK(nrf_drv_clock_init());
    nrf_drv_clock_lfclk_request(NULL);

    APP_ERROR_CHECK(app_timer_init());
    #endif
    //APP_ERROR_CHECK(bsp_init(BSP_INIT_BUTTONS, bsp_evt_handler));
    //APP_ERROR_CHECK(bsp_buttons_enable());
     #if 1
    nrf_gpio_cfg(2,
            NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_H0H1,
        NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_clear(2);
    
    nrf_gpio_cfg(0,
            NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
        NRF_GPIO_PIN_H0H1,
        NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_clear(0);
    #endif
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    bsp_board_leds_on();
    app_error_save_and_stop(id, pc, info);
}

#define UART_HWFC APP_UART_FLOW_CONTROL_ENABLED
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
int main(void)
{
     #if 1
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          NRF_UARTE_PSEL_DISCONNECTED,
          NRF_UARTE_PSEL_DISCONNECTED,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
//#if defined (UART_PRESENT)
//          #error...
//          NRF_UART_BAUDRATE_115200
//#else
          NRF_UARTE_BAUDRATE_115200
//#endif
      };


    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);
  #endif
    //APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    //NRF_LOG_DEFAULT_BACKENDS_INIT();
    printf("init_bsp\r\n");
    nrf_delay_ms(50);
    init_bsp();

    //NRF_LOG_INFO("PWM example started.");

    // Start with Demo 1, then switch to another one when the user presses
    // button 1 or button 2 (see the 'bsp_evt_handler' function).
    printf("run demo2\r\n");
    nrf_delay_ms(50);
    //demo2();
    pwm_set_freq(0,0,1110);
    nrf_delay_ms(235);
    pwm_stop();
    pwm_set_freq(0,0,4350);
    nrf_delay_ms(156);
    pwm_stop();

    pwm_set_freq(0,0,3750);
    nrf_delay_ms(235);
    pwm_stop();

    pwm_set_freq(0,0,4350);
    nrf_delay_ms(156);
    pwm_stop();

    pwm_set_freq(0,0,2200);
    nrf_delay_ms(156);
    pwm_stop();

    pwm_set_freq(0,0,3750);
    nrf_delay_ms(235);
    pwm_stop();

    pwm_set_freq(0,0,4350);
    nrf_delay_ms(156);
    pwm_stop();

    pwm_set_freq(0,0,1100);
    nrf_delay_ms(235);
    pwm_stop();

    for (;;)
    {
        // Wait for an event.
        __WFE();
        printf(" . \n");
        nrf_delay_ms(50);
        // Clear the event register.
        __SEV();
        __WFE();

       // NRF_LOG_FLUSH();
    }
    demo2();
}


/** @} */
