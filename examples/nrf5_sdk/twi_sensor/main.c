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
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include <string.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "app_uart.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif
#include "lsm303agr_reg.h"

// porting the driver code from https://github.com/STMicroelectronics/lsm303agr-pid
// porting the demo code from https://github.com/STMicroelectronics/STMems_Standard_C_drivers/tree/master/lsm303agr_STdC/examples

/* comment out this define LSM303_SELF_TEST to deom the polling data from LSM303AGR chip */
//#define LSM303_SELF_TEST






static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
//static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0


/* Indicates if operation on TWI has ended. */
//static volatile bool m_xfer_done = false;

/* TWI instance. */
static  nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
//static uint8_t m_sample;

#define    BOOT_TIME               5 //ms

/* Self test limits. */
#define    MIN_ST_XL_LIMIT_mg     68.0f
#define    MAX_ST_XL_LIMIT_mg   1440.0f
#define    MIN_ST_MG_LIMIT_mG     15.0f
#define    MAX_ST_MG_LIMIT_mG    500.0f
/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

//====
typedef struct {
  void   *hbus;
  uint8_t i2c_address;
} sensbus_t;


static sensbus_t xl_bus  = {&m_twi,
                            LSM303AGR_I2C_ADD_XL  /* 0x19 , i2c addr for micro:bit v2 */
                            
                           };
static sensbus_t mag_bus = {&m_twi,
                            LSM303AGR_I2C_ADD_MG  /* 0x1E ,i2c addr for micro:bit v2 */
                            
                           };

//==
/* Main Example --------------------------------------------------------------*/

void lsm303agr_self_test(void)
{
  //uint8_t tx_buffer[1000];
  int16_t data_raw[3];
  float maes_st_off[3];
  float maes_st_on[3];
  lsm303agr_reg_t reg;
  float test_val[3];
  uint8_t st_result;
  uint8_t i, j;
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx_xl;
  dev_ctx_xl.write_reg = platform_write;
  dev_ctx_xl.read_reg = platform_read;
  dev_ctx_xl.handle = (void *)&xl_bus;
  stmdev_ctx_t dev_ctx_mg;
  dev_ctx_mg.write_reg = platform_write;
  dev_ctx_mg.read_reg = platform_read;
  dev_ctx_mg.handle = (void *)&mag_bus;
  /* Initialize self test results */
  st_result = ST_PASS;
  /* Wait boot time and initialize platform specific hardware */
  //platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  reg.byte = 0;
  lsm303agr_xl_device_id_get(&dev_ctx_xl, &reg.byte);

  printf("xl_id=%x\r\n",reg.byte);

  if ( reg.byte != LSM303AGR_ID_XL )
    while (1); /*manage here device not found */

  reg.byte = 0;
  lsm303agr_mag_device_id_get(&dev_ctx_mg, &reg.byte);
  printf("mag_id=%x\r\n",reg.byte);

  if ( reg.byte != LSM303AGR_ID_MG )
    while (1); /*manage here device not found */

  printf("start... test ...\r\n");
  /*
   * Accelerometer Self Test
   */
  /* Enable Block Data Update. */
  printf("Enable Block Data Update.\r\n");
  lsm303agr_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
  /* Set full scale to 2g. */
  printf("Set full scale to 2g.\r\n");
  lsm303agr_xl_full_scale_set(&dev_ctx_xl, LSM303AGR_2g);
  /* Set device in normal mode. */
  printf("Set device in normal mode. 10bit \r\n");
  lsm303agr_xl_operating_mode_set(&dev_ctx_xl, LSM303AGR_NM_10bit);
  /* Set Output Data Rate. */
  printf("Set Output Data Rate. ODR 100Hz \r\n");
  lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_100Hz);
  /* Wait stable output. */
  platform_delay(90);

  /* Check if new value available */
  do {
    lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
    printf("status(zyxda)=%x\r\n",(int)reg.status_reg_a.zyxda);
  } while (!reg.status_reg_a.zyxda);

  /* Read dummy data and discard it */
  printf("Read dummy data and discard it.\r\n");
  lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw);

  /* Read 5 sample and get the average value for each axis */
  printf("Read 5 sample and get the average value for each axis\r\n");
  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
    } while (!reg.status_reg_a.zyxda);

    /* Read data and accumulate the mg value */
    lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw);

    for (j = 0; j < 3; j++) {
      maes_st_off[j] += lsm303agr_from_fs_2g_nm_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  printf("Calculate the mg average values\r\n");
  for (i = 0; i < 3; i++) {
    maes_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  printf("Enable Self Test positive (or negative)\r\n");
  lsm303agr_xl_self_test_set(&dev_ctx_xl, LSM303AGR_ST_POSITIVE);
  //lsm303agr_xl_self_test_set(&dev_ctx, LSM303AGR_XL_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(90);

  /* Check if new value available */
  do {
    lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
  } while (!reg.status_reg_a.zyxda);

  /* Read dummy data and discard it */
  printf("Read dummy data and discard it\r\n");
  lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw);

  /* Read 5 sample and get the average value for each axis */
  printf("Read 5 sample and get the average value for each axis\r\n");
  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);
    } while (!reg.status_reg_a.zyxda);

    /* Read data and accumulate the mg value */
    lsm303agr_acceleration_raw_get(&dev_ctx_xl, data_raw);

    for (j = 0; j < 3; j++) {
      maes_st_on[j] += lsm303agr_from_fs_2g_nm_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  printf("Calculate the mg average values\r\n");
  for (i = 0; i < 3; i++) {
    maes_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  printf("Calculate the mg values for self test \r\n");
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((maes_st_on[i] - maes_st_off[i]));
  }

  /* Check self test limit */
  printf("Check self test limit\r\n");
  for (i = 0; i < 3; i++) {
    if (( MIN_ST_XL_LIMIT_mg > test_val[i] ) ||
        ( test_val[i] > MAX_ST_XL_LIMIT_mg)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  printf("Disable Self Test\r\n");
  lsm303agr_xl_self_test_set(&dev_ctx_xl, LSM303AGR_ST_DISABLE);
  /* Disable sensor. */
  printf("Disable sensor.\r\n");
  lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_POWER_DOWN);
  /*
   * Magnetometer  Self Test
   */
  /* Restore default configuration for magnetometer */
  printf("Restore default configuration for magnetometer\r\n");
  lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);

  do {
    lsm303agr_mag_reset_get(&dev_ctx_mg, &reg.byte);
    printf("reg.byte=%x\r\n",reg.byte);
  } while (reg.byte);

  /* Enable Block Data Update. */
  printf("Enable Block Data Update.\r\n");
  lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Set / Reset sensor mode. */
  lsm303agr_mag_set_rst_mode_set(&dev_ctx_mg,
                                 LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
  /* Enable temperature compensation. */
  lsm303agr_mag_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Set device in continuous mode. */
  lsm303agr_mag_operating_mode_set(&dev_ctx_mg,
                                   LSM303AGR_CONTINUOUS_MODE);
  /* Set Output Data Rate. */
  lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_100Hz);
  /* Wait stable output. */
  platform_delay(20);

  /* Check if new value available .*/
  printf("Check if new value available \r\n");
  do {
    lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
  } while (!reg.status_reg_m.zyxda);

  /* Read dummy data and discard it. */
  printf("Read dummy data and discard it.\r\n");
  lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw);

  /* Read 50 sample and get the average value for each axis */
  printf("Read 50 sample and get the average value for each axis\r\n");
  for (i = 0; i < 50; i++) {
    /* Check if new value available */
    do {
      lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
    } while (!reg.status_reg_m.zyxda);

    /* Read data and accumulate the mg value */
    lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw);

    for (j = 0; j < 3; j++) {
      maes_st_off[j] += lsm303agr_from_lsb_to_mgauss(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    maes_st_off[i] /= 50.0f;
  }

  /* Enable Self Test. */
  printf("Enable Self Test.\r\n");
  lsm303agr_mag_self_test_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Wait stable output */
  platform_delay(60);

  /* Check if new value available .*/
  do {
    lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
  } while (!reg.status_reg_m.zyxda);

  /* Read dummy data and discard it. */
  lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw);

  /* Read 50 sample and get the average value for each axis */
  printf("Read 50 sample and get the average value for each axis\r\n");
  for (i = 0; i < 50; i++) {
    /* Check if new value available */
    do {
      lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);
    } while (!reg.status_reg_m.zyxda);

    /* Read data and accumulate the mg value */
    lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw);

    for (j = 0; j < 3; j++) {
      maes_st_on[j] += lsm303agr_from_lsb_to_mgauss(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    maes_st_on[i] /= 50.0f;
  }

  /* Calculate the mg values for self test */
  printf("Calculate the mg values for self test\r\n");
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((maes_st_on[i] - maes_st_off[i]));
  }

  /* Check self test limit */
  printf("Check self test limit\r\n");
  for (i = 0; i < 3; i++) {
    printf("test_val[%d]=%f\r\n",i,test_val[i]);
    if (( MIN_ST_MG_LIMIT_mG > test_val[i] ) ||
        ( test_val[i] > MAX_ST_MG_LIMIT_mG)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  printf("Disable Self Test \r\n");
  lsm303agr_mag_self_test_set(&dev_ctx_mg, PROPERTY_DISABLE);
  /* Disable sensor. */
  printf("Disable sensor.\r\n");
  lsm303agr_mag_operating_mode_set(&dev_ctx_mg, LSM303AGR_POWER_DOWN);

  /* Print self test result */
  if (st_result == ST_PASS) {
    printf("Self Test - PASS\r\n" );
  }

  else {
    printf("Self Test - FAIL\r\n" );
  }

  //tx_com(tx_buffer, strlen((char const *)tx_buffer));
}


static int16_t data_raw_acceleration[3];
static int16_t data_raw_magnetic[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float magnetic_mG[3];
static float temperature_degC;
static uint8_t whoamI, rst;
//static uint8_t tx_buffer[TX_BUF_DIM];
void lsm303agr_read_data_polling(void)
{
  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx_xl;
  dev_ctx_xl.write_reg = platform_write;
  dev_ctx_xl.read_reg = platform_read;
  dev_ctx_xl.handle = (void *)&xl_bus;
  stmdev_ctx_t dev_ctx_mg;
  dev_ctx_mg.write_reg = platform_write;
  dev_ctx_mg.read_reg = platform_read;
  dev_ctx_mg.handle = (void *)&mag_bus;
  /* Wait boot time and initialize platform specific hardware */
  //platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  whoamI = 0;
  lsm303agr_xl_device_id_get(&dev_ctx_xl, &whoamI);
  printf("xl=%x\r\n",(int)whoamI);
  if ( whoamI != LSM303AGR_ID_XL )
    while (1); /*manage here device not found */

  whoamI = 0;
  lsm303agr_mag_device_id_get(&dev_ctx_mg, &whoamI);
  printf("mag=%x\r\n",(int)whoamI);
  if ( whoamI != LSM303AGR_ID_MG )
    while (1); /*manage here device not found */

  /* Restore default configuration for magnetometer */
  printf("Restore default configuration for magnetometer\r\n");
  lsm303agr_mag_reset_set(&dev_ctx_mg, PROPERTY_ENABLE);

  do {
    lsm303agr_mag_reset_get(&dev_ctx_mg, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm303agr_xl_block_data_update_set(&dev_ctx_xl, PROPERTY_ENABLE);
  lsm303agr_mag_block_data_update_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm303agr_xl_data_rate_set(&dev_ctx_xl, LSM303AGR_XL_ODR_1Hz);
  lsm303agr_mag_data_rate_set(&dev_ctx_mg, LSM303AGR_MG_ODR_10Hz);
  /* Set accelerometer full scale */
  lsm303agr_xl_full_scale_set(&dev_ctx_xl, LSM303AGR_2g);
  /* Set / Reset magnetic sensor mode */
  lsm303agr_mag_set_rst_mode_set(&dev_ctx_mg,
                                 LSM303AGR_SENS_OFF_CANC_EVERY_ODR);
  /* Enable temperature compensation on mag sensor */
  lsm303agr_mag_offset_temp_comp_set(&dev_ctx_mg, PROPERTY_ENABLE);
  /* Enable temperature sensor */
  lsm303agr_temperature_meas_set(&dev_ctx_xl, LSM303AGR_TEMP_ENABLE);
  /* Set device in continuous mode */
  lsm303agr_xl_operating_mode_set(&dev_ctx_xl, LSM303AGR_HR_12bit);
  /* Set magnetometer in continuous mode */
  lsm303agr_mag_operating_mode_set(&dev_ctx_mg,
                                   LSM303AGR_CONTINUOUS_MODE);

  /* Read samples in polling mode (no int) */
  printf("Read samples in polling mode (no int) \r\n");
  while (1) {
    /* Read output only if new value is available */
    lsm303agr_reg_t reg;
    lsm303agr_xl_status_get(&dev_ctx_xl, &reg.status_reg_a);

    if (reg.status_reg_a.zyxda) {
      /* Read accelerometer data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lsm303agr_acceleration_raw_get(&dev_ctx_xl,
                                     data_raw_acceleration);
      acceleration_mg[0] = lsm303agr_from_fs_2g_hr_to_mg(
                             data_raw_acceleration[0] );
      acceleration_mg[1] = lsm303agr_from_fs_2g_hr_to_mg(
                             data_raw_acceleration[1] );
      acceleration_mg[2] = lsm303agr_from_fs_2g_hr_to_mg(
                             data_raw_acceleration[2] );
      printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      platform_delay(32);
      //tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
    }

    lsm303agr_mag_status_get(&dev_ctx_mg, &reg.status_reg_m);

    if (0/*reg.status_reg_m.zyxda*/) {
      /* Read magnetic field data */
      memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
      lsm303agr_magnetic_raw_get(&dev_ctx_mg, data_raw_magnetic);
      magnetic_mG[0] = lsm303agr_from_lsb_to_mgauss(
                         data_raw_magnetic[0]);
      magnetic_mG[1] = lsm303agr_from_lsb_to_mgauss(
                         data_raw_magnetic[1]);
      magnetic_mG[2] = lsm303agr_from_lsb_to_mgauss(
                         data_raw_magnetic[2]);
      printf("Magnetic field [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
              magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);
      platform_delay(32);
      //tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
    }

    lsm303agr_temp_data_ready_get(&dev_ctx_xl, &reg.byte);

    if (0/*reg.byte*/) {
      /* Read temperature data */
      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
      lsm303agr_temperature_raw_get(&dev_ctx_xl,
                                    &data_raw_temperature);
      temperature_degC = lsm303agr_from_lsb_hr_to_celsius(
                           data_raw_temperature );
      printf("Temperature [degC]:%6.2f\r\n",
              temperature_degC );
      platform_delay(32);
      //tx_com( tx_buffer, strlen( (char const *)tx_buffer ) );
    }
  }
}


/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
    ret_code_t err_code;
    sensbus_t *sensbus = (sensbus_t *)handle;

    /* Writing to LM75B_REG_CONF "0" set temperature sensor in NORMAL mode. */
    uint8_t d_reg[128]={0};

    #if 1
    /* enable auto incremented in multiple read/write commands */
    reg |= 0x80;
    #endif
    
    //safe check 
    if ( (len+1) > sizeof(d_reg))
      return -1; // error ...

    d_reg[0]=reg;

    memcpy(d_reg+1,bufp,len);
    err_code = nrf_drv_twi_tx(sensbus->hbus, sensbus->i2c_address, d_reg, len+1, false);
    APP_ERROR_CHECK(err_code);
    return 0;
}
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    ret_code_t err_code;
    sensbus_t *sensbus = (sensbus_t *)handle;
   
    #if 1
    /* enable auto incremented in multiple read/write commands */
    reg |= 0x80;
    #endif
    err_code = nrf_drv_twi_tx(sensbus->hbus, sensbus->i2c_address, &reg, 1, true);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_twi_rx(sensbus->hbus, sensbus->i2c_address,bufp,len);
    APP_ERROR_CHECK(err_code);

  return 0;
}
//static void tx_com(uint8_t *tx_buffer, uint16_t len);
static void platform_delay(uint32_t ms)
{
    nrf_delay_ms(ms);
}

static void platform_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lsm303_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lsm303_config, NULL /*twi_handler*/, NULL);
    //printf("ret=%x\r\n",(int)err_code);
    //nrf_delay_ms(500);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&m_twi);
    
}
/**
 * @brief Function for main application entry.
 */
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
    
    printf("TWI sensor example started.\r\n");
    nrf_delay_ms(50);

    platform_init();
    
    #ifdef LSM303_SELF_TEST
    lsm303agr_self_test();
    #else
    lsm303agr_read_data_polling();
    #endif
    
    while (true)
    {
        nrf_delay_ms(500);
        printf(".");
        
    }
}

/** @} */
