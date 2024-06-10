/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "bme68x.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "common.h"

/******************************************************************************/
/*!                 Macro definitions                                         */
#define BME68X_I2C          i2c0
#define BME68X_I2C_HZ       (100 * 1000)
#define BME68X_SCL_GPIO     17
#define BME68X_SDA_GPIO     16

/******************************************************************************/
/*!                Static variable definition                                 */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    /* Send reg address first without stop bit */
    i2c_write_blocking(BME68X_I2C, device_addr, &reg_addr, 1, true);

    return !i2c_read_blocking(BME68X_I2C, device_addr, reg_data, (size_t)len, false);
}

/*!
 * I2C write function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    uint8_t write_buf[BME68X_LEN_INTERLEAVE_BUFF] = {0};

    /* Combine reg_addr and reg_data into one packet */
    write_buf[0] = reg_addr;
    memcpy(&write_buf[1], reg_data, len);

    return !i2c_write_blocking(BME68X_I2C, device_addr, write_buf, (size_t)len + 1, false);
}

// /*!
//  * SPI read function map to COINES platform
//  */
// BME68X_INTF_RET_TYPE bme68x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
// {
//     uint8_t device_addr = *(uint8_t*)intf_ptr;

//     (void)intf_ptr;

//     return coines_read_spi(COINES_SPI_BUS_0, device_addr, reg_addr, reg_data, (uint16_t)len);
// }

// /*!
//  * SPI write function map to COINES platform
//  */
// BME68X_INTF_RET_TYPE bme68x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
// {
//     uint8_t device_addr = *(uint8_t*)intf_ptr;

//     (void)intf_ptr;

//     return coines_write_spi(COINES_SPI_BUS_0, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
// }

/*!
 * Delay function map to COINES platform
 */
void bme68x_delay_us(uint32_t period, __attribute__((unused)) void *intf_ptr)
{
    busy_wait_us_32(period);
}

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:

            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
            printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BME68X_E_COM_FAIL:
            printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BME68X_E_INVALID_LENGTH:
            printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BME68X_E_DEV_NOT_FOUND:
            printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BME68X_E_SELF_TEST:
            printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            break;
        case BME68X_W_NO_NEW_DATA:
            printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            break;
        default:
            printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf)
{
    int8_t rslt = BME68X_OK;

    if (bme != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BME68X_I2C_INTF)
        {
            printf("I2C Interface\n");
            dev_addr = BME68X_I2C_ADDR_HIGH;
            bme->read = bme68x_i2c_read;
            bme->write = bme68x_i2c_write;
            bme->intf = BME68X_I2C_INTF;

            /* SDO pin is made low (Adafruit board is pulled high) */

            /* Initialize I2C interface */
            i2c_init(BME68X_I2C, BME68X_I2C_HZ);
            gpio_set_function(BME68X_SDA_GPIO, GPIO_FUNC_I2C);
            gpio_set_function(BME68X_SCL_GPIO, GPIO_FUNC_I2C);
            gpio_pull_up(BME68X_SDA_GPIO);
            gpio_pull_up(BME68X_SCL_GPIO);

        }
        /* Bus configuration : SPI */
        else if (intf == BME68X_SPI_INTF)
        {
            // printf("SPI Interface\n");
            // dev_addr = COINES_SHUTTLE_PIN_7;
            // bme->read = bme68x_spi_read;
            // bme->write = bme68x_spi_write;
            // bme->intf = BME68X_SPI_INTF;
            // (void)coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_7_5_MHZ, COINES_SPI_MODE0);
            assert("BME68X SPI Interface not supported.");
        }

        bme->delay_us = bme68x_delay_us;
        bme->intf_ptr = &dev_addr;
        bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
    }
    else
    {
        rslt = BME68X_E_NULL_PTR;
    }

    return rslt;
}

void bme68x_coines_deinit(void)
{
    (void)fflush(stdout);

    /* Disable I2C */
    i2c_deinit(BME68X_I2C);
}