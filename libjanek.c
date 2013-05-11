/* Copyright (c) 2013 Janek Mann
*/

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "twi_master.h"

#include "libjanek.h"

#define MMA_ADDRESS 0x38

static uint8_t mma_init_cmd[] = {0x2a, 0x2d};
static uint8_t mma_init_cmd2[] = {0x2b, 0x03}; // Low Power
static uint8_t mma_reg_x[] = { 0x01 };
static uint8_t mma_reg_who_am_i[] = { 0x0d };

void MMA_init(void) {
		uint8_t data2[2];
		twi_master_transfer(MMA_ADDRESS, mma_init_cmd, 2, TWI_ISSUE_STOP);
        twi_master_transfer(MMA_ADDRESS, mma_init_cmd2, 2, TWI_ISSUE_STOP);
		twi_master_transfer(MMA_ADDRESS, mma_reg_who_am_i, 1, TWI_DONT_ISSUE_STOP);
		twi_master_transfer(MMA_ADDRESS | TWI_READ_BIT, data2, 1, TWI_ISSUE_STOP);
}

void MMA_getdata(uint8_t* data) {
		twi_master_transfer(MMA_ADDRESS, mma_reg_x, 1, TWI_DONT_ISSUE_STOP);
		twi_master_transfer(MMA_ADDRESS | TWI_READ_BIT, data, 6, TWI_ISSUE_STOP);
}		

#define TMP_ADDRESS (0x49 << 1)

void TMP_getdata(uint8_t* data) {
		twi_master_transfer(TMP_ADDRESS | TWI_READ_BIT, data, 2, TWI_ISSUE_STOP);
}

uint8_t ADS_conf[] = { 0x0c };

void ADS_setconf(uint8_t new_conf) {
		if (new_conf != ADS_conf[0]) {
				ADS_conf[0] = new_conf & 0x0F;
				twi_master_transfer(ADS_ADDRESS, ADS_conf, 1, TWI_ISSUE_STOP);
		}
}

void ADS_getdata(uint8_t* data) {
		twi_master_transfer(ADS_ADDRESS | TWI_READ_BIT, data, 3, TWI_ISSUE_STOP);
}

#define LED_PIN 0

void LED_init(void) {
		nrf_gpio_pin_write(LED_PIN, 0);
		nrf_gpio_cfg_output(LED_PIN);
}

void LED_set(bool state) {
		nrf_gpio_pin_write(LED_PIN, state);
}

bool LED_get(void) {
    return nrf_gpio_pin_read(LED_PIN);
}

#define CHRG_OUT_PIN 25
#define CHRG_IN_PIN  24

void CHRG_init(void) {
    nrf_gpio_cfg_input(CHRG_OUT_PIN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(CHRG_IN_PIN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_pin_write(CHRG_OUT_PIN, 1);
}

uint8_t CHRG_getstate(void) {
    uint8_t result = 0;
    result = nrf_gpio_pin_read(CHRG_IN_PIN);
    nrf_gpio_cfg_output(CHRG_OUT_PIN);
    result |= (nrf_gpio_pin_read(CHRG_IN_PIN)<<1);
    nrf_gpio_cfg_input(CHRG_OUT_PIN, NRF_GPIO_PIN_NOPULL);
    return result;
}
