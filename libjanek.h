#ifndef __LIBJANEK_H
#define __LIBJANEK_H
#include <stdint.h>
#include <stdbool.h>

#define ADS_ADDRESS (0x48 << 1)
extern uint8_t ADS_conf[];

void MMA_init(void);
void MMA_getdata(uint8_t* data);
void TMP_getdata(uint8_t* data);
void ADS_setconf(uint8_t new_conf);
void ADS_getdata(uint8_t* data);
void LED_init(void);
void LED_set(bool state);
bool LED_get(void);
void CHRG_init(void);
uint8_t CHRG_getstate(void);

#endif // __LIBJANEK_H
