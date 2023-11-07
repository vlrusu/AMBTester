#include "pico/stdlib.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ADG706_H
#define ADG706_H

typedef struct {
  int _enablePin;
  int _A0;
  int _A1;
  int _A2;
  int _A3;
} ADG706;



void adg706_init(ADG706 *self, int enablePin, int a0pin,int a1pin,int a2pin,int a3pin );
void adg706_set_address(ADG706 *self, uint8_t address);
void adg706_set_enable(ADG706 *self, bool enable);

#endif /* ADG706*/
