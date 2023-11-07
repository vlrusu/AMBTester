#include "pico/stdlib.h"
#include "ADG706.h"





// Function to initialize GPIO pins
void adg706_init(ADG706* self, int enablePin, int a0pin,int a1pin,int a2pin,int a3pin) {

  self->_enablePin = enablePin;
  self->_A0 = a0pin;
  self->_A1 = a1pin;
  self->_A2 = a2pin;
  self->_A3 = a3pin;
  
  // Initialize all pins as output and set them low
    gpio_init(self->_enablePin);
    gpio_set_dir(self->_enablePin, GPIO_OUT);
    gpio_put(self->_enablePin, 0);

    gpio_init(self->_A0);
    gpio_set_dir(self->_A0, GPIO_OUT);
    gpio_put(self->_A0, 0);


    gpio_init(self->_A1);
    gpio_set_dir(self->_A1, GPIO_OUT);
    gpio_put(self->_A1, 0);


    gpio_init(self->_A2);
    gpio_set_dir(self->_A2, GPIO_OUT);
    gpio_put(self->_A2, 0);


    gpio_init(self->_A3);
    gpio_set_dir(self->_A3, GPIO_OUT);
    gpio_put(self->_A3, 0);
    
}

// Function to set the address lines based on the desired channel
void adg706_set_address(ADG706* self, uint8_t address) {
    // Ensure the address is within the 4-bit range
    address &= 0x0F;

    // Set the address pins
    gpio_put(self->_A0, (address >> 0) & 1);
    gpio_put(self->_A1, (address >> 1) & 1);
    gpio_put(self->_A2, (address >> 2) & 1);
    gpio_put(self->_A3, (address >> 3) & 1);
}

// Function to enable or disable the multiplexer
void adg706_set_enable(ADG706* self, bool enable) {
    gpio_put(self->_enablePin, enable ? 1 : 0);
}


