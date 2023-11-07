#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <stdio.h>

#include "ADG706.h"

// GPIO pin number where the input is connected
#define INPUT_PIN 21

// Define the GPIO pins to be used
#define EN_PIN  20   // Enable pin
#define A0_PIN  1   // Address 0
#define A1_PIN  2   // Address 1
#define A2_PIN  3   // Address 2
#define A3_PIN  0  // Address 3

#define EN_PIN_G  4   // Enable pin
#define A0_PIN_G  5   // Address 0
#define A1_PIN_G  6   // Address 1
#define A2_PIN_G  7   // Address 2
#define A3_PIN_G  14  // Address 3


static const uint16_t CAL_MASK = 0xffff;

// Define a pair of int values
typedef struct {
    int first;
    int second;
} IntPair;


// Define a key-value pair
typedef struct {
    unsigned int pairnumber;

  struct {
        int first;
        int second;
    } mbpair;    //this is the pair on the motherboard
  struct {
        int first;
        int second;
    } dbpair;    //this is the pair on the physical connector on the AMB



} KeyValuePair;


// Define a map as an array of key-value pairs. First is the dut, second is the ground ref


KeyValuePair bridgeMap[] = {
			    {0, {10,12}, {2,1}  }, //csbar+sdi 6+8, P2-P3
			    {1, {10,14}, {2,3}  }, //sdi sck 6+4, P2-P1
			    {2, {8,4}, {4,14}  }, //vcc vcc 2+7, P0-P11		
			{3, {9,15}, {3,4} }, //sck vcc 4+2, P1-P0	    
						    

				
};




  

// Define the size of the map based on the number of entries
const size_t intMapSize = sizeof(bridgeMap) / sizeof(bridgeMap[0]);


// Function to get a pair of values from the map given a key
/* IntPair* getValues(int key) { */
/*     for (size_t i = 0; i < intMapSize; ++i) { */
/*         if (bridgeMap[i].key == key) { */
/*             return &bridgeMap[i].values; */
/*         } */
/*     } */
/*     return NULL; // Key not found */
/* } */

// This function will be called when GPIO 5 goes high
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == INPUT_PIN) {
        // Your custom logic here
        printf("GPIO %d went high!\n", gpio);
    }
}

int main() {
    // Initialize stdio
    stdio_init_all();

    // Initialize the chosen GPIO pin
    gpio_init(INPUT_PIN);
    // Set the GPIO as input
    gpio_set_dir(INPUT_PIN, GPIO_IN);
    // Enable pull-down to make the input low by default
    gpio_pull_down(INPUT_PIN);

    // Setup the interrupt to trigger when GPIO goes high
    //    gpio_set_irq_enabled_with_callback(INPUT_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);


    ADG706 dut;
    ADG706 gndref;

    adg706_init(&dut,EN_PIN, A0_PIN, A1_PIN, A2_PIN, A3_PIN);
    adg706_init(&gndref,EN_PIN_G, A0_PIN_G, A1_PIN_G, A2_PIN_G, A3_PIN_G);

    adg706_set_enable(&dut,0);
    adg706_set_enable(&gndref,0);


    adg706_set_enable(&dut,1);
    adg706_set_enable(&gndref,1);

    
    

    // Wait forever
    while (1) {

      //cycle through all channels

      for (size_t i = 0; i < intMapSize; ++i) {
	int pin1 = bridgeMap[i].mbpair.first;
	int pin2 = bridgeMap[i].mbpair.second;

	adg706_set_address(&dut,pin1);
	adg706_set_address(&gndref,pin2);

	sleep_ms(10);
	
	bool pin_state = gpio_get(INPUT_PIN);

	if (i==2 && pin_state != 1) printf("Self check failed\n");
	  
	// Print the state of the pin
	printf("Pin state is: %s ", pin_state ? "High" : "Low");
	printf(" between pins %d and %d \n", bridgeMap[i].dbpair.first, bridgeMap[i].dbpair.second);
	sleep_ms(10);

	sleep_ms(100);
	  
      }
    }


    return 0;
}
