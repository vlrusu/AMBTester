#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <stdio.h>
#include "hardware/adc.h"

#include "ADG706.h"

#include "DEV_Config.h"
#include "GUI_Paint.h"
#include "Debug.h"

#include "Infrared.h"

#include "LCD_1in14.h"

// GPIO pin number where the input is connected
#define INPUT_PIN 21
#define CHARGING_PIN 24
// Define the GPIO pins to be used
#define EN_PIN 20 // Enable pin
#define A0_PIN 1  // Address 0
#define A1_PIN 2  // Address 1
#define A2_PIN 3  // Address 2
#define A3_PIN 0  // Address 3

#define EN_PIN_G 4  // Enable pin
#define A0_PIN_G 5  // Address 0
#define A1_PIN_G 6  // Address 1
#define A2_PIN_G 7  // Address 2
#define A3_PIN_G 14 // Address 3

#define KEY_A 15
#define KEY_B 17

static const uint16_t CAL_MASK = 0xffff;

// Voltage divider ratio
#define DIVIDER_RATIO 3.0f

// ADC Reference Voltage
#define ADC_VREF 3.3f

// ADC Resolution
#define ADC_RESOLUTION (1 << 12) // 12-bit ADC

static const float full_battery = 4.2; // these are our reference voltages for a full / empty battery, in volts
static const float empty_battery = 2.8;

// Define a pair of int values
typedef struct
{
    int first;
    int second;
} IntPair;

// Define a key-value pair
typedef struct
{
    unsigned int pairnumber;

    struct
    {
        int first;
        int second;
    } mbpair; // this is the pair on the motherboard
    struct
    {
        int first;
        int second;
    } dbpair; // this is the pair on the physical connector on the AMB

} KeyValuePair;

// Define a map as an array of key-value pairs. First is the dut, second is the ground ref

KeyValuePair bridgeMap[] = {
    {0, {10, 12}, {2, 1}}, // csbar+sdi 6+8, P2-P3
    {1, {10, 14}, {2, 3}}, // sdi sck 6+4, P2-P1
    {2, {8, 4}, {4, 14}},  // vcc vcc 2+7, P0-P11
    {3, {9, 15}, {3, 4}},  // sck vcc 4+2, P1-P0

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

void init_adc_to_read_vsys()
{
    adc_init();
    // Enable ADC for GPIO 29 (ADC3) which is connected to VSYS/3
    adc_gpio_init(29);

    // Select ADC input 3 (GPIO 29)
    adc_select_input(3);
}

float read_vsys_voltage()
{
    // Read the raw ADC value and convert it to voltage
    uint16_t raw = adc_read();
    float voltage = (raw * ADC_VREF) / ADC_RESOLUTION;

    // Multiply by the divider ratio to get the system voltage
    return voltage * DIVIDER_RATIO;
}

// This function will be called when GPIO 5 goes high
void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == INPUT_PIN)
    {
        // Your custom logic here
        printf("GPIO %d went high!\n", gpio);
    }
}

int main()
{
    // Initialize stdio
    stdio_init_all();

    // Initialize the chosen GPIO pin
    gpio_init(INPUT_PIN);
    gpio_init(CHARGING_PIN);
    // Set the GPIO as input
    gpio_set_dir(INPUT_PIN, GPIO_IN);
    gpio_set_dir(CHARGING_PIN, GPIO_IN);
    // Enable pull-down to make the input low by default
    gpio_pull_down(INPUT_PIN);

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);

    // Setup the interrupt to trigger when GPIO goes high
    //    gpio_set_irq_enabled_with_callback(INPUT_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    ADG706 dut;
    ADG706 gndref;

    adg706_init(&dut, EN_PIN, A0_PIN, A1_PIN, A2_PIN, A3_PIN);
    adg706_init(&gndref, EN_PIN_G, A0_PIN_G, A1_PIN_G, A2_PIN_G, A3_PIN_G);

    adg706_set_enable(&dut, 0);
    adg706_set_enable(&gndref, 0);

    adg706_set_enable(&dut, 1);
    adg706_set_enable(&gndref, 1);

    // Initialize ADC
    init_adc_to_read_vsys();

    DEV_Delay_ms(100);
    printf("LCD_1in14_test Demo\r\n");
    if (DEV_Module_Init() != 0)
    {
        return -1;
    }
    DEV_SET_PWM(50);
    /* LCD Init */
    printf("1.14inch LCD demo...\r\n");
    LCD_1IN14_Init(HORIZONTAL);
    LCD_1IN14_Clear(WHITE);

    // LCD_SetBacklight(1023);
    UDOUBLE Imagesize = (LCD_1IN14_HEIGHT)*LCD_1IN14_WIDTH * 2;
    UWORD *BlackImage;
    if ((BlackImage = (UWORD *)malloc(Imagesize)) == NULL)
    {
        printf("Failed to apply for black memory...\r\n");
        exit(0);
    }
    // /*1.Create a new image cache named IMAGE_RGB and fill it with white*/
    Paint_NewImage((UBYTE *)BlackImage, LCD_1IN14.WIDTH, LCD_1IN14.HEIGHT, 0, RED);
    Paint_SetScale(65);
    Paint_Clear(WHITE);
    Paint_SetRotate(ROTATE_0);
    Paint_Clear(WHITE);

    SET_Infrared_PIN(KEY_A);
    SET_Infrared_PIN(KEY_B);

    // Wait forever
    while (1)
    {

        if (DEV_Digital_Read(KEY_A) == 0)
        {
            Paint_ClearWindows(1, 20, LCD_1IN14.WIDTH, LCD_1IN14.HEIGHT, WHITE);
            Paint_DrawString_EN(1, 100, "Set for AMBCAL", &Font20, 0x000f, 0xfff0);
            cleartogo = 0;
            LCD_1IN14_Display(BlackImage);
            printf("gpio =%d\r\n", KEY_A);
            DEV_Delay_ms(1000);
            // set for AMBCAL
        }

        // wait for selfcheck to go low
        adg706_set_address(&dut, bridgeMap[0].mbpair.first);
        adg706_set_address(&gndref, bridgeMap[0].mbpair.second);

        sleep_ms(10);

        bool pin_state = gpio_get(INPUT_PIN);
        if (pin_state == 0)
        {
            Paint_ClearWindows(1, 20, LCD_1IN14.WIDTH, LCD_1IN14.HEIGHT, RED);
            LCD_1IN14_Display(BlackImage);
            cleartogo = 1;
            continue;
        }

        if (cleartogo == 1)
        {
            if (pin_state == 0)
                continue; // don't do anything until self check

            // cycle through all channels except first

            for (size_t i = 1; i < intMapSize; ++i)
            {
                int pin1 = bridgeMap[i].mbpair.first;
                int pin2 = bridgeMap[i].mbpair.second;

                adg706_set_address(&dut, pin1);
                adg706_set_address(&gndref, pin2);

                sleep_ms(10);

                bool pin_state = gpio_get(INPUT_PIN);
                bool charging_state = gpio_get(CHARGING_PIN);

                // Print the state of the pin
                printf("Pin state is: %s ", pin_state ? "High" : "Low");
                printf(" between pins %d and %d \n", bridgeMap[i].dbpair.first, bridgeMap[i].dbpair.second);
                sleep_ms(10);

                if (pin_state == 1)
                {
                    Paint_ClearWindows(1, 20, LCD_1IN14.WIDTH, LCD_1IN14.HEIGHT, RED);
                    Paint_DrawString_EN(1, 50, "Bridge detected", &Font20, 0x000f, 0xfff0);
                    LCD_1IN14_Display(BlackImage);
                    cleartogo = 0;
                    break;
                }

                // Read VSYS voltage
                float vsys_voltage = read_vsys_voltage();
                char str_float[20]; // Enough space for a float and null-terminator
                sprintf(str_float, "%.2f", vsys_voltage);
                printf("VSYS Voltage: %.2fV %s\n", vsys_voltage, charging_state ? "Charging" : "not charging");
                Paint_DrawString_EN(1, 1, str_float, &Font20, 0x000f, 0xfff0);

                if (vsys_voltage < empty_battery)
                {
                    Paint_DrawString_EN(100, 1, "Low batt", &Font20, 0x000f, 0xfff0);
                }
                LCD_1IN14_Display(BlackImage);

                sleep_ms(1000);

                if (i == intMapSize-1){            //if we got here turn allto green

                    //turn to green
                    cleartogo=0;
                }
            }

            
        }
    }

    return 0;
}
