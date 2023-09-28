#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <stdio.h>
#include "hardware/adc.h"

#include "pico/multicore.h"
#include "pico/multicore.h"

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

#define PWM_PIN 16

static const uint16_t CAL_MASK = 0xffff;

#define AIRCR_Register (*((volatile uint32_t *)(PPB_BASE + 0x0ED0C)))

// Voltage divider ratio
#define DIVIDER_RATIO 3.0f

// ADC Reference Voltage
#define ADC_VREF 3.3f

// ADC Resolution
#define ADC_RESOLUTION (1 << 12) // 12-bit ADC

static const float full_battery = 4.9; // these are our reference voltages for a full / empty battery, in volts
static const float empty_battery = 2.8;

static bool cleartogo;

UWORD *BlackImage;
#define TOPCOLORWINDOW 50

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

// CAL side
KeyValuePair bridgeMapCAL[] = {
    {0, {8, 4}, {4, 14}},   // vcc vcc 2+7, P0-P11 self check
    {1, {5, 1}, {16, 17}},  // calsiga vcalab 11+13, P13-P14
    {2, {4, 2}, {15, 16}},  // calsigb calsiga 9+11, P12-P13
    {3, {3, 3}, {14, 15}},  // vcc calsigb 7+9, P11-P12
    {4, {3, 5}, {14, 13}},  // vcc gndcal 7+5, P11-P10
    {5, {0, 6}, {11, 12}},  // sigbp gnd 1+3, P8-P9
    {6, {12, 7}, {10, 11}}, // sigbn sigbp 10+1, P4-P8
    {7, {12, 6}, {10, 9}},  // sigbn gnd 10+3, P4-P9
    {8, {13, 6}, {7, 8}},   // sigap gnd 12+3, P5-P9
    {9, {14, 10}, {6, 7}},  // sigan sigap 14+12, P6-P5
    {10, {14, 6}, {6, 5}},  // sigan gnd 14+3, P6-P9
    {11, {8, 6}, {4, 5}},   // vcc gnd 2+3, P0-P9
    {12, {9, 15}, {3, 4}},  // sck vcc 4+2, P1-P0
    {13, {10, 14}, {2, 3}}, // sdi sck 6+4, P2-P1
    {14, {11, 13}, {1, 2}}, // cs sdi 8+6, P3-P2
};

KeyValuePair bridgeMapHV[] = {
    {0, {10, 12}, {2, 1}}, // csbar+sdi 6+8, P2-P3
    {1, {10, 14}, {2, 3}}, // sdi sck 6+4, P2-P1
    {2, {8, 4}, {4, 14}},  // vcc vcc 2+7, P0-P11

};

// Define the size of the map based on the number of entries
const size_t intMapSizeCAL = sizeof(bridgeMapCAL) / sizeof(bridgeMapCAL[0]);
const size_t intMapSizeHV = sizeof(bridgeMapHV) / sizeof(bridgeMapHV[0]);

KeyValuePair *bridgeMap;
size_t intMapSize;

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

void AMBCAL_setup()
{
    Paint_ClearWindows(1, TOPCOLORWINDOW, LCD_1IN14.WIDTH, LCD_1IN14.HEIGHT, WHITE);
    Paint_DrawString_EN(1, 20, "Set for AMBCAL", &Font20, 0x000f, 0xfff0);
    // AIRCR_Register = 0x5FA0004;
    cleartogo = 0;
    bridgeMap = bridgeMapCAL;
    intMapSize = intMapSizeCAL;
    LCD_1IN14_Display(BlackImage);
}

void AMBHV_setup()
{

    Paint_ClearWindows(1, TOPCOLORWINDOW, LCD_1IN14.WIDTH, LCD_1IN14.HEIGHT, WHITE);
    Paint_DrawString_EN(1, 20, "Set for  AMBHV", &Font20, 0x000f, 0xfff0);
    // AIRCR_Register = 0x5FA0004;
    cleartogo = 0;
    bridgeMap = bridgeMapHV;
    intMapSize = intMapSizeHV;
    LCD_1IN14_Display(BlackImage);
}

// This function will be called when GPIO 5 goes high
void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == KEY_A)
    {
        // Your custom logic here
        AMBCAL_setup();

        printf("gpio =%d\r\n", KEY_A);
    }

    else if (gpio == KEY_B)
    {
        AMBHV_setup();
        printf("gpio =%d\r\n", KEY_B);
    }
}

// Function to run on core 1
/* void core1_entry()
{

    while (1)
    {
        // Read VSYS voltage
        float vsys_voltage = read_vsys_voltage();
        char str_float[20]; // Enough space for a float and null-terminator
        sprintf(str_float, "%.2f", vsys_voltage);
        printf("VSYS Voltage: %.2f \n", vsys_voltage);
        Paint_DrawString_EN(1, 1, str_float, &Font20, 0x000f, 0xfff0);

        if (vsys_voltage < empty_battery)
        {
            Paint_DrawString_EN(100, 1, "Low batt", &Font20, 0x000f, 0xfff0);
        }
        LCD_1IN14_Display(BlackImage);

        sleep_ms(1000);
    }
} */

int main()
{
    // Initialize stdio
    stdio_init_all();

    // Tell GPIO 16 to use the PWM functions
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 16 (each slice controls two PWM channels).
    // The channel is either 0 or 1, with 0 being the A side and 1 being the B side of the slice.
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);

    // Set the PWM frequency
    // The frequency is determined by the clock divider and the wrap value.
    // The default clock is 125 MHz. A clock divider of 125 will divide it by 125, to 1 MHz.
    // A wrap value of 1000 will cause the counter to reset at 10000, thus 1 MHz / 100 = 10 kHz PWM frequency.
    pwm_set_clkdiv(slice_num, 125.0);

    pwm_set_wrap(slice_num, 100);

    // Set the PWM level; setting it to half the wrap value will give a 50% duty cycle.
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    pwm_set_enabled(slice_num, true);

    cleartogo = 0;

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

    ADG706 dut;
    ADG706 gndref;

    adg706_init(&dut, EN_PIN, A0_PIN, A1_PIN, A2_PIN, A3_PIN);
    adg706_init(&gndref, EN_PIN_G, A0_PIN_G, A1_PIN_G, A2_PIN_G, A3_PIN_G);

    adg706_set_enable(&dut, 0);
    adg706_set_enable(&gndref, 0);

    adg706_set_enable(&dut, 1);
    adg706_set_enable(&gndref, 1);

    AMBCAL_setup();

    // Launch core1_entry() on core 1
    //    multicore_launch_core1(core1_entry);

    // Set up the interrupt to trigger when the button goes from high to low (a falling edge),
    // which corresponds to the button being pressed
    gpio_set_irq_enabled_with_callback(KEY_A, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(KEY_B, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);


    
    // Wait forever
    while (1)
    {
        // Read VSYS voltage
        float vsys_voltage = read_vsys_voltage();
        char str_float[20]; // Enough space for a float and null-terminator
        sprintf(str_float, "%d%%", (int)(100 * vsys_voltage / full_battery));
        printf("VSYS Voltage: %.2f \n", vsys_voltage);
        Paint_DrawString_EN(1, 1, str_float, &Font20, 0x000f, 0xfff0);

        if (vsys_voltage < empty_battery)
        {
            Paint_DrawString_EN(100, 1, "Low batt", &Font20, 0x000f, 0xfff0);
        }
        LCD_1IN14_Display(BlackImage);

        // wait for selfcheck to go low
        adg706_set_address(&dut, bridgeMap[0].mbpair.first);
        adg706_set_address(&gndref, bridgeMap[0].mbpair.second);
        sleep_ms(150);

        bool pin_state = gpio_get(INPUT_PIN);

        if (pin_state == 0)
        {
            Paint_ClearWindows(1, TOPCOLORWINDOW, LCD_1IN14.WIDTH, LCD_1IN14.HEIGHT, YELLOW);
            Paint_DrawString_EN(1, 80, "Waiting for AMB", &Font20, 0x000f, 0xfff0);
            LCD_1IN14_Display(BlackImage);
            cleartogo = 1;
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

                sleep_ms(150);

                bool pin_state = gpio_get(INPUT_PIN);

                // Print the state of the pin
                printf("Pin state is: %s ", pin_state ? "High" : "Low");
                printf(" between pins %d and %d \n", bridgeMap[i].dbpair.first, bridgeMap[i].dbpair.second);

                if (pin_state == 1)
                {
                    Paint_ClearWindows(1, TOPCOLORWINDOW, LCD_1IN14.WIDTH, LCD_1IN14.HEIGHT, RED);
                    sprintf(str_float, "Bridge  %d - %d", bridgeMap[i].dbpair.first, bridgeMap[i].dbpair.second);
                    Paint_DrawString_EN(1, 70, str_float, &Font20, 0x000f, 0xfff0);
                    LCD_1IN14_Display(BlackImage);
                    pwm_set_chan_level(slice_num, PWM_CHAN_A, 5000);

                    sleep_ms(3000);

                    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

                    cleartogo = 0;
                    break;
                }

                if (i == intMapSize - 1)
                { // if we got here turn allto green

                    // turn to green
                    Paint_ClearWindows(1, TOPCOLORWINDOW, LCD_1IN14.WIDTH, LCD_1IN14.HEIGHT, GREEN);
                    Paint_DrawString_EN(1, 70, "All ok", &Font20, 0x000f, 0xfff0);
                    printf("All ok");
                    LCD_1IN14_Display(BlackImage);

                    pwm_set_chan_level(slice_num, PWM_CHAN_A, 5000);
                    sleep_ms(100);
                    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
                    sleep_ms(100);
                    pwm_set_chan_level(slice_num, PWM_CHAN_A, 5000);
                    sleep_ms(100);
                    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

                    cleartogo = 0;
                }
            }
        }
    }
    

    return 0;
}
