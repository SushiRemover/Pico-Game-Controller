#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#define SW_GPIO_SIZE 8                  // Number of switches
#define LED_GPIO_SIZE 7                 // Number of switch LEDs
#define ENC_GPIO_SIZE 2                 // Number of encoders
#define ENC_PPR 24                     // Encoder PPR
#define ENC_DEBOUNCE true               // Encoder Debouncing
#define ENC_PULSE (ENC_PPR * 4)         // 4 pulses per PPR
#define ENC_IPK 5                       // Encoder impulses per Keypress
#define SW_MIN_HOLD (5 * 1000)          // Minimum time the firmware hold a switch after pressing
#define SW_MIN_PAUSE (5 * 1000)         // Minimum time the firmware will keep a switch at state false after pressing
#define KEYPULSE_DURATION (10 * 1000)   // Keypulse duration in microseconds
#define REACTIVE_TIMEOUT_MAX 1000000  // HID to reactive timeout in us
#define KEYPULSE_DELAY (10 * 1000)      // Keypulse delay in microseconds
#define WS2812B_LED_SIZE 10             // Number of WS2812B LEDs
#define WS2812B_LED_ZONES 2             // Number of WS2812B LED Zones
#define WS2812B_LEDS_PER_ZONE \
  WS2812B_LED_SIZE / WS2812B_LED_ZONES  // Number of LEDs per zone

#ifdef PICO_GAME_CONTROLLER_C

// MODIFY KEYBINDS HERE, MAKE SURE LENGTHS MATCH SW_GPIO_SIZE
const uint8_t MD1_SW_KEYCODE[] =     {HID_KEY_D, HID_KEY_F, HID_KEY_J, HID_KEY_K,
                                  HID_KEY_C, HID_KEY_M, HID_KEY_1, HID_KEY_G};

const uint8_t MD2_SW_KEYCODE[] = {HID_KEY_D, HID_KEY_F, HID_KEY_J, HID_KEY_K,
                                  HID_KEY_SPACE, HID_KEY_P, HID_KEY_ENTER, HID_KEY_ESCAPE};
const uint8_t MD2_ENC_KEYCODE[] = {HID_KEY_ARROW_RIGHT, HID_KEY_ARROW_LEFT, HID_KEY_ARROW_DOWN, HID_KEY_ARROW_UP};

const uint8_t SW_GPIO[] = {
    4, 6, 8, 10, 12, 14, 20, 27,
};
const uint8_t LED_GPIO[] = {
    5, 7, 9, 11, 13, 15, 21,
};
const uint8_t ENC_GPIO[] = {0, 2};      // L_ENC(0, 1); R_ENC(2, 3)
const bool ENC_REV[] = {false, false};  // Reverse Encoders
const uint8_t WS2812B_GPIO = 28;

#endif

extern bool joy_mode_check;

#endif