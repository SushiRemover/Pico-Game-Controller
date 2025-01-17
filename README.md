Forked from https://github.com/speedypotato/Pico-Game-Controller

I changed the Firmware to fit my needs, not sain that I can write clean or effecient code...

Features of my Version:
- New mode, Alternative_Keyboard-Mode:
  - Vol-L act as Arrow Left and Right
  - Vol-R act as Arrow Up and Down
  - Keybindings (incl. Vol-L and Vol-R) can be changed in controller_config.h and are seperate to Standart Keyboard/Mouse-Mode
  - Delay and Duration of the Keypulse as well as the impules from the encoder needed to start a pulse can also be set in controller_config.h
- New Feature, Modechanger
  - Can be opened with FX-L + FX-R + Push_Btn at the same time
  - Can switch between modes without needing to replug the con
  - MD1 Keyboardmode with Encoders as Mouse (Press BT-A for direct Selection or select with FX and apply with Push_Btn)
  - MD2 Keyboardmode with Encoders as Arrowkeys (Press BT-B for direct Selection or select with FX and apply with Push_Btn)
  - MD4 Joypad-Mode (Press BT-D for direct Selection or select with FX and apply with Push_Btn)
- Debouncing of all switches with no added latency, if a switch gets triggerg it will be at least 5ms hold on pressed and after that a pause of 5ms to prevent bouncing.
  - Times can be changed in controller_config.h

Note: Con does NOT select a Mode automatically, if you are in the Mode-Selector both FX Leds are on and the bt-a to bt-d leds are showing the mode you are selecting.

How to select modes (same as original):
- Switch between modes while in menu with FX-R/FX-L and apply with Push button
- Select modes directly, BT-A for MD1 (Keyboard), BT-B for MD2 (Alt-Keyboard) and BT-D for MD4 (Joypad).

Newest build is available in Pico-Game-Controller/build_uf2/Pico_Game_Controller.uf2

# Original README
Code for a keyboard or game controller using a Raspberry Pi Pico. Capable of handling 11 buttons, 10 LEDs, 1 WS2812B RGB strip, and 2 encoders.  Developed with SDVX and IIDX in mind - see branches release/pocket-sdvx-pico and release/pocket-iidx for preconfigured versions.

Demo of this firmware running on Pocket SDVX Pico, purchasable at https://discord.gg/MmuKd73XbY

![Pocket SDVX Pico](demo.gif)

Currently working/fixed:

- Gamepad mode - default boot mode
- NKRO Keyboard & Mouse Mode - hold first button(gpio4) to enter kb mode
- HID LEDs with Reactive LED fallback
- ws2812b rgb on second core
- 2 ws2812b hid descriptor zones
- sdvx/iidx spoof - Tested on EAC - checkout branches release/pocket-sdvx-pico or release/pocket-iidx
- 1000hz polling
- Reversable Encoders with debouncing
- Switch and LED pins are now staggered for easier wiring
- Fix 0-~71% encoder rollover in gamepad mode, uint32 max val isn't divisible evenly by ppr\*4 for joystick - thanks friends
- HID LEDs now have labels, thanks CrazyRedMachine

TODO:

- refactor ws2812b into a seperate file for cleaner code & implement more RGB modes
- store configuration settings in a text file? consider implementing littlefs https://github.com/littlefs-project/littlefs https://www.raspberrypi.org/forums/viewtopic.php?t=313009 https://www.raspberrypi.org/forums/viewtopic.php?p=1894014#p1894014
- Store last mode in flash memory (probably implement into above TODO if possible) https://www.raspberrypi.org/forums/viewtopic.php?t=305570
- debounce on switches

How to Use:

- For basic flashing, see README in build_uf2
- Otherwise, setup the C++ environment for the Pi Pico as per https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf
- Build pico-examples directory once to ensure all the tinyusb and other libraries are there. You might have to move the pico-sdk folder into pico-examples/build for it to play nice.
- Move pico-sdk back outside to the same level directory as Pico-Game-Controller.
- Open Pico-Game-Controller in VSCode(assuming this is setup for the Pi Pico) and see if everything builds.
- Tweakable parameters are in controller_config.h

Thanks to:

- Everyone in the Cons & Stuff Discord for providing near instant support to questions.
- https://github.com/hathach/tinyusb/tree/master/examples/device/hid_composite
- https://github.com/mdxtinkernick/pico_encoders for encoders which performed better than both interrupts and polling.
- My SE buddies who helped come up with a solution for the encoder rollover edge case scenario.
- https://github.com/Drewol/rp2040-gamecon for usb gamepad descriptor info.
- https://github.com/veroxzik/arduino-konami-spoof for konami spoof usb descriptor info.
- https://github.com/veroxzik/roxy-firmware for nkro descriptor and logic info.
- KyubiFox for bringing clkdiv to my attention for encoder debouncing
