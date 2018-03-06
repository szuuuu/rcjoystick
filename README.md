# PPM Radio - USB Joystick Interface

Yet another PPM-USB interface, this one using [LUFA (Lightweight USB Framework for AVRs)](http://www.fourwalledcubicle.com/LUFA.php) and the ATmega32U4 chip (for convenience, cheap ready-made Arduino Pro Micro clone boards are available).

## Hardware

Connect the PPM signal from the transmitter to the D4 pin (and GND, of course). Pinouts for various transmitter brands are listed [here](http://www.mftech.de/buchsen_en.htm).

Arduino Pro Micro layout:

```
               |||||
               |usb|
           +---+---+---+
           |:SJ1       |
          -|TXO     RAW|-
          -|RXI     GND|---x optional
          -|GND   RESET|---x reset button
   o-------|GND     VCC|-
  PPM     -|D2       A3|-
 INPUT    -|D3       A2|-
   o-------|D4       A1|-
          -|D5       A0|-
          -|D6      SCK|-
          -|D7     MISO|-
          -|D8     MOSI|-
          -|D9      D10|-
           +-----------+ 
```

The board runs on the 16MHz crystal, which needs more than 3.3V power supply voltage according to the Atmel datasheet. Close the SJ1 jumper if you want 5V (but it turns out it also works on 3.3V, slightly outside of specs).

## Software

The code is based on the LUFA Joystick demo, retaining its VID/PID of 0x03EB/0x2043. This is only suitable for private use, no devices using these IDs may be released to the general public.
My modifications of the demo are meant for ATmega32U4. Presumably, it won't work on anything else without some adjustments.

The code is configured for 4 axes and 2 buttons, which works with my SpektrumDX6i radio and should be okay for most PPM-compatible transmitters.

If your radio needs different configuration, here are the places to tweak:
- #define NUM_TIMINGS 6 in RCJoystick.c - number of channels read from PPM
- CALLBACK_HID_Device_CreateHIDReport in RCJoystick.c - reorder or add joystick axes and buttons
- RCJoystickReport in Description.c -  HID report definition

HEX file: [RCJoystick.hex](./RCJoystick.hex)

## Compiling

Obviously, the gcc-avr toolchain is needed.
Download [LUFA](http://www.fourwalledcubicle.com/LUFA.php) and set its path in makefile (LUFA_PATH)
I placed RCJoystick inside LUFA folder so i have

        LUFA_PATH = ../LUFA

Tested with lufa-LUFA-170418.zip release, should work with any recent version.

AVRDUDE_PROGRAMMER is set to avr109, which is compatible with the bootloader normally installed in Arduino boards using the board's USB connector - no external programmer hardware is needed.


make -> compiling

make avrdude -> installing the compiled code

