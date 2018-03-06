#ifndef __LEDS_USER_H__
#define __LEDS_USER_H__

#define LEDS_PORTB_LEDS       (LEDS_LED1)
#define LEDS_PORTD_LEDS       (LEDS_LED2)

#define LEDS_LED1        (1 << 0)
#define LEDS_LED2        (1 << 5)
#define LEDS_ALL_LEDS    (LEDS_LED1 | LEDS_LED2)
#define LEDS_NO_LEDS     0

static inline void LEDs_Init(void)
{
DDRD  |=  LEDS_PORTD_LEDS;
PORTD &= ~LEDS_PORTD_LEDS;
DDRB  |=  LEDS_PORTB_LEDS;
PORTB &= ~LEDS_PORTB_LEDS;
}

static inline void LEDs_SetAllLEDs(const uint8_t LEDMask)
{
PORTD = ((PORTD & ~LEDS_PORTD_LEDS) | ((~LEDMask) & LEDS_PORTD_LEDS));
PORTB = ((PORTB & ~LEDS_PORTB_LEDS) | ((~LEDMask) & LEDS_PORTB_LEDS));
}

#endif
