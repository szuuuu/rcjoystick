/*
             LUFA Library
     Copyright (C) Dean Camera, 2017.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2017  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the Joystick demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "RCJoystick.h"
#include <util/delay.h>

/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevRCJoystickHIDReportBuffer[sizeof(USB_RCJoystickReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t RCJoystick_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = INTERFACE_ID_RCJoystick,
				.ReportINEndpoint             =
					{
						.Address              = JOYSTICK_EPADDR,
						.Size                 = JOYSTICK_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevRCJoystickHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevRCJoystickHIDReportBuffer),
			},
	};

#define MILLISEC(t) (16*(t))

#define NUM_TIMINGS 6
uint8_t current_index;
uint8_t waiting;
uint16_t timings[NUM_TIMINGS];

#define MAX_WAITING 20

enum LedPattern { LedUSBNotReady=0,
		  LedUSBEnumerating,
		  LedUSBError,
		  LedWaiting,
		  LedOK };

#define LED_PATTERN_SIZE 8
uint8_t led_patterns[][LED_PATTERN_SIZE]=
 {
  { LEDS_LED1,0,0,0,0,0,0,0}, // USBNotReady 
  { LEDS_LED1,0,LEDS_LED1,0,LEDS_LED1,0,LEDS_LED1,0}, // USBEnumerating
  { LEDS_LED1+LEDS_LED2,0,0,0,LEDS_LED1+LEDS_LED2,0,0,0}, // USBError
  { LEDS_LED1,LEDS_LED2,LEDS_LED1,0,0,0,0,0}, // Waiting
  { LEDS_LED1,LEDS_LED1+LEDS_LED2,LEDS_LED2,LEDS_LED1+LEDS_LED2,LEDS_LED1,LEDS_LED1+LEDS_LED2,LEDS_LED2,LEDS_LED1+LEDS_LED2}, // OK
 };
enum LedPattern led_state=0;
uint8_t led_time=0;
uint8_t usb_state=0;
uint8_t led_tick=0;

void setLeds(enum LedPattern p)
{
led_state=p;
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */

int main(void)
{
	SetupHardware();
	current_index=NUM_TIMINGS;
	waiting=1;
	for(uint8_t i=0;i<NUM_TIMINGS;i++)
		timings[i]=MILLISEC(1500);
	
	GlobalInterruptEnable();

	for (;;)
	{
		if (!(++led_tick & 127))
			LEDs_SetAllLEDs(led_patterns[led_state][++led_time&(LED_PATTERN_SIZE-1)]);
		_delay_ms(1);
		HID_Device_USBTask(&RCJoystick_HID_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
	
	LEDs_Init();
	USB_Init();

	TCCR1A = 0; //normal mode, no output capture
	TCCR1B = (1<<ICNC1) | (0<<ICES1) | (1<<CS10); //normal mode, input capture falling, input capture filter, div/1 = 16 MHz (res 0.06us  max 4ms)
	TIMSK1 = (1<<ICIE1) | (1<<TOIE1); //input capture interrupt, overflow interrupt
	PORTD |= (1<<4); //pull up
}

ISR(TIMER1_OVF_vect)
{
if (waiting<MAX_WAITING)
	{
	waiting++;
	}
else
	{
	waiting=1;
	if (usb_state) setLeds(LedWaiting);
	}
}

ISR(TIMER1_CAPT_vect)
{
TCNT1=0;
uint16_t t=ICR1;
if (waiting)
	{
	waiting=0;
	if (usb_state) setLeds(LedOK);
	current_index=0;
	}
else if (current_index<NUM_TIMINGS)
	{
	timings[current_index]=t;
	current_index++;
	}
}

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

int16_t axisFromTime(uint16_t t)
{
t=max(MILLISEC(1000),t);
t=min(MILLISEC(2000),t);
return ((int16_t)t)-MILLISEC(1500);
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	setLeds(LedUSBEnumerating);
	usb_state=0;
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	setLeds(LedUSBNotReady);
	usb_state=0;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&RCJoystick_HID_Interface);

	USB_Device_EnableSOFEvents();

	if (ConfigSuccess)
		{
		setLeds(LedWaiting);
		usb_state=1;
		}
	else
		{
		setLeds(LedUSBError);
		usb_state=0;
		}
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&RCJoystick_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&RCJoystick_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	USB_RCJoystickReport_Data_t* rep = (USB_RCJoystickReport_Data_t*)ReportData;

	// my radio transmits: yaw roll pitch throttle
	rep->axes[0] = axisFromTime(timings[0]);
	rep->axes[1] = axisFromTime(timings[3]);
	rep->axes[2] = axisFromTime(timings[1]);
	rep->axes[3] = axisFromTime(timings[2]);
	
	rep->buttons = ((timings[4] < MILLISEC(1500)) ? 1 : 0)
		+ ((timings[5] < MILLISEC(1500)) ? 2 : 0);

	*ReportSize = sizeof(USB_RCJoystickReport_Data_t);
	return false;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	// Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports
}

