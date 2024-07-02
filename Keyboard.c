/*
             LUFA Library
     Copyright (C) Dean Camera, 2021.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2021  Dean Camera (dean [at] fourwalledcubicle [dot] com)

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

#include "Keyboard.h"

/** Buffer to hold the previously generated Keyboard HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevKeyboardHIDReportBuffer[sizeof(USB_KeyboardReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Keyboard_HID_Interface =
  {
    .Config = {
      .InterfaceNumber = INTERFACE_ID_Keyboard,
      .ReportINEndpoint = {
	.Address = KEYBOARD_EPADDR,
	.Size = KEYBOARD_EPSIZE,
	.Banks = 1,
      },
      .PrevReportINBuffer = PrevKeyboardHIDReportBuffer,
      .PrevReportINBufferSize = sizeof(PrevKeyboardHIDReportBuffer),
    },
  };

void init(void) {
  // Set LED pins as output
  DDRD |= (1 << 5);  // PD5
  DDRB |= (1 << 0);  // PB0

  // Set encoder pins as input
  DDRB &= ~(1 << 2);  // PB2
  DDRB &= ~(1 << 6);  // PB6
  DDRB &= ~(1 << 1); //PB1
  DDRB &= ~(1 << 3); //PB3

  // Enable pull-up resistors on encoder pins
  // Set the state as high (pull-up) for encoder to register
  PORTB |= (1 << 2);  // PB2
  PORTB |= (1 << 6);  // PB6
  PORTB |= (1 << 1);
  PORTB |= (1 << 3);
}

// LED illumination on microcontroller
// 5 v to power the LED
// 0 V to shut off the LED
void illuminate_right_led() {
  PORTD |= (1 << 5);  // Turn off LED on PD5 
  PORTB &= ~(1 << 0); // Turn on LED on PB0
}

void illuminate_left_led() {
  PORTB |= (1 << 0);  // Turn off LED on PB0
  PORTD &= ~(1 << 5); // Turn on LED on PD5
}

void leds_off(){
    PORTB |= (1 << 0); //Turn off LED on PB0
    PORTD |= (1 << 5); // Turn off LED on PD5
}

int main(void)
{
	SetupHardware();

	GlobalInterruptEnable();
	
	// setup matrix
	uint8_t num_rows = 4;
	uint8_t num_cols = 3;
	matrix.num_rows = num_rows;
	matrix.num_cols = num_cols;
	pin_t rows[4] = {d1, d0, d4, c6};
	pin_t cols[3] =	{d7, e6, b4};
	for (int i = 0; i < num_rows; ++i) {
	  matrix.rows[i] = rows[i];
	}
	for (int i = 0; i < num_cols; ++i) {
	  matrix.cols[i] = cols[i];
	}

	// set all rows at outputs
	DDRD |= _BV(1);
	DDRD |= _BV(0);
	DDRD |= _BV(4);
	DDRC |= _BV(6);
	// enable pull-up (set state to digital high) on all rows
	for (uint8_t i = 0; i < matrix.num_rows; ++i) {
	  setPinHigh(matrix.rows[i]);
	}
	// set all columns as inputs
	DDRB &= ~_BV(4); // B4
	DDRE &= ~_BV(6); // E6
	DDRD &= ~_BV(7); // D7
	// enable pull-up on all columns
	for (uint8_t i = 0; i < matrix.num_cols; ++i) {
	  setPinHigh(matrix.cols[i]);
	}	

	// init encoder pins
	DDRF &= ~(1 << 6); // F6 as input
	PORTF |= (1 << 6); // F6 as high
	DDRF &= ~(1 << 7); // F7 as input
	PORTF |= (1 << 7); // F7 as high

	init();
    leds_off();

    // Read initial state of PB2 (pin A rotary encoder 1), store in bit 0
    uint8_t last_state = (PINB & (1<<2)) >> 2; //shift back 2 to put it in the least significant bit
	uint8_t last_state_e2 = (PINB & (1<<1)) >> 1;

	for (;;) {
	    for (uint8_t i = 0; i < matrix.num_rows; ++i) {
	        setPinLow(matrix.rows[i]);
	        matrix.active_col = i;
	        HID_Device_USBTask(&Keyboard_HID_Interface);
	        USB_USBTask();
	        setPinHigh(matrix.rows[i]);
	    }

		// Read current state of PB2, store in bit 0
        uint8_t current_state = (PINB & (1<<2)) >> 2;
		uint8_t current_state_e2 = (PINB & (1<<1)) >> 1;

        //Check if the previous state == current state
        if (current_state != last_state) {
            // Check the state of the next pin (read current state of PB2, store in bit 0)
            uint8_t current_b_state = (PINB & (1 << 6)) >> 6;

            // If the same are the same -> counterclockwise
            if (current_b_state == current_state) { //Compare bit 0
                illuminate_left_led();
            }
            // If the states are not the same -> clockwise
            else {
                illuminate_right_led();
            }
        }

		if (current_state_e2 != last_state_e2) {
			uint8_t current_b_state_e2 = (PINB & (1 << 3)) >> 3;
			// If the same are the same -> counterclockwise
            if (current_b_state_e2 == current_state_e2) { //Compare bit 0
                illuminate_right_led();
            }
            // If the states are not the same -> clockwise
            else {
                illuminate_left_led();
            }
		}
		
        // Push current state to last state to read a new current state next time
        last_state = current_state;
		last_state_e2 = current_state_e2;

		// _delay_ms(10);
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware()
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
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void) { }

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void) { }

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Keyboard_HID_Interface);

	USB_Device_EnableSOFEvents();
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Keyboard_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Keyboard_HID_Interface);
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
	USB_KeyboardReport_Data_t* KeyboardReport = (USB_KeyboardReport_Data_t*)ReportData;
	uint8_t UsedKeyCodes = 0;
	uint8_t offset = matrix.active_col * matrix.num_cols; // Define where in the keyboard we send our keycode
	// encoder 1 push button
	if (!(PINF & (1 << 6))) {
	  KeyboardReport->KeyCode[UsedKeyCodes++] = K_MEDIA_MUTE;
	}
	// encoder 2 push button
	if (!(PINF & (1 << 7))) {
	  KeyboardReport->KeyCode[UsedKeyCodes++] = K_I;
	}

	// check columns one at a time to determine which key is pressed
	for (uint8_t i = 0; i < matrix.num_cols; ++i) {
	  if (isPinLow(matrix.cols[i])) {
		// if (KeyboardReport->KeyCode[UsedKeyCodes] != layout[i+offset])
		// {
	    	KeyboardReport->KeyCode[UsedKeyCodes++] = layout[i+offset];
		// }
	  }
	}
	//  if (UsedKeyCodes) 
	//    KeyboardReport->Modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT; 

	*ReportSize = sizeof(USB_KeyboardReport_Data_t);
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
                                          const uint16_t ReportSize) {}

void setPinLow(pin_t pin) {
  *(pin.port) &= ~_BV(pin.pin_num);
}

void setPinHigh(pin_t pin) {
  *(pin.port) |= _BV(pin.pin_num);
}

bool isPinLow(pin_t pin) {
  return !(*(pin.pin) & _BV(pin.pin_num));
}
