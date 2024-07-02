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

#pragma once

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>

#include "Descriptors.h"

#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Platform/Platform.h>

#include "keys.h"
#include "pins.h"

//row (top to bottom)
pin_t d1 = {&PORTD, &PIND, 1};
pin_t d0 = {&PORTD, &PIND, 0};
pin_t d4 = {&PORTD, &PIND, 4};
pin_t c6 = {&PORTC, &PINC, 6};
//col (from right to left)
pin_t b4 = {&PORTB, &PINB, 4};
pin_t e6 = {&PORTE, &PINE, 6};
pin_t d7 = {&PORTD, &PIND, 7};

typedef struct {
  uint8_t num_rows;
  uint8_t num_cols;
  pin_t rows[16];
  pin_t cols[16];
  uint8_t active_col;
} matrix_t;

matrix_t matrix;

uint8_t layout[256] = {
  K_HOME, K_END, K_PAGE_UP,
  K_END, K_INSERT, K_PAGE_DOWN,
  K_I, K_UP_ARROW, K_L,
  K_LEFT_ARROW, K_DOWN_ARROW, K_RIGHT_ARROW
};

void SetupHardware(void);

void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);
void EVENT_USB_Device_StartOfFrame(void);

bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
					 uint8_t* const ReportID,
					 const uint8_t ReportType,
					 void* ReportData,
					 uint16_t* const ReportSize);
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
					  const uint8_t ReportID,
					  const uint8_t ReportType,
					  const void* ReportData,
					  const uint16_t ReportSize);

void setPinLow(pin_t pin);
void setPinHigh(pin_t pin);
bool isPinLow(pin_t pin);
