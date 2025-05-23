/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

// The definitions here needs a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610


#include <WVariant.h>

// General definitions
// -------------------

// Frequency of the board main oscillator
#define VARIANT_MAINOSC (32768ul)

// Master clock frequency
#define VARIANT_MCK     (48000000ul)

// Pins
// ----

// Number of pins defined in PinDescription array
#ifdef __cplusplus
extern "C" unsigned int PINCOUNT_fn();
#endif
#define PINS_COUNT           (PINCOUNT_fn())
#define NUM_DIGITAL_PINS     (31u)
#define NUM_ANALOG_INPUTS    (8u)
#define NUM_ANALOG_OUTPUTS   (1u)

// Low-level pin register query macros
// -----------------------------------
#define digitalPinToPort(P)      (&(PORT->Group[g_APinDescription[P].ulPort]))
#define digitalPinToBitMask(P)   (1 << g_APinDescription[P].ulPin)
//#define analogInPinToBit(P)    ()
#define portOutputRegister(port) (&(port->OUT.reg))
#define portInputRegister(port)  (&(port->IN.reg))
#define portModeRegister(port)   (&(port->DIR.reg))
#define digitalPinHasPWM(P)      (g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER)

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)


// LEDs
// ----
#define PIN_LED_13  (29u)
#define PIN_LED     PIN_LED_13
#define LED_BUILTIN PIN_LED

// LoRa
#define LORA_GPIO0  (30u)
#define LORA_RESET  (15u)
#define LORA_CS     (22u)

/*
 * Analog pins
 */
#define PIN_A0               (0u)
#define PIN_A1               (1u)
#define PIN_A2               (2u)
#define PIN_A3               (3u)
#define PIN_A4               (4u)
#define PIN_A5               (7u)
#define PIN_A6               (31ul)
#define PIN_A7               (32ul)

#define PIN_DAC0             (0u)

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6;
static const uint8_t A7  = PIN_A7;
static const uint8_t DAC0 = PIN_DAC0;

#define ADC_RESOLUTION		12

#define ISS3_RAIN_INT (8u)
#define ISS4_WINDSPEED_INT (9u)

/*
 * SPI Interfaces
 */
// SPI Ethernet
#define SPI_INTERFACES_COUNT 1

// SPI1 LoRa
#define PIN_SPI_MISO        (17u)
#define PIN_SPI_SCK         (18u)
#define PIN_SPI_SS          (19u)
#define PIN_SPI_MOSI        (20u)
#define PERIPH_SPI          sercom1
#define PAD_SPI_TX          SPI_PAD_0_SCK_1
#define PAD_SPI_RX          SERCOM_RX_PAD_3
static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

//#define SPIWIFI_SS       PIN_SPI1_SS
//#define SPIWIFI_ACK      NINA_ACK
//#define SPIWIFI_RESET    NINA_RESETN

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 2

  // SMT i2c interface
#define PIN_WIRE1_SDA         (7u)
#define PIN_WIRE1_SCL         (8u)
#define PERIPH_WIRE1          sercom2
#define WIRE1_IT_HANDLER      SERCOM2_Handler
static const uint8_t SDA1 = PIN_WIRE1_SDA;
static const uint8_t SCL1 = PIN_WIRE1_SCL;

  // "external" public i2c interface
#define PIN_WIRE_SDA         (13u)
#define PIN_WIRE_SCL         (14u)
#define PERIPH_WIRE          sercom4
#define WIRE_IT_HANDLER      SERCOM4_Handler
static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

// USB
// ---
#define PIN_USB_DM          (25ul)
#define PIN_USB_DP          (26ul)

// I2S Interfaces
// --------------
#define I2S_INTERFACES_COUNT 1

#define I2S_DEVICE          0
#define I2S_CLOCK_GENERATOR 3

//#define PIN_I2S_SD (4u) 
//#define PIN_I2S_SCK (PIN_A3) 
//#define PIN_I2S_FS (PIN_A2) 

// Serial ports
// ------------
#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"

// Instances of SERCOM
extern SERCOM sercom0;
extern SERCOM sercom1;  // spi1 LoRa 
extern SERCOM sercom2;  // wire SMT
extern SERCOM sercom3;  // serial2 RS232
extern SERCOM sercom4;  // wire
extern SERCOM sercom5;  // serial1 Modbus

// Serial1 - Modbus
extern Uart Serial1;
#define PIN_SERIAL1_TX       (27ul)
#define PIN_SERIAL1_RX       (28ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_2)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_3)

// Serial2 - RS232
extern Uart Serial2;
#define PIN_SERIAL2_TX       (23ul)
#define PIN_SERIAL2_RX       (21ul)
#define PAD_SERIAL2_TX       (UART_TX_PAD_0)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_2)

// extern Uart SerialHCI;
// #define PIN_SERIALHCI_RX (23ul)
// #define PIN_SERIALHCI_TX (22ul)
// #define PAD_SERIALHCI_TX (UART_TX_RTS_CTS_PAD_0_2_3)
// #define PAD_SERIALHCI_RX (SERCOM_RX_PAD_1)
// #define PIN_SERIALHCI_RTS (24u)
// #define PIN_SERIALHCI_CTS (25u)

#endif // __cplusplus



// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         SerialUSB
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial1

// Alias Serial1 to SerialNina (useful in libraries)
#define SPILORA                     SPI

// Alias Serial to SerialUSB
#define Serial                      SerialUSB
