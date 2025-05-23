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

// Sync Outputs
#define SYNC1 (31u)
#define SYNC2 (32u)

// Ethernet
#define LORA_GPIO0  (3u)
#define LORA_RESET  (4u)
#define ETH_RESETN  (22u)

// RS485
#define DE_RS (5u)
#define RE_RS (6u)

// Isolated Inputs
#define ISO_1 (11u)
#define ISO_2 (12u)
#define ISO_3 (13u)
#define ISO_4 (14u)
#define ISO_5 (15u)
#define ISO_6 (16u)
#define ISO_7 (1u)
#define ISO_8 (2u)

//#define PIN_LED_RXL          (25u)
//#define PIN_LED_TXL          (26u)
//#define PIN_LED2             PIN_LED_RXL
//#define PIN_LED3             PIN_LED_TXL

/*
 * Analog pins
 */
#define PIN_A0               (1ul)
#define PIN_A1               (2ul)
#define PIN_A2               (3ul)
#define PIN_A3               (4ul)
#define PIN_A4               (5ul)
#define PIN_A5               (6ul)
#define PIN_A6               (9ul)
#define PIN_A7               (10ul)

#define PIN_DAC0             (0)

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

/*
 * SPI Interfaces
 */
// SPI Ethernet
#define SPI_INTERFACES_COUNT 2

#define PIN_SPI_MISO        (17u)
#define PIN_SPI_SCK         (18u)
#define PIN_SPI_SS          (19u)
#define PIN_SPI_MOSI        (20u)
#define PERIPH_SPI          sercom1
#define PAD_SPI_TX          SPI_PAD_3_SCK_1
#define PAD_SPI_RX          SERCOM_RX_PAD_0
static const uint8_t SS   = PIN_SPI_SS;   // SPI Slave SS not used. Set here only for reference.
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// SPI1 LoRa
#define PIN_SPI1_MISO        (21u)
#define PIN_SPI1_SS          (22u)
#define PIN_SPI1_MOSI        (23u)
#define PIN_SPI1_SCK         (24u)
#define PERIPH_SPI1          sercom3
#define PAD_SPI1_TX          SPI_PAD_0_SCK_1
#define PAD_SPI1_RX          SERCOM_RX_PAD_2
static const uint8_t SS1   = PIN_SPI1_SS;
static const uint8_t MOSI1 = PIN_SPI1_MOSI;
static const uint8_t MISO1 = PIN_SPI1_MISO;
static const uint8_t SCK1  = PIN_SPI1_SCK;

// Needed for SD library
#define SDCARD_SPI      SPI1
#define SDCARD_MISO_PIN PIN_SPI1_MISO
#define SDCARD_MOSI_PIN PIN_SPI1_MOSI
#define SDCARD_SCK_PIN  PIN_SPI1_SCK
#define SDCARD_SS_PIN   PIN_SPI1_SS
//#define SPIWIFI_SS       PIN_SPI1_SS
//#define SPIWIFI_ACK      NINA_ACK
//#define SPIWIFI_RESET    NINA_RESETN

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

  // "external" public i2c interface
#define PIN_WIRE_SDA         (7u)
#define PIN_WIRE_SCL         (8u)
#define PERIPH_WIRE          sercom2
#define WIRE_IT_HANDLER      SERCOM2_Handler
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
extern SERCOM sercom2;  // wire
extern SERCOM sercom3;  // spi eth
extern SERCOM sercom4;  // serial2 RS232
extern SERCOM sercom5;  // serial1 RS485

// Serial1
extern Uart Serial1;
#define PIN_SERIAL1_TX       (27ul)
#define PIN_SERIAL1_RX       (28ul)
#define PAD_SERIAL1_TX       (UART_TX_PAD_2)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_3)

// Serial2
extern Uart Serial2;
#define PIN_SERIAL2_RX       (1ul)
#define PIN_SERIAL2_TX       (2ul)
#define PAD_SERIAL2_TX       (UART_TX_PAD_0)
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_1)

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

// Alias Serial to SerialUSB
#define Serial                      SerialUSB
