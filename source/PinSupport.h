// Copyright (c) Microsoft Open Technologies, Inc.  All rights reserved.  
// Licensed under the BSD 2-Clause License.  
// See License.txt in the project root for license information.

#ifndef _PIN_FUNCTIONS_H_
#define _PIN_FUNCTIONS_H_

#include <Windows.h>

#include "ArduinoCommon.h"


// Pin function type values.
const UCHAR FUNC_DIO = 0x01;    // Digital I/O
const UCHAR FUNC_PWM = 0x02;    // Pulse Width Modulation (PWM)
const UCHAR FUNC_AIN = 0x04;    // Analog In
const UCHAR FUNC_I2C = 0x08;    // I2C Bus
const UCHAR FUNC_SPI = 0x10;    // SPI Bus
const UCHAR FUNC_SER = 0x20;    // Hardware Serial

// GPIO type values.
const UCHAR GPIO_FABRIC = 1;
const UCHAR GPIO_LEGRES = 2;
const UCHAR GPIO_LEGCOR = 3;
const UCHAR GPIO_EXP1 = 4;
const UCHAR GPIO_EXP2 = 5;

// I/O Expander name values.
const UCHAR EXP0 = 0;           // I/O Expander 0
const UCHAR EXP1 = 1;           // I/O Expander 1
const UCHAR EXP2 = 2;           // I/O Expander 2
const UCHAR PWM = 3;            // PWM used as I/O Expander
const UCHAR NUM_IO_EXP = 4;     // Number of I/O Expanders present
const UCHAR NO_X = 15;          // No I/O Expander

// I/O Expander types.
const UCHAR PCAL9535A = 0;      // I/O Expander chip
const UCHAR PCA9685 = 1;        // PWM chip
const UCHAR NUM_EXP_TYPSES = 2; // Number of I/O Expanders types

// PWM chip bit values.
const UCHAR LED0 = 0;
const UCHAR LED1 = 1;
const UCHAR LED2 = 2;
const UCHAR LED3 = 3;
const UCHAR LED4 = 4;
const UCHAR LED5 = 5;
const UCHAR LED6 = 6;
const UCHAR LED7 = 7;
const UCHAR LED8 = 8;
const UCHAR LED9 = 9;
const UCHAR LED10 = 10;
const UCHAR LED11 = 11;
const UCHAR LED12 = 12;
const UCHAR LED13 = 13;
const UCHAR LED14 = 14;
const UCHAR LED15 = 15;

// I/O Expander bit values.
const UCHAR P0_0 = 0;
const UCHAR P0_1 = 1;
const UCHAR P0_2 = 2;
const UCHAR P0_3 = 3;
const UCHAR P0_4 = 4;
const UCHAR P0_5 = 5;
const UCHAR P0_6 = 6;
const UCHAR P0_7 = 7;
const UCHAR P1_0 = 8;
const UCHAR P1_1 = 9;
const UCHAR P1_2 = 10;
const UCHAR P1_3 = 11;
const UCHAR P1_4 = 12;
const UCHAR P1_5 = 13;
const UCHAR P1_6 = 14;
const UCHAR P1_7 = 15;

// MUX name values.
const UCHAR MUX0 = 0;
const UCHAR MUX1 = 1;
const UCHAR MUX2 = 2;
const UCHAR MUX3 = 3;
const UCHAR MUX4 = 4;
const UCHAR MUX5 = 5;
const UCHAR MUX6 = 6;
const UCHAR MUX7 = 7;
const UCHAR MUX8 = 8;
const UCHAR MUX9 = 9;
const UCHAR MUX10 = 10;
const UCHAR AMUX1 = 11;
const UCHAR AMUX2_1 = 12;
const UCHAR AMUX2_2 = 13;
const UCHAR NUM_MUXES = 14;
const UCHAR NO_MUX = 15;

// This struct stores all the pin-specific attributes needed to configure 
// and use one of the I/O pins.
typedef struct {
    UCHAR gpioType;         // Fabric, Legacy Resume, Legacy Core, Expander
    UCHAR portBit;          // Which bit on the port is attached to this pin
    UCHAR pullupExp : 4;    // Number of I/O expander for pull-up control
    UCHAR pullupBit : 4;    // Bit of I/O expander for pull-up control
    UCHAR triStExp : 4;     // Number of I/O expander for tri-state control
    UCHAR triStBit : 4;     // Bit of I/O expander for tri-state control
    UCHAR muxA : 4;         // Number of first MUX for pin, if any
    UCHAR muxB : 4;         // Number of second MUX for pin, if any
    UCHAR digIoMuxA : 1;    // State of 1st MUX for digital I/O use of pin
    UCHAR digIoMuxB : 1;    // State of 2nd MUX for digital I/O use of pin
    UCHAR pwmMuxA : 1;      // State of 1st MUX for PWM use of pin
    UCHAR pwmMuxB : 1;      // State of 2nd MUX for PWM use of pin
    UCHAR anInMuxA : 1;     // State of 1st MUX for analog input use of pin
    UCHAR anInMuxB : 1;     // State of 2nd MUX for analog input use of pin
    UCHAR i2cMuxA : 1;      // State of 1st MUX for I2C use of pin
    UCHAR i2cMuxB : 1;      // State of 2nd MUX for I2C use of pin
    UCHAR spiMuxA : 1;      // State of 1st MUX for SPI use of pin
    UCHAR spiMuxB : 1;      // State of 2nd MUX for SPI use of pin
    UCHAR serMuxA : 1;      // State of 1st MUX for serial use of pin
    UCHAR serMuxB : 1;      // State of 1nd MUX for serial use of pin
    UCHAR triStIn : 1;      // Tri-state control bit state for input pin
    UCHAR _pad : 3;         // Pad to byte boundary
    UCHAR funcMask;         // Mask of functin types supported on the pin
} PORT_ATTRIBUTES, *PPORT_ATTRIBUTES;

//// The array of pin attributes by pin number.
//_declspec (selectany) PORT_ATTRIBUTES _PinAttributs[NUM_ARDUINO_PINS] =
//{
//    //gpioType           pullupExp   triStExp    muxA             Muxes (A,B) by function:      triStIn Function_mask
//    //             portBit     pullupBit   triStBit      muxB     Dio  Pwm  AnIn I2C  Spi  Ser     _pad
//    { GPIO_FABRIC, 3,    EXP1, P0_1, EXP1, P0_0, NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_SER },            // D0
//    { GPIO_FABRIC, 4,    EXP0, P1_5, EXP0, P1_4, MUX7,   NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 1,0, 1, 0, FUNC_DIO | FUNC_SER },            // D1
//    { GPIO_FABRIC, 5,    EXP1, P0_3, EXP1, P0_2, MUX10,  NO_MUX,  1,0, 0,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_SER },            // D2
//    { GPIO_FABRIC, 6,    EXP0, P0_1, EXP0, P0_0, MUX0,   MUX9,    0,0, 1,0, 0,0, 0,0, 0,0, 0,1, 1, 0, FUNC_DIO | FUNC_PWM | FUNC_SER }, // D3
//    { GPIO_LEGRES, 4,    EXP1, P0_5, EXP1, P0_4, NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO },                       // D4
//    { GPIO_LEGCOR, 0,    EXP0, P0_3, EXP0, P0_2, MUX1,   NO_MUX,  0,0, 1,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_PWM },            // D5
//    { GPIO_LEGCOR, 1,    EXP0, P0_5, EXP0, P0_4, MUX2,   NO_MUX,  0,0, 1,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_PWM },            // D6
//    { GPIO_EXP1,   P0_6, EXP1, P0_7, NO_X, 0,    NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO },                       // D7
//    { GPIO_EXP1,   P1_0, EXP1, P1_1, NO_X, 0,    NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO },                       // D8
//    { GPIO_LEGRES, 2,    EXP0, P0_7, EXP0, P0_6, MUX3,   NO_MUX,  0,0, 1,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_PWM },            // D9
//    { GPIO_FABRIC, 2,    EXP0, P1_3, EXP0, P1_2, MUX6,   NO_MUX,  0,0, 1,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_PWM },            // D10
//    { GPIO_LEGRES, 3,    EXP0, P1_1, EXP0, P1_0, MUX4,   MUX5,    0,0, 1,0, 0,0, 0,0, 0,1, 0,0, 1, 0, FUNC_DIO | FUNC_PWM | FUNC_SPI }, // D11
//    { GPIO_FABRIC, 7,    EXP1, P1_3, EXP1, P1_2, NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_SPI },            // D12
//    { GPIO_LEGRES, 5,    EXP0, P1_7, EXP0, P1_6, MUX8,   NO_MUX,  0,0, 0,0, 0,0, 0,0, 1,0, 0,0, 1, 0, FUNC_DIO | FUNC_SPI },            // D13
//    { GPIO_EXP2,   P0_0, EXP2, P0_1, NO_X, 0,    NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO | FUNC_AIN },            // A0
//    { GPIO_EXP2,   P0_2, EXP2, P0_3, NO_X, 0,    NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO | FUNC_AIN },            // A1
//    { GPIO_EXP2,   P0_4, EXP2, P0_5, NO_X, 0,    NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO | FUNC_AIN },            // A2
//    { GPIO_EXP2,   P0_6, EXP2, P0_7, NO_X, 0,    NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO | FUNC_AIN },            // A3
//    { GPIO_EXP2,   P1_0, EXP2, P1_1, NO_X, 0,    AMUX1,  AMUX2_1, 1,1, 0,0, 1,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO | FUNC_AIN | FUNC_I2C }, // A4
//    { GPIO_EXP2,   P1_2, EXP2, P1_3, NO_X, 0,    AMUX1,  AMUX2_2, 1,1, 0,0, 1,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO | FUNC_AIN | FUNC_I2C }  // A5
//};

// This struct stores the mux-specific attributes needed to use the mux.
typedef struct {
    UCHAR selectExp : 4;    // I/O Expander that drives the select signal
    UCHAR selectBit : 4;    // Bit of I/O Expander that drives select
} MUX_ATTRIBUTES, *PMUX_ATTRIBUTES;

//// The array of MUX Attributes by MUX number.
//__declspec (selectany) MUX_ATTRIBUTES _MuxAttributes[NUM_MUXES] = 
//{
//    { PWM, LED1 },      // MUX0
//    { PWM, LED2 },      // MUX1
//    { PWM, LED5 },      // MUX2
//    { PWM, LED7 },      // MUX3
//    { PWM, LED9 },      // MUX4
//    { EXP1, P1_4 },     // MUX5
//    { PWM, LED11 },     // MUX6
//    { EXP1, P1_5 },     // MUX7
//    { EXP1, P1_6 },     // MUX8
//    { PWM, LED12 },     // MUX9
//    { PWM, LED13 },     // MUX10
//    { EXP2, P1_4 },     // AMUX1
//    { PWM, LED14 },     // AMUX2_1
//    { PWM, LED15 }      // AMUX2_2
//};

// This struct stores the I/O Expander-specific attributes needed to use it.
typedef struct {
    UCHAR Exp_Type;         // I/O Expander chip type
    UCHAR I2c_Address;      // I2C address of the I/O expander
} EXP_ATTRIBUTES, *PEXP_ATTRIBUTES;

// The array of I/O Expander Attributes by I/O Expander number.
__declspec (selectany) EXP_ATTRIBUTES _ExpAttributes[NUM_IO_EXP] =
{
    { PCAL9535A, 0x25 },    // EXP0
    { PCAL9535A, 0x26 },    // EXP1
    { PCAL9535A, 0x27 },    // EXP2
    { PCA9685,   0x47 }     // PWM
};

#pragma warning(push)
#pragma warning(disable : 4201) // Ignore nameless struct/union warnings

// Port A Data Register.
typedef union {
    struct {
        ULONG GPIO_SWPORTA_DR : 8;  // Port Data
        ULONG _reserved : 24;
    };
    ULONG ALL_BITS;
} _GPIO_SWPORTA_DR;

// Port A Data Direction Register.
typedef union {
    struct {
        ULONG GPIO_SWPORTA_DDR : 8;     // Port Data Direction
        ULONG _reserved : 24;
    };
    ULONG ALL_BITS;
} _GPIO_SWPORTA_DDR;

// Interrupt Enable Register for Port A.
typedef union {
    struct {
        ULONG GPIO_INTEN : 8;           // Interrupt Enable
        ULONG _reserved : 24;
    };
    ULONG ALL_BITS;
} _GPIO_INTEN;

// Interrupt Mask Register for Port A.
typedef union {
    struct {
        ULONG GPIO_INTMASK : 8;         // Interrupt Mask
        ULONG _reserved : 24;
    };
    ULONG ALL_BITS;
} _GPIO_INTMASK;

// Interrupt Type Register for Port A.
typedef union {
    struct {
        ULONG GPI_INTTYPE_LEVEL : 8;    // Interrupt Type
        ULONG _reserved : 24;
    };
    ULONG ALL_BITS;
} _GPIO_INTTYPE_LEVEL;

// Interrupt Polarity Register for Port A.
typedef union {
    struct {
        ULONG GPIO_INT_POLARITY : 8;    // Interrupt Polarity
        ULONG _reserved : 24;
    };
    ULONG ALL_BITS;
} _GPIO_INT_POLARITY;

// Interrupt Status Register for Port A.
typedef union {
    struct {
        ULONG GPIO_INTSTATUS : 8;       // Interrupt Status
        ULONG _reserved : 24;
    };
    ULONG ALL_BITS;
} _GPIO_INTSTATUS;

// Raw Interrupt Status Register for Port A.
typedef union {
    struct {
        ULONG GPIO_RAW_INSTATUS : 8;    // Raw Interrupt Status
        ULONG _reserved : 24;
    };
    ULONG ALL_BITS;
} _GPIO_RAW_INTSTATUS;

// Debounce Enable Register for Port A.
typedef union {
    struct {
        ULONG GPIO_DEBOUNCE : 8;        // Debounce Enable
        ULONG _reserved : 24;
    };
    ULONG ALL_BITS;
} _GPIO_DEBOUNCE;

// Clear Interrupt Register for Port A.
typedef union {
    struct {
        ULONG GPIO_PORTA_EOI : 8;       // Clear Interrupt
        ULONG _reserved : 24;
    };
    ULONG ALL_BITS;
} _GPIO_PORTA_EOI;

// Port A External Port Register.
typedef union {
    struct {
        ULONG GPIO_EXT_PORTA : 8;       // External Port
        ULONG _reserved : 24;
    };
    ULONG ALL_BITS;
} _GPIO_EXT_PORTA;

// Register controls Synchronization of Level-Sensitive interrupts.
typedef union {
    struct {
        ULONG GPIO_LS_SYNC : 1;         // Synchronization Level
        ULONG _reserved : 31;
    };
    ULONG ALL_BITS;
} _GPIO_LS_SYNC;

#pragma warning( pop )

// Layout of the Quark Fabric GPIO Controller registers in memory.
typedef struct _FABRIC_GPIO {
    volatile _GPIO_SWPORTA_DR       GPIO_SWPORTA_DR;    // 0x00 - Port A Data
    volatile _GPIO_SWPORTA_DDR      GPIO_SWPORTA_DDR;   // 0x04 - Port A Data Direction
    ULONG                           _reserved1[0x28];   // 0x08 - 0x2F
    volatile _GPIO_INTEN            GPIO_INTEN;         // 0x30 - Interrupt Enable
    volatile _GPIO_INTMASK          GPIO_INTMASK;       // 0x34 - Interrupt Mask
    volatile _GPIO_INTTYPE_LEVEL    GPIO_INTTYPE_LEVEL; // 0x38 - Interrupt Type
    volatile _GPIO_INT_POLARITY     GPIO_INT_POLARITY;  // 0x3C - Interrupt Polarity
    volatile _GPIO_INTSTATUS        GPIO_INTSTATUS;     // 0x40 - Interrupt Status
    volatile _GPIO_RAW_INTSTATUS    GPIO_RAW_INTSTATUS; // 0x44 - Raw Interrupt Status
    volatile _GPIO_DEBOUNCE         GPIO_DEBOUNCE;      // 0x48 - Debounce enable
    volatile _GPIO_PORTA_EOI        GPIO_PORTA_EOI;     // 0x4C - Clear Interrupt
    volatile _GPIO_EXT_PORTA        GPIO_EXT_PORTA;     // 0x50 - Port A External Port
    ULONG                           _reserved[0x0C];    // 0x54 - 0x5F
    volatile _GPIO_LS_SYNC          GPIO_LS_SYNC;       // 0x60 - Synchronization Level
} FABRIC_GPIO, *PFABRIC_GPIO;

// Pin name to number mapping.
const UCHAR D0 = 0;
const UCHAR D1 = 1;
const UCHAR D2 = 2;
const UCHAR D3 = 3;
const UCHAR D4 = 4;
const UCHAR D5 = 5;
const UCHAR D6 = 6;
const UCHAR D7 = 7;
const UCHAR D8 = 8;
const UCHAR D9 = 9;
const UCHAR D10 = 10;
const UCHAR D11 = 11;
const UCHAR D12 = 12;
const UCHAR D13 = 13;
const UCHAR A0 = 14;
const UCHAR A1 = 15;
const UCHAR A2 = 16;
const UCHAR A3 = 17;
const UCHAR A4 = 18;
const UCHAR A5 = 19;

// SPI signal to pin mapping.
const UCHAR PIN_MOSI = D11;
const UCHAR PIN_MISO = D12;
const UCHAR PIN_SCK = D13;

// I2C signal to pin mapping.
const UCHAR PIN_I2C_DAT = A4;
const UCHAR PIN_I2C_CLK = A5;

BOOL _setPinFunction(UCHAR pin, UCHAR function);
BOOL _setPinDigitalIo(UCHAR pin);
BOOL _setPinPwm(UCHAR pin);
BOOL _setPinAnalogInput(UCHAR pin);
BOOL _setPinI2c(UCHAR pin);
BOOL _setPinSpi(UCHAR pin);
BOOL _setPinHwSerial(UCHAR pin);

BOOL _setPinMode(UCHAR pin, UCHAR mode, bool pullUp);

BOOL _setMux(UCHAR mux, UCHAR selection);

//BOOL _setPinState(UCHAR pin, UCHAR state);
inline BOOL _setPinState(UCHAR pin, UCHAR state)
{
    BOOL status = TRUE;
    DWORD error = ERROR_SUCCESS;


    if (!status)
    {
        SetLastError(error);
    }
    return status;
}


// Function to make sure a pin number is in range.
inline BOOL _verifyPinNumberValid(UCHAR pin)
{
    return (pin < NUM_ARDUINO_PINS);
}

#endif // _PIN_FUNCTIONS_H_