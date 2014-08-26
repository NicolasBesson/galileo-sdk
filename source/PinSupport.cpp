// Copyright (c) Microsoft Open Technologies, Inc.  All rights reserved.  
// Licensed under the BSD 2-Clause License.  
// See License.txt in the project root for license information.

#include <Windows.h>
#include "PinSupport.h"
#include "PCAL9535ASuppport.h"
#include "PCA9685Support.h"

// The handle to the Fabric GPIO controller.
HANDLE g_hFabricGpio = INVALID_HANDLE_VALUE;

// The handle to the Legacy GPIO controller.
HANDLE g_hLegacyGpio = INVALID_HANDLE_VALUE;

// Pointer to the object used to address the Fabric GPIO registers.
PFABRIC_GPIO g_fabricGpio = nullptr;

// The array of pin attributes by pin number.
PORT_ATTRIBUTES _PinAttributes[NUM_ARDUINO_PINS] =
{
    //gpioType           pullupExp   triStExp    muxA             Muxes (A,B) by function:      triStIn   Function_mask
    //             portBit     pullupBit   triStBit      muxB     Dio  Pwm  AnIn I2C  Spi  Ser     _pad
    { GPIO_FABRIC, 3,    EXP1, P0_1, EXP1, P0_0, NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_SER },            // D0
    { GPIO_FABRIC, 4,    EXP0, P1_5, EXP0, P1_4, MUX7,   NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 1,0, 1, 0, FUNC_DIO | FUNC_SER },            // D1
    { GPIO_FABRIC, 5,    EXP1, P0_3, EXP1, P0_2, MUX10,  NO_MUX,  1,0, 0,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_SER },            // D2
    { GPIO_FABRIC, 6,    EXP0, P0_1, EXP0, P0_0, MUX0,   MUX9,    0,0, 1,0, 0,0, 0,0, 0,0, 0,1, 1, 0, FUNC_DIO | FUNC_PWM | FUNC_SER }, // D3
    { GPIO_LEGRES, 4,    EXP1, P0_5, EXP1, P0_4, NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO },                       // D4
    { GPIO_LEGCOR, 0,    EXP0, P0_3, EXP0, P0_2, MUX1,   NO_MUX,  0,0, 1,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_PWM },            // D5
    { GPIO_LEGCOR, 1,    EXP0, P0_5, EXP0, P0_4, MUX2,   NO_MUX,  0,0, 1,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_PWM },            // D6
    { GPIO_EXP1,   P0_6, EXP1, P0_7, NO_X, 0,    NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO },                       // D7
    { GPIO_EXP1,   P1_0, EXP1, P1_1, NO_X, 0,    NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO },                       // D8
    { GPIO_LEGRES, 2,    EXP0, P0_7, EXP0, P0_6, MUX3,   NO_MUX,  0,0, 1,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_PWM },            // D9
    { GPIO_FABRIC, 2,    EXP0, P1_3, EXP0, P1_2, MUX6,   NO_MUX,  0,0, 1,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_PWM },            // D10
    { GPIO_LEGRES, 3,    EXP0, P1_1, EXP0, P1_0, MUX4,   MUX5,    0,0, 1,0, 0,0, 0,0, 0,1, 0,0, 1, 0, FUNC_DIO | FUNC_PWM | FUNC_SPI }, // D11
    { GPIO_FABRIC, 7,    EXP1, P1_3, EXP1, P1_2, NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 1, 0, FUNC_DIO | FUNC_SPI },            // D12
    { GPIO_LEGRES, 5,    EXP0, P1_7, EXP0, P1_6, MUX8,   NO_MUX,  0,0, 0,0, 0,0, 0,0, 1,0, 0,0, 1, 0, FUNC_DIO | FUNC_SPI },            // D13
    { GPIO_EXP2,   P0_0, EXP2, P0_1, NO_X, 0,    NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO | FUNC_AIN },            // A0
    { GPIO_EXP2,   P0_2, EXP2, P0_3, NO_X, 0,    NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO | FUNC_AIN },            // A1
    { GPIO_EXP2,   P0_4, EXP2, P0_5, NO_X, 0,    NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO | FUNC_AIN },            // A2
    { GPIO_EXP2,   P0_6, EXP2, P0_7, NO_X, 0,    NO_MUX, NO_MUX,  0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO | FUNC_AIN },            // A3
    { GPIO_EXP2,   P1_0, EXP2, P1_1, NO_X, 0,    AMUX1,  AMUX2_1, 1,1, 0,0, 1,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO | FUNC_AIN | FUNC_I2C }, // A4
    { GPIO_EXP2,   P1_2, EXP2, P1_3, NO_X, 0,    AMUX1,  AMUX2_2, 1,1, 0,0, 1,0, 0,0, 0,0, 0,0, 0, 0, FUNC_DIO | FUNC_AIN | FUNC_I2C }  // A5
};

// The array of MUX Attributes by MUX number.
MUX_ATTRIBUTES _MuxAttributes[NUM_MUXES] =
{
    { PWM,  LED1 },     // MUX0
    { PWM,  LED2 },     // MUX1
    { PWM,  LED5 },     // MUX2
    { PWM,  LED7 },     // MUX3
    { PWM,  LED9 },     // MUX4
    { EXP1, P1_4 },     // MUX5
    { PWM,  LED11 },    // MUX6
    { EXP1, P1_5 },     // MUX7
    { EXP1, P1_6 },     // MUX8
    { PWM,  LED12 },    // MUX9
    { PWM,  LED13 },    // MUX10
    { EXP2, P1_4 },     // AMUX1
    { PWM,  LED14 },    // AMUX2_1
    { PWM,  LED15 }     // AMUX2_2
};

//// The array of I/O Expander Attributes by I/O Expander number.
//EXP_ATTRIBUTES _ExpAttributes[NUM_IO_EXP] =
//{
//    { PCAL9535A, 0x25 },    // EXP0
//    { PCAL9535A, 0x26 },    // EXP1
//    { PCAL9535A, 0x27 },    // EXP2
//    { PCA9685,   0x47 }     // PWM
//};

//
// Routine to set the current function of a multi-function pin.
// INPUT:
//      pin - The number of the pin in question.
//      function - The function to be used on the pin.
// RETURN:
//      TRUE - Success.
//      FALSE - Failure.  GetLastError() provides error code.
//
BOOL _setPinFunction(UCHAR pin, UCHAR function)
{
    BOOL status = TRUE;
    DWORD error = ERROR_SUCCESS;

    // Verify the pin number is in range.
    status = _verifyPinNumberValid(pin);
    if (!status) { error = ERROR_INVALID_PARAMETER; }

    // Verify the requsted function is supported on this pin.
    if (status && ((_PinAttributes[pin].funcMask & function) == 0))
    {
        status = FALSE;
        error = ERROR_NOT_SUPPORTED;
    }

    if (status)
    {
        if (function == FUNC_DIO)
        {
            status = _setPinDigitalIo(pin);
            if (!status) { error = GetLastError(); }
        }
        else if (function == FUNC_PWM)
        {
            status = _setPinPwm(pin);
            if (!status) { error = GetLastError(); }
        }
        else if (function == FUNC_AIN)
        {
            status = _setPinAnalogInput(pin);
            if (!status) { error = GetLastError(); }
        }
        else if (function == FUNC_I2C)
        {
            status = _setPinI2c(pin);
            if (!status) { error = GetLastError(); }
        }
        else if (function == FUNC_SPI)
        {
            status = _setPinSpi(pin);
            if (!status) { error = GetLastError(); }
        }
        else if (function == FUNC_SER)
        {
            status = _setPinHwSerial(pin);
            if (!status) { error = GetLastError(); }
        }
        else
        {
            status = FALSE;
            ERROR_INVALID_PARAMETER;
        }
    }

    if (!status)
    {
        SetLastError(error);
    }
    return status;
}

//
// Routine to set the current function of a pin to Digital I/O.
// This routine assumes the caller has verified pin number range
// and that Digital I/O is supported on the specified pin.
// INPUT:
//      pin - The number of the pin in question.
// RETURN:
//      TRUE - Success.
//      FALSE - Failure.  GetLastError() provides error code.
//
BOOL _setPinDigitalIo(UCHAR pin)
{
    BOOL status = TRUE;
    DWORD error = ERROR_SUCCESS;


    // If the pin is tied to at lease one MUX:
    if (_PinAttributes[pin].muxA != NO_MUX)
    {
        // Set the MUX to the desired state for Digital I/O.
        status = _setMux(_PinAttributes[pin].muxA, _PinAttributes[pin].digIoMuxA);
        if (!status) { error = GetLastError(); }
    }

    // If the pin is tied to a second MUX:
    if (status && (_PinAttributes[pin].muxB != NO_MUX))
    {
        // Set the MUX to the desired state for Digital I/O.
        status = _setMux(_PinAttributes[pin].muxB, _PinAttributes[pin].digIoMuxB);
        if (!status) { error = GetLastError(); }
    }

    if (!status)
    {
        SetLastError(error);
    }
    return status;
}

//
// Routine to set the current function of a pin to PWM.
// This routine assumes the caller has verified pin number range
// and that PWM is supported on the specified pin.
// INPUT:
//      pin - The number of the pin in question.
// RETURN:
//      TRUE - Success.
//      FALSE - Failure.  GetLastError() provides error code.
//
BOOL _setPinPwm(UCHAR pin)
{
    BOOL status = TRUE;
    DWORD error = ERROR_SUCCESS;


    // If the pin is tied to at lease one MUX:
    if (_PinAttributes[pin].muxA != NO_MUX)
    {
        // Set the MUX to the desired state for PWM.
        status = _setMux(_PinAttributes[pin].muxA, _PinAttributes[pin].pwmMuxA);
        if (!status) { error = GetLastError(); }
    }

    // If the pin is tied to a second MUX:
    if (status && (_PinAttributes[pin].muxB != NO_MUX))
    {
        // Set the MUX to the desired state for PWM.
        status = _setMux(_PinAttributes[pin].muxB, _PinAttributes[pin].pwmMuxB);
        if (!status) { error = GetLastError(); }
    }

    if (!status)
    {
        SetLastError(error);
    }
    return status;
}

//
// Routine to set the current function of a pin to Analog Input.
// This routine assumes the caller has verified pin number range
// and that Analog Input is supported on the specified pin.
// INPUT:
//      pin - The number of the pin in question.
// RETURN:
//      TRUE - Success.
//      FALSE - Failure.  GetLastError() provides error code.
//
BOOL _setPinAnalogInput(UCHAR pin)
{
    BOOL status = TRUE;
    DWORD error = ERROR_SUCCESS;


    // If the pin is tied to at lease one MUX:
    if (_PinAttributes[pin].muxA != NO_MUX)
    {
        // Set the MUX to the desired state for Analog Input.
        status = _setMux(_PinAttributes[pin].muxA, _PinAttributes[pin].anInMuxA);
        if (!status) { error = GetLastError(); }
    }

    // If the pin is tied to a second MUX:
    if (status && (_PinAttributes[pin].muxB != NO_MUX))
    {
        // Set the MUX to the desired state for Analog Input.
        status = _setMux(_PinAttributes[pin].muxB, _PinAttributes[pin].anInMuxB);
        if (!status) { error = GetLastError(); }
    }

    if (!status)
    {
        SetLastError(error);
    }
    return status;
}

//
// Routine to set the current function of a pin to I2C.
// This routine assumes the caller has verified pin number range
// and that I2C bus use is supported on the specified pin.
// INPUT:
//      pin - The number of the pin in question.
// RETURN:
//      TRUE - Success.
//      FALSE - Failure.  GetLastError() provides error code.
//
BOOL _setPinI2c(UCHAR pin)
{
    BOOL status = TRUE;
    DWORD error = ERROR_SUCCESS;


    // If the pin is tied to at lease one MUX:
    if (_PinAttributes[pin].muxA != NO_MUX)
    {
        // Set the MUX to the desired state for I2C bus use.
        status = _setMux(_PinAttributes[pin].muxA, _PinAttributes[pin].i2cMuxA);
        if (!status) { error = GetLastError(); }
    }

    // If the pin is tied to a second MUX:
    if (status && (_PinAttributes[pin].muxB != NO_MUX))
    {
        // Set the MUX to the desired state for I2C bus use.
        status = _setMux(_PinAttributes[pin].muxB, _PinAttributes[pin].i2cMuxB);
        if (!status) { error = GetLastError(); }
    }

    if (!status)
    {
        SetLastError(error);
    }
    return status;
}

//
// Routine to set the current function of a pin to SPI.
// This routine assumes the caller has verified pin number range
// and that SPI bus use is supported on the specified pin.
// INPUT:
//      pin - The number of the pin in question.
// RETURN:
//      TRUE - Success.
//      FALSE - Failure.  GetLastError() provides error code.
//
BOOL _setPinSpi(UCHAR pin)
{
    BOOL status = TRUE;
    DWORD error = ERROR_SUCCESS;


    // If the pin is tied to at lease one MUX:
    if (_PinAttributes[pin].muxA != NO_MUX)
    {
        // Set the MUX to the desired state for SPI bus use.
        status = _setMux(_PinAttributes[pin].muxA, _PinAttributes[pin].spiMuxA);
        if (!status) { error = GetLastError(); }
    }

    // If the pin is tied to a second MUX:
    if (status && (_PinAttributes[pin].muxB != NO_MUX))
    {
        // Set the MUX to the desired state for SPI bus use.
        status = _setMux(_PinAttributes[pin].muxB, _PinAttributes[pin].spiMuxB);
        if (!status) { error = GetLastError(); }
    }

    if (!status)
    {
        SetLastError(error);
    }
    return status;
}

//
// Routine to set the current function of a pin to Hardware Serial.
// This routine assumes the caller has verified pin number range
// and that Hardware Serial is supported on the specified pin.
// INPUT:
//      pin - The number of the pin in question.
// RETURN:
//      TRUE - Success.
//      FALSE - Failure.  GetLastError() provides error code.
//
BOOL _setPinHwSerial(UCHAR pin)
{
    BOOL status = TRUE;
    DWORD error = ERROR_SUCCESS;


    // If the pin is tied to at lease one MUX:
    if (_PinAttributes[pin].muxA != NO_MUX)
    {
        // Set the MUX to the desired state for Hardware Serial use.
        status = _setMux(_PinAttributes[pin].muxA, _PinAttributes[pin].serMuxA);
        if (!status) { error = GetLastError(); }
    }

    // If the pin is tied to a second MUX:
    if (status && (_PinAttributes[pin].muxB != NO_MUX))
    {
        // Set the MUX to the desired state for Hardware Serial use.
        status = _setMux(_PinAttributes[pin].muxB, _PinAttributes[pin].serMuxB);
        if (!status) { error = GetLastError(); }
    }

    if (!status)
    {
        SetLastError(error);
    }
    return status;
}

//
// Routine to set the mode and drive type of a pin (Input, Output, etc.)
// INPUT:
//      pin - The number of the pin in question.
//      mode - The desired mode: INPUT or OUTPUT
//      pullup: true - enable pin pullup resistor, false - disable pullup
// RETURN:
//      TRUE - Success.
//      FALSE - Failure.  GetLastError() provides error code.
//
BOOL _setPinMode(UCHAR pin, UCHAR mode, bool pullup)
{
    BOOL status = TRUE;
    DWORD error = ERROR_SUCCESS;


    if (!status)
    {
        SetLastError(error);
    }
    return status;
}

//
// Routine to set a MUX to select a deired signal.
// INPUT:
//      mux - The number of the MUX in question.
//      selection - The desired state of the MUX "select" input.
// RETURN:
//      TRUE - Success.
//      FALSE - Failure.  GetLastError() provides error code.
//
BOOL _setMux(UCHAR mux, UCHAR selection)
{
    BOOL status = TRUE;
    DWORD error = ERROR_SUCCESS;
    UCHAR expNo = 0;                // I/O Expander number
    UCHAR i2cAdr = 0;               // I2C address of I/O Expander
    UCHAR bitNo = 0;                // Bit number on I/O Expander


    // If the MUX number is outside the valid range, fail.
    if (mux >= NUM_MUXES)
    {
        status = FALSE;
        error = ERROR_INVALID_PARAMETER;
    }

    if (status)
    {
        // Determine which I/O Expander drives the MUX select input.
        expNo = _MuxAttributes[mux].selectExp;
        if (expNo >= NUM_IO_EXP)
        {
            status = FALSE;
            error = ERROR_INVALID_ENVIRONMENT;
        }
    }

    if (status)
    {
        // For clarity, get information about bit that drives the MUX select line.
        i2cAdr = _ExpAttributes[expNo].I2c_Address;
        bitNo = _MuxAttributes[mux].selectBit;

        // Determine what type of I/O Expander drives the MUX select input.
        if (_ExpAttributes[expNo].Exp_Type == PCAL9535A)
        {
            // Set the bit of the I/O Expander chip to the desired state.
            status = PCAL9535ADevice::SetBitState(i2cAdr, bitNo, selection);
            if (!status) { error = GetLastError(); }
        }
        else if (_ExpAttributes[expNo].Exp_Type == PCA9685)
        {
            // Set the bit of the PWM chip to the desired state.
            status = PCA9685Device::SetBitState(i2cAdr, bitNo, selection);
            if (!status) { error = GetLastError(); }
        }
        else
        {
            status = FALSE;
            error = ERROR_INVALID_ENVIRONMENT;
        }
    }

    if (!status)
    {
        SetLastError(error);
    }
    return status;
}
