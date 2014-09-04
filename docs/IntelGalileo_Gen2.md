# Supporting the Intel Galileo Gen 2
The Intel Galileo is a maker target board which implements the pinout of an Arduino Uno. The Quark which powers the board makes extensive use of port expanders, muxes and logic gates to implement the specification as closely as possible.
 
During the early part of this year, the Windows team ported Windows to the Galileo Gen 1 and worked with Intel to develop a set of drivers which exposed the low level buses via Windows APIs - ultimately exposed by a Windows component called Embprpusr.dll - Embedded Peripheral User.
 
Galileo Gen 2 has a significantly different hardware layout which invalidates the existing driver stack. Fortunately, an alternative mechanism was identified for radically improving performance.
 
## Dmap Driver
The Intel Quark exposes a number of GPIO pins, I2C, and 2 SPI ports via a PCIe memory map, as well as a few GPIO via legacy IO map.
 
This memory map is exposed to user mode via the Direct map driver. Exposing the legacy IO map and PCIe memory segment to user mode allows user code to directly manipulate these features - without having to traverse a driver stack.
 
## Galileo SDK
When a developer's application initializes the Galileo SDK, the dmap driver is opened and the Quark functions are mapped and locked into application space.
 
### “Fabric” GPIO Pins
Eight GPIO pins are memory mapped in the Quark.  These are referred to as the “Fabric” GPIO pins.  Six of these eight pins can be routed to external Galileo Gen2 pins for digital I/O.  These pins are D0, D1, D2, D3, D10 and D12.  The Dmap driver can map the Quark GPIO registers for these pins directly into user virtual address space, so they can be changed very quickly.  They constitute the high performance digital I/O pins on the Galileo Gen2 board.  Once these pins are configured as desired, signals can be written to or read from them at speeds similar to memory accesses.
 
### “Legacy” GPIO Pins
Eight GPIO pins are I/O mapped in the Quark.  These are referred to as the “Legacy” GPIO pins.  Six of these eight pins can be routed to external Galileo Gen2 pins for digital I/O.  These pins are D4, D5, D6, D9, D11 and D13.  The Dmap driver can perform I/O operations on the behalf of user-mode code to access the registers for these GPIO pins.  They constitute the medium performance digital I/O pins on the Galileo Gen2 board.  Once these pins are configured, signals can be written to or read from them by performing DeviceIoControl() calls to the Dmap driver.
 
### I/O Expander GPIO Pins
Eight GPIO pins are not directly serviced by the Quark—they are implemented by I/O Expander ports.  These pins are D7, D8, A0, A1, A2, A3, A4 and A5.  The I/O Expanders are attached to the Quark by the I2C bus.  I2C transfers are required to write signals to or read them from these pins. These pins constitute the low performance digital I/O pins on the Galileo Gen2 board.
 
### I2C
The Arduino Wiring API exposes I2C via Wire.h. However, the I2C bus on the Galileo also serves other purposes.  The Gen2 has three I/O Expanders, a PWM chip and an Analog-to-Digital Converter (ADC) chip.  All of these are driven by the I2C bus.  The Gen2 also contains eleven digital multiplexors and four analog multiplexors.  These multiplexors are switched by a collection of signals from I/O Expander and PWM ports.   Some of the external I/O pins (D0-D13 and A0-A5) can operate in more than one mode.  Pin D3, for example can perform as a Digital I/O pin, a PWM output pin, or as the UART1 TX line.  One or more multiplexors are used to connect a pin to the particular function it should perform.
 
The I2C controller in the Quark is mapped in memory address space.  This allows the Dmap driver to map the Quark I2C controller registers directly in to user-mode address space, where these registers can be manipulated directly by the Galileo SDK code.  This means that the I2C transfers used to set up configurations or to write/read digital I/O signals can be performed without the need to call into drivers using DeviceIoControl() calls.  This provides speed and flexibility improvements.
 
### SPI
SPI is exposed by the Arduino Wiring API via SPI.h. SPI is also used to configure and read the ADC.
The Quark contains two SPI controllers.  One is used to drive the ADC and one is used for the SPI bus that is externally accessible (used by the Wiring API).   Like the I2C Controller in the Quark, the SPI Controller registers are mapped into memory space—using the Dmap driver these registers can be read and written directly by the Galileo SDK code.

---
#ADC
The ADC chip, Texas Instruments ADC108S102, operates via a dedicated SPI bus, SPI0.  The ADC has a conversion rate from .5 to 1 MS/s with 10-bit resolution. The mux selecting which analog input port to read, will be selected with an initial write operation. Subsequently, the analog signal will be captured and clocked-out over sixteen, rising-edge clock cycles.

##Serial Interface
Input (8-bit)

*format - XXX012XX
012 should be replaced with a binary number indicating the input channel you wish to read from.

NOTE: Input 6 and 7 are tied to ground.

Output (16-bit)

*format - XXXX0123456789XX (MSB first)
10-bit resolution of the analog signal.
## Implementation Details
###Hardware
*A0 - connected directly to ADC IN0
*A1 - connected directly to ADC IN1
*A2 - connected directly to ADC IN2
*A3 - connected directly to ADC IN3
*A4 - analog muxed (two times) between I2C, GPIO Expander 2 and ADC IN4
*A5 - analog muxed (two times) between I2C, GPIO Expander 2 and ADC IN5

We will communicate with the ADC controller over SPI using the "SpiController" layer. The pins will need to be activated using "PinSupport" functions (optimizations will need to be driven to the PinSupport layer to reflect the dedicated nature of pins A0-A3). Special code will need to be put in place to access the chip select GPIO pin coming directly off the CPU.Each read transaction will need to be made atomic, so it can be accessed simultaneously by multiple threads. Any operation specific code will be housed in a Class of the same name as the chip and will be accessible at the Win32 layer. Finally, the analog functions from the Wiring API, will be implemented atop the Win32 object.


The ADC functionality of the Galileo Gen2 is implemented using several chips

[ADC108S102 - ADC](http://www.ti.com/lit/ds/symlink/adc108s102.pdf)
8-Channel, 500 kSPS to 1 MSPS, 10-Bit A/D Converter

[TS5A23159 - Analog MUX](http://www.ti.com/lit/ds/scds201g/scds201g.pdf)
1-Ohm DUAL SPDT ANALOG SWITCH
5-V/3.3-V 2-CHANNEL 2:1 MULTIPLEXER/DEMULTIPLEXER

[PCAL9535AHF - GPIO Expander 2](http://www.nxp.com/documents/data_sheet/PCAL9535A.pdf)
Low-voltage 16-bit I2C-bus I/O port with interrupt and Agile I/O

### Things to Note:
* Base Address Register (BAR)
* (D:#, F:#) = Device #, Function # (All of these are on the PCI Bus 0)
* All GPIOs default to inputs.
* [Using DeviceIoControl](http://msdn.microsoft.com/en-us/library/windows/desktop/aa363216(v=vs.85).aspx)


### PCI Configuration Register Access
Access to PCI configuration space registers is performed through one of two different configuration access methods (CAMs):

* I/O indexed - PCI CAM
* Memory mapped - PCI Enhanced CAM (ECAM)

Each PCI function has a standard PCI header consisting of 256 bytes for the I/O access scheme (CAM), or 4096 bytes for the enhanced memory access method (ECAM). Invalid read accesses return binary strings of 1s.

---

# Fabric GPIO

Provides 8 GPIO pins from the Quark via Port A of the GPIO Controller

* 8 independently configurable GPIOs
* Separate data register and data direction for each GPIO
* Interrupt source mode supported for each FPIO
* De-bounce logic for interrupt sources
* Uses memory mapped register access.
* Maps to GPIO 0-7 with both I/O (page 738 on the Intel Quark SoC x1000)
* Register Map
    * I2C*/GPIO PCI Header (D:21, F:2) -> Memory Space
    * (BAR1 -> GPIO Mem Registers)

## Memory Referenced Register Access
* The SoC uses programmable BARs to set a range of physical address (memory) location that it uses to decode memory reads and writes from the CPU to directly access a register.
* These BARs act as pointers to blocks of actual memory mapped I/O (MMIO) registers.
* To access a memory referenced register for a specific base address, start with that base address and add the register's offset.
* Ex:
    * Register_Snapshot = MEMREAD([MEM_BAR]+Register_Offset)
* Memory Mapped I/O Register - 32 bits

## Implementation Plan:
1. CreateFile
    * check invalid handle
1. Retrieve base address for the GPIO controller using DeviceIoControl
    * check if failed
1. Get registerSnapshot using base address and register offset
1. Function to implement:
    1. Set to Output
    1. Set to Input
    1. Read
    1. Write
    

Example from DMapTest Solution:
``` C++
QUARK_FABRIC_GPIO *FabricGpioInit()
{
    wprintf(L"Initializing IOFABRIC GPIO\n");

    hDev = CreateFile(
        L"\\\\.\\PCI#VEN_8086&DEV_0934&SUBSYS_09348086&REV_10#3&b1bfb68&0&AA#{109b86ad-f53d-4b76-aa5f-821e2ddf2141}\\1",
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL);

    if (hDev == INVALID_HANDLE_VALUE)
    {
        wprintf(L"CreateFile failed! 0x%x\n", GetLastError());
        exit(1);
    }

    DMAP_MAPMEMORY_OUTPUT_BUFFER buf;
    DWORD bytesReturned;

    //  get address
    if (!DeviceIoControl(
        hDev,
        IOCTL_DMAP_MAPMEMORY,
        nullptr,
        0,
        &buf,
        sizeof(buf),
        &bytesReturned,
        nullptr))
    {
        wprintf(L"DeviceIoControl failed! 0x%x\n", GetLastError());
        exit(1);
    }

    QUARK_FABRIC_GPIO *fio = (QUARK_FABRIC_GPIO *)buf.Address;

    return fio;
}

//
// The pins with 1s in the mask will be set as inputs
//
inline void FabricGpioSetInput(QUARK_FABRIC_GPIO *fio, ULONG mask)
{
    // clear bits in direction register
    InterlockedAnd((volatile LONG *)&fio->GPIO_SWPORTA_DDR, ~(mask & 0xff));
    //fio->GPIO_SWPORTA_DDR &= ~(mask & 0xff);
}

//
// The pins with 1s in the mask will be set as outputs
//
inline void FabricGpioSetOutput(QUARK_FABRIC_GPIO *fio, ULONG mask)
{
    // set bits in direction register
    InterlockedOr((volatile LONG *)&fio->GPIO_SWPORTA_DDR, mask & 0xff);
    //fio->GPIO_SWPORTA_DDR |= (mask & 0xff);
}

inline void FabricGpioWritePins(QUARK_FABRIC_GPIO *fio, ULONG mask)
{
    fio->GPIO_SWPORTA_DR = (mask & 0xff);
}

//
// The pins with 1s in the mask will be set HIGH
//
inline void FabricGpioSetPins(QUARK_FABRIC_GPIO *fio, ULONG mask)
{
    // set bits in data register
    InterlockedOr((volatile LONG *)&fio->GPIO_SWPORTA_DR, mask & 0xff);
    //fio->GPIO_SWPORTA_DR |= (mask & 0xff);
}

//
// The pins with 1s in the mask will be set LOW
//
inline void FabricGpioClearPins(QUARK_FABRIC_GPIO *fio, ULONG mask)
{
    // clear bits in data register
    InterlockedAnd((volatile LONG *)&fio->GPIO_SWPORTA_DR, ~(mask & 0xff));
    //fio->GPIO_SWPORTA_DR &= ~(mask & 0xff);
}

//
// Read state of all pins
//
ULONG FabricGpioRead(QUARK_FABRIC_GPIO *fio)
{
    return fio->GPIO_EXT_PORTA;
}
```
    
---
    
# Legacy GPIO
* Uses fixed I/O register access.
* Maps to GPIO 8-9 and GPIO_SUS 0-5
* Each GPIO has six registers that control how it is used, or report its status:
    * Use Select - selects a pin as GPIO, or leaves it as its programmed function
    * I/O Select - determines the direction of the GPIO
    * GPIO Level - 
    * Trigger Positive Edge - Enables general purpose events on a rising edge (Only applies to GPIOs set to input)
    * Trigger Negative Edge - Enables general purpose events on a falling edge (Only applies to GPIOs set to input)
    * Trigger Status - determines if the GPIO triggered a GPE (only applies to GPIOs set to input and one of the trigger modes enabled)
* Register Map
    * Legacy PCI Header (D:31, F:0) -> IO Space
    * (GPIO_BASE_ADDRESS -> Legace GPIO Registers)

## I/O Referenced Register Access
* I/O referenced registers use programmable BARs to select a range of I/O addresses that it uses to decode PORT IN and/or PORT OUT transactions from the CPU to directly access a register.
* I/O BARs act as pointers to blocks of actual I/O registers.
* To access an I/O referenced register for a specific I/O base address, start with that base address and add the register's offset.
* Ex:
    * Register_Snapshot = IOREAD([IO_BAR]+Register_Offset)
* I/O Register - 16 bits

## Implementation Plan:
1. CreateFile
    * check invalid handle
1. Retrieve base address for the GPIO controller using DeviceIoControl
    * check if failed
1. Get registerSnapshot using base address and register offset
1. Function to implement:
    1. Set to Output
    1. Set to Input
    1. Read
    1. Write
    
Example from DMapTest Solution:
``` C++
void WritePortUChar(ULONG address, UCHAR value)
{
	DMAP_WRITEPORT_INPUT_BUFFER inp;
	DWORD bytesReturned;

	inp.Address = address;
	inp.Value = value;

	//  Perform the write operation
	if (!DeviceIoControl(
		hDev,
		IOCTL_DMAP_WRITEPORT,
		&inp,
		sizeof(inp),
		nullptr,
		0,
		&bytesReturned,
		nullptr))
	{
		wprintf(L"DeviceIoControl failed! 0x%x\n", GetLastError());
		exit(1);
	}
}

UCHAR ReadPortUChar(ULONG address)
{
	DWORD bytesReturned;
	UCHAR value;

	//  Perform the write operation
	if (!DeviceIoControl(
		hDev,
		IOCTL_DMAP_READPORT,
		&address,
		sizeof(address),
		&value,
		sizeof(value),
		&bytesReturned,
		nullptr))
	{
		wprintf(L"DeviceIoControl failed! 0x%x\n", GetLastError());
		exit(1);
	}

	return value;
}

void LGpioSetDirection(ULONG mask)
{
	WritePortUChar(GBA + RGIO, UCHAR(~mask & 0x1f));
}

void LGpioSet(ULONG mask)
{
	ULONG temp = ReadPortUChar(GBA + RGLVL);
	temp |= mask;
	WritePortUChar(GBA + RGLVL, UCHAR(temp & 0x1f));
}

void LGpioClear(ULONG mask)
{
	ULONG temp = ReadPortUChar(GBA + RGLVL);
	temp &= ~mask;
	WritePortUChar(GBA + RGLVL, UCHAR(temp & 0x1f));
}

ULONG LGpioRead()
{
	return ReadPortUChar(GBA + RGLVL);
}

```

---

# Write Arduino APIs for GPIO

Will need to configure a lot of different Muxes / external GPIO port expanders in order to get the right configurations

* 6 GPIO pins are driven from the Legacy GPIO 
    * D0, D1, D2, D3, D10, D12
* 6 GPIO pins are driven from the Fabric GPIO
    * D4, D5, D6, D9, D11, D13
* 8 GPIO pins are driven off of I2C (6 are Analog, 2 are Digital)
    * D7, D8, A0, A1, A2, A3, A4, and A5

## Using the table designated in the PinSupport:
```
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
```

### Hardware
* [PCAL9535AHF (EXP0, EXP1, and EXP2)](http://www.nxp.com/products/interface_and_connectivity/i2c/i2c_general_purpose_i_o/PCAL9535AHF.html)

  Low voltage 16-bit I2C-bus I/O port with interrupt and Agile I/O

* [SN74LV125A](http://www.ti.com/product/sn74lv125a)

  Quadruple Bus Buffer Gates with 3-State Outputs
  3 different ones:
  * U26
      * IO0, IO2, IO4, IO12
  * U18
      * IO3, IO5, IO6, IO9
  * U15
      * IO11, IO10, IO1, IO13

* [SN74LV541AT](http://www.ti.com/product/sn74lv541at)

  Octal Buffer/Driver with 3-State Outputs

  * Drives the IO13 LED, BUF_IO0, BUF_IO2, BUF_IO4, BUF_IO12, 

### Quark
[Quark Datasheet](https://communities.intel.com/servlet/JiveServlet/previewBody/21828-102-3-26270/Quark%20Datasheet%20Rev02.pdf)
Drives the UART 0 AND UART 1

* SIU0-RXD = UART0_RXD (IN)
* SIU0-TXD = UART0_TXD (OUT)
* UART1_CTS_N = SIU1_CTS_B (IN)
* UART1_RTS_N = SIU1_RTS_B (OUT)
* UART1_RXD = SIU1_RXD (IN)
* UART1_TXD = SIU1_TXD (OUT)

From page 7 of the Galileo Gen2 PVT Schematic