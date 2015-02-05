
/* ============================================================================ **

  The MIT License (MIT)

  Copyright (c) 2015 - Randall L. Stockberger

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

** ============================================================================ */

//
// Command line:
// 	no FP
//		gcc -mthumb -mcpu=cortex-m4
//		gcc -mthumb -march=armv7e-m
// 	soft FP
//		gcc -mthumb -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16
//		gcc -mthumb -march=armv7e-m -mfloat-abi=softfp -mfpu=fpv4-sp-d16
// 	hard FP
//		gcc -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
//		gcc -mthumb -march=armv7e-m -mfloat-abi=hard -mfpu=fpv4-sp-d16
//
// Multilib - armv7e-m
//
// STM32F302R8 - Cortex-M4 w/ 64K Flash @ 0x00000000, 16K RAM @ 0x20000000
//
// System peripherial locations (DM00094349-RM0365 - STM32F302x6/8 Reference Manual)
//

#include <stdint.h>

#include "isr.h"

extern void _reset( void );

extern uint32_t _stacktop;
extern uint32_t _sdata, _edata, _etext;
extern uint32_t _sbss, _ebss;

__attribute__ ((section(".isr_vector"))) void (* const g_pfnVectors[])(void) =
{
	(void (*)(void))((uint32_t)&_stacktop),	// 0x00
	_reset,						// 0x0004
	ISRNmi,						// 0x0008
	ISRHardFault,				// 0x000C
	ISRMemManage,				// 0x0000
	ISRBusFault,				// 0x0014
	ISRUsageFault,				// 0x0018
	0,							// 0x001C
	0,							// 0x0020
	0,							// 0x0024
	0,							// 0x0028
	ISRSVCall,					// 0x002C
	0,							// 0x0030
	0,							// 0x0034
	ISRPendSV,					// 0x0038
	ISRSysTick,					// 0x003C
	DefaultHandler,				// WWDG		0x0040
	DefaultHandler,				// PVD		0x0044
	DefaultHandler,				// T_STAMP	0x0048
	DefaultHandler,				// RTC_WKUP	0x004C
	DefaultHandler,				// FLASH	0x0050
	DefaultHandler,				// 0x0054
	DefaultHandler,				// 0x0058
	DefaultHandler,				// 0x005C
	DefaultHandler,				// 0x0060
	DefaultHandler,				// 0x0064
	DefaultHandler,				// 0x0068
	DefaultHandler,				// 0x006C
	DefaultHandler,				// 0x0070
	DefaultHandler,				// 0x0074
	DefaultHandler,				// 0x0078
	DefaultHandler,				// 0x007C
	DefaultHandler,				// 0x0080
	DefaultHandler,				// 0x0084
	DefaultHandler,				// 0x0088
	DefaultHandler,				// 0x008C
	DefaultHandler,				// 0x0090
	DefaultHandler,				// 0x0094
	DefaultHandler,				// 0x0098
	DefaultHandler,				// 0x009C
	DefaultHandler,				// 0x00A0
	DefaultHandler,				// 0x00A4
	DefaultHandler,				// 0x00A8
	DefaultHandler,				// 0x00AC
	DefaultHandler,				// 0x00B0
	DefaultHandler,				// 0x00B4
	DefaultHandler,				// 0x00B8
	DefaultHandler,				// 0x00BC
	DefaultHandler,				// 0x00C0
	DefaultHandler,				// 0x00C4
	DefaultHandler,				// 0x00C8
	DefaultHandler,				// 0x00CC
	DefaultHandler,				// 0x00D0
	DefaultHandler,				// 0x00D4
	DefaultHandler,				// 0x00D8
	DefaultHandler,				// 0x00DC
	DefaultHandler,				// 0x00E0
	DefaultHandler,				// 0x00E4
	DefaultHandler,				// 0x00E8
	DefaultHandler,				// 0x00EC
	DefaultHandler,				// 0x00F0
	DefaultHandler,				// 0x00F4
	DefaultHandler,				// 0x00F8
	DefaultHandler,				// 0x00FC
	DefaultHandler,				// 0x0100
	DefaultHandler,				// 0x0104
	DefaultHandler,				// 0x0108
	DefaultHandler,				// 0x010C
	DefaultHandler,				// 0x0110
	DefaultHandler,				// 0x0114
	DefaultHandler,				// 0x0118
	DefaultHandler,				// 0x011C
	DefaultHandler,				// 0x0120
	DefaultHandler,				// 0x0124
	DefaultHandler,				// 0x0128
	DefaultHandler,				// 0x012C
	DefaultHandler,				// 0x0130
	DefaultHandler,				// 0x0134
	DefaultHandler,				// 0x0138
	DefaultHandler,				// 0x013C
	DefaultHandler,				// 0x0140
	DefaultHandler,				// 0x0144
	DefaultHandler,				// 0x0148
	DefaultHandler,				// 0x014C
	DefaultHandler,				// 0x0150
	DefaultHandler,				// 0x0154
	DefaultHandler,				// 0x0158
	DefaultHandler,				// 0x015C
	DefaultHandler,				// 0x0160
	DefaultHandler,				// 0x0164
	DefaultHandler,				// 0x0168
	DefaultHandler,				// 0x016C
	DefaultHandler,				// 0x0170
	DefaultHandler,				// 0x0174
	DefaultHandler,				// 0x0178
	DefaultHandler,				// 0x017C
	DefaultHandler,				// 0x0180
	DefaultHandler,				// 0x0184
};

void _reset( void )
{
	unsigned long * pSrc, *pDst;

	// Copy data initializers from Flash to the appropriate place in SRAM
	// Linker script defines _data, _edata and _etext
	pSrc = &_etext;
	pDst = &_sdata;
	while ( pDst < &_edata ) {
		*pDst++ = *pSrc++;
	}

	// Zero the .bss segment
	pDst = &_sbss;
	while ( pDst < &_ebss ) {
		*pDst = 0;
	}

	// Floating point initialization
	// ??

	main();

	while(1)		// Main never exits, but if it does...
		;
}

