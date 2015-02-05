
#ifndef __GENERIC_H__
#define __GENERIC_H__

/* ============================================================================ **
 *
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

// System defines
//
// CPU Boots with an 8MHz
//#define FCPU				8000000ul
//
// Maximum SYSCLK with HSI source clock
#define FCPU				64000000ul

#define HB_HZ				500

#define SYSTICK_DIVISOR		1ul

//STK_CTRL fields
#define SYSTICK_ENABLE		0
#define TICKINT				1
#define SYSTICK_CLKSOURCE	2
#define SYSTICK_COUNTFLAG	16

#define SYSTICK_CNT			(((FCPU)/SYSTICK_DIVISOR)/HB_HZ)

// User driven LED is on port B.13
#define LD2		13

// UART2 -- Communicates with host PC monitor program
// Monitor USART is USART2 on GPIOA pins 2 and 3
#define	MON_GPIO	GPIOA
#define MON_USART	USART2
#define MON_PIN_TX	2
#define MON_PIN_RX	3

#define MON_BAUD		9600

// ============================================================================
// Typedef and Structure definitions
//

// ============================================================================
// Globals
//
extern volatile uint32_t SysTickCnt;

// ============================================================================

#endif	// __GENERIC_H__

