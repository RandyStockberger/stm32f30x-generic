// ============================================================================
//
// generic.c -- boilerplate code for an ARM STM32F30x program
//
// ============================================================================

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

#include <stdint.h>

#include "stm32f302r8.h"
#include "generic.h"

// ============================================================================
// Global declarations

volatile uint32_t SysTickCnt;

// ============================================================================
//
void InitSystemClock( void )
{

    // Initializing of PLL for using a higher internal clock-speed (64MHz)
    // Enable Prefetch Buffer and set Flash Latency
	//
	// Flash wait states:
	//	0..24MHz	- 0 wait states
	// 24..48MHz	- 1 wait state
	// 48..72MHz	- 2 wait states 
    FLASH->ACR = (1<<FLASH_ACR_PRFTBE) | (2<<FLASH_ACR_LATENCY);
 
    //configure prescaler
    RCC->CFGR |= (uint32_t)(0<<HPRE) |		// HCLK == SYSCLK
                 (uint32_t)(4<<PPRE1) |		// APB1 prescaler == 2
                 (uint32_t)(0<<PPRE2);		// APB2 prescaler == 2
 
    // PLL configuration
	// PLL Clock source is HSI/2 or 4MHz
	// PLL multiplier is 16 which gives a SYSCLOCK of 64
	//clear PLLSRC and PLLMUL
    RCC->CFGR &= (uint32_t)((uint32_t)~((0x01<<PLLSRC) | (0x0F<<PLLMUL)));
	//PLL gets HSI/2, PLL-factor is 16 => 64MHz
    RCC->CFGR |= (uint32_t)((0<<PLLSRC) | (14<<PLLMUL));
 
    RCC->CR |= 1<<PLLON; // Enable PLL

	// Wait till PLL is ready
    while((RCC->CR & (1<<PLLRDY)) == 0)
		;
 
	// PLL has locked, select PLL as system clock source
    RCC->CFGR &= (uint32_t)((uint32_t)~(3<<SW));	//clear old bits in register
    RCC->CFGR |= (uint32_t)(2<<SW);

    // Wait until system has switched to PLL as system clock
    while ((RCC->CFGR & (uint32_t)(3<<SWS)) != (uint32_t)(2<<SWS))
		;
}

// ============================================================================
// InitMonitor -- Initialize the Montitor USART
//
// Config GPIO for USART TX and RX, including GPIO alternate functions
// Config USART for BAUD, word size, stop bits and parity
//
#ifdef UART
void InitMonitor( void )
{
	// Power up the USART
	RCC->APB1ENR |= (1<<USART2EN);				// Turn on the USART clock
	
	// Configure the GPIO and alternate function for TX pin
	MON_GPIO->MODER &= ~(3ul<<(MON_PIN_TX*2));	// Clear previous mode
	MON_GPIO->MODER |= GPIO_MODE_ALTFUNC<<(MON_PIN_TX*2);// Set Alt. Func. mode
	MON_GPIO->OTYPER &= ~(1ul<<MON_PIN_TX);		// TX is Push/Pull
//	MON_GPIO->OSPEEDR |= (3ul<<(MON_PIN_TX*2));	// High Speed
	MON_GPIO->PUPDR &= ~(3ul<<(MON_PIN_TX*2));	// Disable pull up, pull down
	MON_GPIO->AFRL &= ~(0x0Ful<<(MON_PIN_TX*4));// Clear previous Alt Func
	MON_GPIO->AFRL |= (0x07ul<<(MON_PIN_TX*4));	// Set new alt func

	// Configure the GPIO and alternate function for RX pin
	MON_GPIO->MODER &= ~(3ul<<(MON_PIN_RX*2));	// Clear previous mode
	MON_GPIO->MODER |= GPIO_MODE_ALTFUNC<<(MON_PIN_RX*2);// Set Alt. Func. mode
	MON_GPIO->OTYPER |= 1ul<<MON_PIN_RX;		// RX is not Push/Pull
//	MON_GPIO->OSPEEDR |= (3ul<<(MON_PIN_RX*2));	// High Speed
	MON_GPIO->PUPDR &= ~(3ul<<(MON_PIN_RX*2));	// Disable pull up and pull down
	MON_GPIO->AFRL &= ~(0x0Ful<<(MON_PIN_RX*4));// Clear previous Alt Func
	MON_GPIO->AFRL |= (0x07ul<<(MON_PIN_RX*4));	// Set new alt func

	// Configure the USART
	//RCC->CFGR3 |= 1<<16;						// USART2 clock from SYSCLK

	MON_USART->CR1 &= ~(1<<USART_CR1_UE);		// Disable the USART
	MON_USART->CR1 &= ~((1<<USART_CR1_M1) | (1<<USART_CR1_M0) |
			(1<<USART_CR1_OVER8) |
			(1<<USART_CR1_PCE) );	// 8 bits, 16x oversample, no parity
	MON_USART->BRR = (FCPU/2) / MON_BAUD;		// set baud rate in BRR
	MON_USART->CR2 &= ~(3<<USART_CR2_STOP);		// set 1 stop bit
	MON_USART->CR1 |= 1<<USART_CR1_UE;			// enable USART 
	// Set DMA transmit enable CR3:DMAT
	// Set DMA receive enable CR3:DMAR
	MON_USART->CR1 |= ( (1<<USART_CR1_TE) | (1<<USART_CR1_RE) );// enable TX and RX
}

// ============================================================================
// uputc -- Send a character to the monitor usart
//
void uputc( int ch )
{
	while ( (MON_USART->ISR & (1<<USART_ISR_TXE) ) == 0 )
		;
	MON_USART->TDR = ch;
}
#endif	// UART

// ============================================================================
//
void Init( void )
{
	InitSystemClock();					// set the system clock

	// Enable clock for GPIO port B and set bit13 as output
	RCC->AHBENR |= (1<<IOPAEN) | (1<<IOPBEN);	// Power up GPIOA and GPIOB
	GPIOB->MODER |= (1<<(LD2*2));		// Output mode
	GPIOB->ODR &= ~(1<<LD2);			// Turn off the bit

	// Configure SysTick interrupt to give heartbeat interrupts
	SYSTICK->STK_LOAD = SYSTICK_CNT;
	SYSTICK->STK_VAL = (SYSTICK_CNT)-1;
	SYSTICK->STK_CTRL |= (1<<SYSTICK_CLKSOURCE) | (1<<TICKINT) | (1<<SYSTICK_ENABLE);

#ifdef UART
	InitMonitor();
#endif	// UART
}

// ============================================================================
//

#define CNT_ON		0
#define CNT_OFF		10
#define CNT_LIMIT	50

void main( void )
{
	int32_t cnt = CNT_OFF;
	uint32_t CurTick;

	Init();

	while( 1 ) {
		if ( SysTickCnt >= HB_HZ ) {
			SysTickCnt -= HB_HZ;
		}

		if ( SysTickCnt != CurTick ) {

			if ( SysTickCnt < (HB_HZ/100) )
				GPIOB->BSRR = 1<<LD2;
			else
				GPIOB->BRR = 1<<LD2;

			CurTick = SysTickCnt;
		}
	}
}

// ============================================================================

