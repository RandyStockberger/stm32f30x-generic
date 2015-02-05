/* ==================================================================== **

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

** ==================================================================== */

//
// stm32f302r8.h -- system defintions for this ARM Cortex-M4 chip
//

#ifndef _STM32F320R8_H_
#define _STM32F320R8_H_

#define APB1_BASE	0x40000000
#define APB2_BASE	0x40010000
#define AHB1_BASE	0x40020000
#define AHB2_BASE	0x48000000
#define AHB3_BASE	0x50000000
//
// APB1 resources
#define TIM2_BASE		(APB1_BASE+0x00000000)
#define TIM6_BASE		(APB1_BASE+0x00001000)
#define RTC_BASE		(APB1_BASE+0x00002800)
#define WWDG_BASE		(APB1_BASE+0x00002C00)
#define IWDG_BASE		(APB1_BASE+0x00003000)
#define I2S2_BASE		(APB1_BASE+0x00003400)
#define SPI2_BASE		(APB1_BASE+0x00003800)
#define SPI3_BASE		(APB1_BASE+0x00003C00)
#define I2S3_BASE		(APB1_BASE+0x00004000)
#define USART2_BASE		(APB1_BASE+0x00004400)
#define USART3_BASE		(APB1_BASE+0x00004800)
#define I2C1_BASE		(APB1_BASE+0x00005400)
#define I2C2_BASE		(APB1_BASE+0x00005800)
#define USBFS_BASE		(APB1_BASE+0x00005C00)
#define USBCANSRAM_BASE	(APB1_BASE+0x00006000)
#define BXCAN_BASE		(APB1_BASE+0x00006400)
#define PWR_BASE		(APB1_BASE+0x00007000)
#define DAC1_BASE		(APB1_BASE+0x00007400)
#define I2C3_BASE		(APB1_BASE+0x00007800)
//
// APB2 resources
#define SYSCFG_BASE		(APB2_BASE+0x00000000)
#define EXT1_BASE		(APB2_BASE+0x00000400)
#define TIM1_BASE		(APB2_BASE+0x00002C00)
#define USART1_BASE		(APB2_BASE+0x00003800)
#define TIM15_BASE		(APB2_BASE+0x00004000)
#define TIM16_BASE		(APB2_BASE+0x00004400)
#define TIM17_BASE		(APB2_BASE+0x00004800)
//
// AHB1 resources
#define DMA1_BASE		(AHB1_BASE+0x00000000)
#define	RCC_BASE		(AHB1_BASE+0x00001000)
#define FLASH_BASE		(AHB1_BASE+0x00002000)
#define CRC_BASE		(AHB1_BASE+0x00003000)
#define TSC_BASE		(AHB1_BASE+0x00004000)
//
// AHB2 resources
#define	GPIOA_BASE		(AHB2_BASE+0x00000000)
#define	GPIOB_BASE		(AHB2_BASE+0x00000400)
#define	GPIOC_BASE		(AHB2_BASE+0x00000800)
#define	GPIOD_BASE		(AHB2_BASE+0x00000C00)
//#define	GPIOE_BASE		(AHB2_BASE+0x00001000)
#define	GPIOF_BASE		(AHB2_BASE+0x00001400)

typedef struct
{
	volatile uint32_t CR;		/* RCC clock control register,					Address offset: 0x00 */
	volatile uint32_t CFGR;		/* RCC clock configuration register,			Address offset: 0x04 */
	volatile uint32_t CIR;		/* RCC clock interrupt register,				Address offset: 0x08 */
	volatile uint32_t APB2RSTR;	/* RCC APB2 peripheral reset register,			Address offset: 0x0C */
	volatile uint32_t APB1RSTR;	/* RCC APB1 peripheral reset register,			Address offset: 0x10 */
	volatile uint32_t AHBENR;	/* RCC AHB peripheral clock register,			Address offset: 0x14 */
	volatile uint32_t APB2ENR;	/* RCC APB2 peripheral clock enable register,	Address offset: 0x18 */
	volatile uint32_t APB1ENR;	/* RCC APB1 peripheral clock enable register,	Address offset: 0x1C */
	volatile uint32_t BDCR;		/* RCC Backup domain control register,			Address offset: 0x20 */ 
	volatile uint32_t CSR;		/* RCC clock control & status register,			Address offset: 0x24 */
	volatile uint32_t AHBRSTR;	/* RCC AHB peripheral reset register,			Address offset: 0x28 */
	volatile uint32_t CFGR2;	/* RCC clock configuration register 2,			Address offset: 0x2C */
	volatile uint32_t CFGR3;	/* RCC clock configuration register 3,			Address offset: 0x30 */ 
} RCC_TypeDef;

#define RCC		((RCC_TypeDef *)RCC_BASE)
//
// RCC->CR bits
#define	PLLRDY		25
#define	PLLON		24
#define HSIRDY		 1
#define HSION		 0

// RCC->CFGR bits
#define PLLNODIV	31
#define	MCOPRE		28
#define	MCO			24
#define	I2SSRC		23
#define USBPRE		22
#define PLLMUL		18
#define	PLLXTPRE	17
#define PLLSRC		16
#define PPRE2		11
#define PPRE1		 8
#define HPRE		 4
#define	SWS			 2
#define SW			 0

// ============================================================================

// AHB3 resources
#define ADC1_BASE	(APB3_BASE+0x00000000)

// APB1ENR bits
#define I2C3EN		30
#define	DAC1EN		29
#define PWREN		28
#define CANEN		25
#define USBEN		23
#define I2C2EN		22
#define I2C1EN		21
#define USART5EN	20
#define USART4EN	19
#define USART3EN	18
#define USART2EN	17
#define SPI3EN		15
#define SPI2EN		14
#define WWDGEN		11
#define TIM6EN		 4
#define TIM4EN		 2
#define TIM3EN		 1
#define TIM2EN		 0

// AHBENR bits
#define IOPFEN		22		// GPIOF enable
#define IOPEEN		21		// GPIOE enable
#define IOPDEN		20		// GPIOD enable
#define IOPCEN		19		// GPIOC enable
#define IOPBEN		18		// GPIOB enable
#define IOPAEN		17		// GPIOA enable

// ============================================================================

typedef struct
{
	volatile uint32_t ACR;		/* Flash Access Control Register */
	volatile uint32_t KEYR;		/* Flash Key Register */
	volatile uint32_t OPTKEYR;	/* Flash OptionKey Register */
	volatile uint32_t SR;		/* Flash Status Register */
	volatile uint32_t CR;		/* Flash Control Register */
	volatile uint32_t AR;		/* Flash Address Register */
	volatile uint32_t OBR;		/* Flash Option Byte Register */
			 uint32_t RES1;		/* Reserved/Unused */
	volatile uint32_t WRPR;		/* Flash Write Protection Register */
} FLASH_TypeDef;

#define FLASH	((FLASH_TypeDef *)FLASH_BASE)

// FLASH->ACR bits
#define FLASH_ACR_PRFTBS		5
#define FLASH_ACR_PRFTBE		4
#define FLASH_ACR_HLFCYA		3
#define FLASH_ACR_LATENCY		0

// ============================================================================

typedef struct
{
	volatile uint32_t MODER;		/* GPIO port mode register,					Address offset: 0x00 */
	volatile uint16_t OTYPER;		/* GPIO port output type register,			Address offset: 0x04 */
			 uint16_t RESERVED0; 	/* Reserved,												0x06 */
	volatile uint32_t OSPEEDR;		/* GPIO port output speed register,			Address offset: 0x08 */
	volatile uint32_t PUPDR;		/* GPIO port pull-up/pull-down register,	Address offset: 0x0C */
	volatile uint16_t IDR;			/* GPIO port input data register,			Address offset: 0x10 */
			 uint16_t RESERVED1;	/* Reserved,												0x12 */
	volatile uint16_t ODR;			/* GPIO port output data register,			Address offset: 0x14 */
			 uint16_t RESERVED2;	/* Reserved,												0x16 */
	volatile uint32_t BSRR;			/* GPIO port bit set/reset registerBSRR,	Address offset: 0x18 */
	volatile uint32_t LCKR;			/* GPIO port configuration lock register,	Address offset: 0x1C */
	volatile uint32_t AFRL;			/* GPIO alternate function low register,	Address offset: 0x20 */
	volatile uint32_t AFRH;			/* GPIO alternate function high register,	Address offset: 0x24 */
	volatile uint16_t BRR;			/* GPIO bit reset register,					Address offset: 0x28 */
			 uint16_t RESERVED3;	/* Reserved,												0x2A */
} GPIO_TypeDef;

#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALTFUNC		2
#define GPIO_MODE_ANALOG		3

#define GPIOA		((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB		((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC		((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD		((GPIO_TypeDef *) GPIOD_BASE)
//#define GPIOE		((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF		((GPIO_TypeDef *) GPIOF_BASE)

// ============================================================================

typedef struct
{
	volatile uint32_t	CR1;		/* 0x00 - Control Register 1 */
	volatile uint32_t	CR2;		/* 0x04 - Control Register 2 */
	volatile uint32_t	CR3;		/* 0x08 - Control Register 3 */
	volatile uint32_t	BRR;		/* 0x0C - Baud Rate Register */
	volatile uint32_t	GTPR;		/* 0x10 - Guard Time/Prescaler Register */
	volatile uint32_t	RTOR;		/* 0x14 - Receiver Timeout Register */
	volatile uint32_t	RQR;		/* 0x18 - Request Register */
	volatile uint32_t	ISR;		/* 0x1C - Interrupt and Status Register */
	volatile uint32_t	ICR;		/* 0x20 - Interrupt Flag Clear Register */
	volatile uint32_t	RDR;		/* 0x24 - Receive Data Register */
	volatile uint32_t	TDR;		/* 0x28 - Transmit Data Register */
} USART_TypeDef;

#define USART1		((USART_TypeDef *) USART1_BASE)
#define USART2		((USART_TypeDef *) USART2_BASE)
#define USART3		((USART_TypeDef *) USART3_BASE)

// Field definitions - USART CR1
#define USART_CR1_M1		28
#define USART_CR1_OVER8		15
#define USART_CR1_M0		12
#define USART_CR1_PCE		10
#define USART_CR1_TE		 3
#define USART_CR1_RE		 2
#define USART_CR1_UE		 0

// Field definitions - USART CR2
#define USART_CR2_STOP	12

// Field definitions - USART CR2
#define	USART_ISR_TXE	 7
#define	USART_ISR_TC	 6
#define	USART_ISR_RXNE	 5

// ============================================================================

typedef struct
{
	volatile uint32_t CR;			/* Control Register						0x00 */
	volatile uint32_t SWTRIGR;		/* Software Trigger Register			0x04 */
	volatile uint32_t DHR12R1;		/* 12-bit right aligned data register	0x08 */
	volatile uint32_t DHR12L1;		/* 12-bit left aligned data register	0x0C */
	volatile uint32_t DHR8R1;		/* 8-bit right-aligned data register	0x10 */
			 uint32_t RES0[6];		/* 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28 */
	volatile uint32_t DOR1;			/* Data output register					0x2C */
			 uint32_t RES6;			/* Reserved								0x30 */
	volatile uint32_t SR;			/* Reserved								0x34 */
} DAC_TypeDef;

#define DAC	((DAC_TypeDef *) DAC1_BASE)

// ============================================================================

#define SYSTICK_BASE	0xE000E010

typedef struct
{
	volatile uint32_t STK_CTRL;		/* RW - SysTick Control and Status register */
	volatile uint32_t STK_LOAD;		/* RW - SysTick reload value register */
	volatile uint32_t STK_VAL;		/* RW - SysTick current value register */
	volatile uint32_t STK_CALIB;	/* RO - SysTick calibration value register */
} SysTick_TypeDef;

#define SYSTICK		((SysTick_TypeDef *)SYSTICK_BASE)

// ============================================================================

#endif // _STM32F320R8_H_
