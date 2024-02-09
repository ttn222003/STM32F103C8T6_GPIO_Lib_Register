/*
 * stm32f103c8t6.h
 *
 *  Created on: Feb 3, 2024
 *      Author: Truong Trong Nhan
 */

#ifndef INC_STM32F103C8T6_H_
#define INC_STM32F103C8T6_H_

#include <stdint.h>

// Base address of Flash and SRAM memories
#define FLASH_BASE_ADDR		0x08000000U			// Main memory
#define SRAM_BASE_ADDR		0x20000000U
#define SYS_MEM_BASE_ADDR	0x1FFFF000U
#define SRAM				SRAM_BASE_ADDR

// Base address AHBx and APBx Bus
#define PERI_BASE_ADDR		0x40000000U
#define APB1_BASE_ADDR		PERI_BASE_ADDR
#define APB2_BASE_ADDR		0x40010000U
#define AHB_BASE_ADDR		0x40018000U
#define NVIC_BASE_ADDR		0xE000E100U

// Base address of Peripheral on APBx Bus
#define AFIO_BASE_ADDR		(APB2_BASE_ADDR)
#define EXTI_BASE_ADDR		(APB2_BASE_ADDR + 0x0400)

#define	GPIOA_BASE_ADDR		(APB2_BASE_ADDR + 0x0800)
#define GPIOB_BASE_ADDR		(APB2_BASE_ADDR + 0x0C00)
#define GPIOC_BASE_ADDR		(APB2_BASE_ADDR + 0x1000)

#define TIM1_BASE_ADDR		(APB2_BASE_ADDR + 0x2C00)

// Base address of Peripheral on AHBx Bus
#define	RCC_BASE_ADDR		(AHB_BASE_ADDR + 0x9000)

// Define struct of peripheral
typedef struct
{
	volatile uint32_t GPIOx_CRL;		// GPIO port register low							address offset: 0x00
	volatile uint32_t GPIOx_CRH;		// GPIO port register high							address offset: 0x04
	volatile uint32_t GPIOx_IDR;		// GPIO port input data register					address offset: 0x08
	volatile uint32_t GPIOx_ODR;		// GPIO port output data register					address offset: 0x0C
	volatile uint32_t GPIOx_BSRR;		// GPIO port bit set/reset register					address offset: 0x10
	volatile uint32_t GPIOx_BRR;		// GPIO port bit reset register						address offset: 0x14
	volatile uint32_t GPIOx_LCKR;		// GPIO port configuration lock register			address offset: 0x18
}GPIO_Reg_t;

typedef struct
{
	volatile uint32_t AFIO_EVCR;		// Event control register 							address offset: 0x00
	volatile uint32_t AFIO_MAPR;		// Remap and debug I/O configuration register		address offset: 0x04
	volatile uint32_t AFIO_EXTICR1;		// External interrupt configuration register 1		address offset: 0x08
	volatile uint32_t AFIO_EXTICR2;		// External interrupt configuration register 2		address offset: 0x0C
	volatile uint32_t AFIO_EXTICR3;		// External interrupt configuration register 3		address offset: 0x10
	volatile uint32_t AFIO_EXTICR4;		// External interrupt configuration register 4		address offset: 0x14
	volatile uint32_t AFIO_MAPR2;		// Remap and debug I/O configuration register 2		address offset: 0x18
}AFIO_Reg_t;

typedef struct
{
	volatile uint32_t EXTI_IMR;			// Interrupt mask register 							address offset: 0x00
	volatile uint32_t EXTI_EMR;			// Event mask register 								address offset: 0x04
	volatile uint32_t EXTI_RTSR;		// Rising trigger selection register 				address offset: 0x08
	volatile uint32_t EXTI_FTSR;		// Falling trigger selection register 				address offset: 0x0C
	volatile uint32_t EXTI_SWIER;		// Software interrupt event register  				address offset: 0x10
	volatile uint32_t EXTI_PR;			// Pending register 								address offset: 0x14
}EXTI_Reg_t;

typedef struct
{
	volatile uint32_t RCC_CR;			// Clock control register							address offset: 0x00
	volatile uint32_t RCC_CFGR;			// Clock configuration register						address offset: 0x04
	volatile uint32_t RCC_CIR;			// Clock interrupt register							address offset: 0x08
	volatile uint32_t RCC_APB2RSTR;		// APB2 peripheral reset register					address offset: 0x0C
	volatile uint32_t RCC_APB1RSTR;		// APB1 peripheral reset register					address offset: 0x10
	volatile uint32_t RCC_AHBENR;		// AHB peripheral clock enable register				address offset: 0x14
	volatile uint32_t RCC_APB2ENR;		// ABP2 peripheral clock enable register			address offset: 0x18
	volatile uint32_t RCC_APB1ENR;		// APB1 peripheral clock enable register			address offset: 0x1C
	volatile uint32_t RCC_BDCR;			// Backup domain control register					address offset: 0x20
	volatile uint32_t RCC_CSR;			// Control/status register							address offset: 0x24
	volatile uint32_t RCC_AHBRSTR;		// AHB peripheral clock reset register				address offset: 0x28
	volatile uint32_t RCC_CFGR2;		// Clock configuration register 2					address offset: 0x2C
}RCC_Reg_t;

typedef struct
{
	volatile uint32_t TIMx_CR1;			// Control register	1								address offset: 0x00
	volatile uint32_t TIMx_CR2;			// Control register 2								address offset: 0x04
	volatile uint32_t TIMx_SMCR;		// Slave mode control register						address offset: 0x08
	volatile uint32_t TIMx_DIER;		// DMA/Interrupt enable register					address offset: 0x0C
	volatile uint32_t TIMx_SR;			// Status register									address offset: 0x10
	volatile uint32_t TIMx_EGR;			// Event generation register						address offset: 0x14
	volatile uint32_t TIMx_CCMR1;		// Capture/Compare mode register 1					address offset: 0x18
	volatile uint32_t TIMx_CCMR2;		// Capture/Compare mode register 2					address offset: 0x1C
	volatile uint32_t TIMx_CCER;		// Capture/Compare enable register					address offset: 0x20
	volatile uint32_t TIMx_CNT;			// Counter											address offset: 0x24
	volatile uint32_t TIMx_PSC;			// Prescaler										address offset: 0x28
	volatile uint32_t TIMx_ARR;			// Auto-reload register								address offset: 0x2C
	volatile uint32_t TIMx_RCR;			// Repetition counter register						address offset: 0x30
	volatile uint32_t TIMx_CCR1;		// Capture/Compare register 1						address offset: 0x34
	volatile uint32_t TIMx_CCR2;		// Capture/Compare register 2						address offset: 0x38
	volatile uint32_t TIMx_CCR3;		// Capture/Compare register 3						address offset: 0x3C
	volatile uint32_t TIMx_CCR4;		// Capture/Compare register 4						address offset: 0x40
	volatile uint32_t TIMx_BDTR;		// Break and dead-time register						address offset: 0x44
	volatile uint32_t TIMx_DCR;			// DMA control register								address offset: 0x48
	volatile uint32_t TIMx_DMAR;		// DMA address for full transfer					address offset: 0x4C
}TIMx_Reg_t;

// Define struct for NVIC
typedef struct
{
	volatile uint32_t NVIC_ISERx;		// Interrupt set-enable registers					address offset: 0x00 - 0x0B
	volatile uint32_t NVIC_ICERx;		// Interrupt clear-enable registers 				address offset: 0x00 - 0x0B
	volatile uint32_t NVIC_ISPRx;		// Interrupt set-pending registers					address offset: 0x00 - 0x0B
	volatile uint32_t NVIC_ICPRx;		// Interrupt clear-pending registers 				address offset: 0x00 - 0x0B
	volatile uint32_t NVIC_IABRx;		// Interrupt active bit registers					address offset: 0x00 - 0x0B
	volatile uint32_t NVIC_IPRx;		// Interrupt priority registers						address offset: 0x00 - 0x0B
	volatile uint32_t NVIC_STIR;		// Software trigger interrupt register				address offset: 0xE0
}NVIC_Reg_T;

// Peripheral define address
#define GPIOA	((GPIO_Reg_t*)GPIOA_BASE_ADDR)
#define GPIOB	((GPIO_Reg_t*)GPIOB_BASE_ADDR)
#define GPIOC	((GPIO_Reg_t*)GPIOC_BASE_ADDR)
#define AFIO	((AFIO_Reg_t*)AFIO_BASE_ADDR)
#define EXTI	((EXTI_Reg_t*)EXTI_BASE_ADDR)

#define RCC		((RCC_Reg_t*)RCC_BASE_ADDR)

#define TIM1	((TIMx_Reg_t*)TIM1_BASE_ADDR)

// NVIC define
#define NVIC	((NVIC_Reg_T*)NVIC_BASE_ADDR)

// Define enable and disable for RCC peripheral
#define RCC_APB2EN_AFIO		(1<<0)
#define RCC_APB2EN_GPIOA	(1<<2)
#define RCC_APB2EN_GPIOB	(1<<3)
#define RCC_APB2EN_GPIOC	(1<<4)

#define RCC_APB2DIS_GPIOA	~(1<<2)
#define RCC_APB2DIS_GPIOB	~(1<<3)
#define RCC_APB2DIS_GPIOC	~(1<<4)

#define RCC_APB2RSTR_GPIOA	(1<<2)
#define RCC_APB2RSTR_GPIOB	(1<<3)
#define RCC_APB2RSTR_GPIOC	(1<<4)

// IRQ number
#define IRQ_NUM_EXTI0		6
#define IRQ_NUM_EXTI1		7
#define IRQ_NUM_EXTI2		8
#define IRQ_NUM_EXTI3		9
#define IRQ_NUM_EXTI4		10
#define IRQ_NUM_EXTI9_5		23
#define IRQ_NUM_EXTI15_10	40

// Some generic macro
#define ENABLE				1
#define DISABLE				0
#define SET					1
#define RESET				0
#define PIN_SET				1
#define PIN_RESET			0
#define RISING_EDGE			0
#define FALLING_EDGE		1
#define RISING_FALLING_EDGE	2

#endif /* INC_STM32F103C8T6_H_ */
