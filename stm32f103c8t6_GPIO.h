/*
 * stm32f103c8t6_GPIO.h
 *
 *  Created on: Feb 3, 2024
 *      Author: Truong Trong Nhan
 */

#ifndef INC_STM32F103C8T6_GPIO_H_
#define INC_STM32F103C8T6_GPIO_H_

#include "stm32f103c8t6.h"

//@GPIO_PIN_NUMBER
// Define pin number for GPIO
#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12
#define GPIO_PIN_13		13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15

// Define pin number for EXTI
#define EXTI0			0
#define EXTI1			1
#define EXTI2			2
#define EXTI3			3
#define EXTI4			4
#define EXTI5			5
#define EXTI6			6
#define EXTI7			7
#define EXTI8			8
#define EXTI9			9
#define EXTI10			10
#define EXTI11			11
#define EXTI12			12
#define EXTI13			13
#define EXTI14			14
#define EXTI15			15

// Define Port
#define PortA			0
#define PortB			1
#define PortC			2

//@GPIO_PIN_PURPOSE
// Define input configuration for GPIO
#define GPIO_ANALOG_MODE					0x00
#define GPIO_FLOATING_INPUT_MODE			0x01
#define GPIO_INPUT_PULLUP_PULLDOWN_MODE		0x02
#define GPIO_RESERVE_MODE					0x03

//@GPIO_PIN_PURPOSE
// Define output configuration for GPIO
#define GPIO_PUSHPULL_MODE					0x00
#define GPIO_OPEN_DRAIN_MODE				0x01
#define	ALT_FUNC_OUTPUT_PUSHPULL_MODE		0x02
#define ALT_FUNC_OUTPUT_OPEN_DRAIN			0x03

//@GPIO_PIN_MODE
// Define mode for GPIO
#define	GPIO_INPUT_MODE						0x00
#define GPIO_OUTPUT_MODE_10MHZ				0x01
#define GPIO_OUTPUT_MODE_2MHZ				0x02
#define GPIO_OUTPUT_MODE_50MHZ				0x03

// Define NVIC register
#define NVIC_ISER0							(NVIC + 0x000)
#define NVIC_ISER1							(NVIC + 0x004)
#define NVIC_ISER2							(NVIC + 0x008)

#define NVIC_ICER0							(NVIC + 0x080)
#define NVIC_ICER1							(NVIC + 0x084)
#define NVIC_ICER2							(NVIC + 0x088)

#define NVIC_ISPR0							(NVIC + 0x100)
#define NVIC_ISPR1							(NVIC + 0x104)
#define NVIC_ISPR2							(NVIC + 0x108)

#define NVIC_ICPR0							(NVIC + 0x180)
#define NVIC_ICPR1							(NVIC + 0x184)
#define NVIC_ICPR2							(NVIC + 0x188)

#define NVIC_IABR0							(NVIC + 0x200)
#define NVIC_IABR1							(NVIC + 0x204)
#define NVIC_IABR2							(NVIC + 0x208)

#define NVIC_IPR0							(NVIC + 0x300)

typedef struct
{
	// pointer to hold the base address of GPIO port to which the pin belong peripheral
	GPIO_Reg_t* pGPIOx;

	uint8_t GPIO_PinNumber;				// Possible values from @GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode;				// Possible values from @GPIO_PIN_MODE
	uint8_t GPIO_PinPurpose;			// Possible values from @GPIO_PIN_PURPOSE
}GPIO_Handle_t;

// Peripheral clock setup
void GPIO_PeripheralClockControl(GPIO_Reg_t* pGPIOx, uint8_t EnableOrDisable);
void EXTI_PeripheralClockControl(void);

// Initialize and de-initialize
void GPIO_Init(GPIO_Handle_t* pGPIO_Handle);
void GPIO_DeInit(GPIO_Reg_t* pGPIOx);

// EXTI initialization
void EXTI_Init(uint8_t PortName, uint8_t pinNumber, uint8_t TriggerMode);

// Data read and write
uint8_t GPIO_ReadPin(GPIO_Reg_t* pGPIOx, uint8_t pinNumber);
void GPIO_OutputHigh(GPIO_Reg_t* pGPIOx, uint8_t pinNumber);
void GPIO_OutputLow(GPIO_Reg_t* pGPIOx, uint8_t pinNumber);
void GPIO_TogglePin(GPIO_Reg_t* pGPIOx, uint8_t pinNumber);


// IRQ Configuration and ISR handling
void GPIO_IRQConfigControl(uint8_t IRQn, uint8_t EnableOrDisable);
void GPIO_IRQConfigPriority(uint8_t IRQn, uint8_t Priority);
uint8_t GPIO_IRQCheckActive(uint8_t IRQn);
void GPIO_ClearEXTIPending(uint8_t pinNumber);

#endif /* INC_STM32F103C8T6_GPIO_H_ */
