/*
 * stm32f103c8t6_GPIO.c
 * Author: Truong Trong Nhan
 */

#include "stm32f103c8t6_GPIO.h"

// Enable clock for GPIO
void GPIO_PeripheralClockControl(GPIO_Reg_t* pGPIOx, uint8_t EnableOrDisable)
{

	if(EnableOrDisable == ENABLE)				// If enable clock
	{
		if(pGPIOx == GPIOA)						// Port is chosen is GPIOA
		{
			//GPIOA_PERIPHERAL_CLOCK_ENABLE;		// Turn on the clock of port by point to RCC_APB2ENR
			RCC->RCC_APB2ENR |= RCC_APB2EN_GPIOA;
		}

		else if(pGPIOx == GPIOB)
		{
			RCC->RCC_APB2ENR |= RCC_APB2EN_GPIOB;
		}

		else if(pGPIOx == GPIOC)
		{
			RCC->RCC_APB2ENR |= RCC_APB2EN_GPIOC;
		}
	}
	else if(EnableOrDisable == DISABLE)
	{
		if(pGPIOx == GPIOA)
		{
			RCC->RCC_APB2ENR &= RCC_APB2DIS_GPIOA;
		}
		else if(pGPIOx == GPIOB)
		{
			RCC->RCC_APB2ENR &= RCC_APB2DIS_GPIOB;
		}

		else if(pGPIOx == GPIOC)
		{
			RCC->RCC_APB2ENR &= RCC_APB2DIS_GPIOC;
		}
	}
}

// Enable clock for AFIO
void EXTI_PeripheralClockControl(void)
{
	RCC->RCC_APB2ENR |= RCC_APB2EN_AFIO;
}

// Initialize GPIO and De-initialize GPIO
void GPIO_Init(GPIO_Handle_t* pGPIO_Handle)
{
	if(pGPIO_Handle->GPIO_PinNumber < 8)
	{
		// 1. Config mode and speed for GPIO pin
		pGPIO_Handle->pGPIOx->GPIOx_CRL &= ~(0x03 << (4 * pGPIO_Handle->GPIO_PinNumber)); // Clear bits
		pGPIO_Handle->pGPIOx->GPIOx_CRL |= (pGPIO_Handle->GPIO_PinMode << (4 * pGPIO_Handle->GPIO_PinNumber));

		// 2. Config purpose
		pGPIO_Handle->pGPIOx->GPIOx_CRL &= ~(0x03 <<  ((4 * pGPIO_Handle->GPIO_PinNumber) + 2));
		pGPIO_Handle->pGPIOx->GPIOx_CRL |= (pGPIO_Handle->GPIO_PinPurpose << ((4 * pGPIO_Handle->GPIO_PinNumber) + 2));
	}
	else if(pGPIO_Handle->GPIO_PinNumber >= 8)
	{
		// 1. Config mode and speed for GPIO pin
		pGPIO_Handle->pGPIOx->GPIOx_CRH &= ~(0x03 << (4 * (pGPIO_Handle->GPIO_PinNumber - 8))); // Clear bits
		pGPIO_Handle->pGPIOx->GPIOx_CRH |= (pGPIO_Handle->GPIO_PinMode << (4 * (pGPIO_Handle->GPIO_PinNumber - 8)));

		// 2. Config purpose
		pGPIO_Handle->pGPIOx->GPIOx_CRH &= ~(0x03 <<  ((4 * (pGPIO_Handle->GPIO_PinNumber - 8)) + 2));
		pGPIO_Handle->pGPIOx->GPIOx_CRH |= (pGPIO_Handle->GPIO_PinPurpose << ((4 * (pGPIO_Handle->GPIO_PinNumber - 8)) + 2));
	}
}

void GPIO_DeInit(GPIO_Reg_t* pGPIOx)
{
	if(pGPIOx == GPIOA)			// Port is chosen is GPIOA
	{
		// Reset the GPIO
		RCC->RCC_APB2RSTR |= (1<<2);		// Reset the GPIO
		RCC->RCC_APB2RSTR &= ~(1<<2);		// Clear bit
	}

	else if(pGPIOx == GPIOB)
	{
		RCC->RCC_APB2RSTR |= (1<<3);
		RCC->RCC_APB2RSTR &= ~(1<<3);
	}

	else if(pGPIOx == GPIOC)
	{
		RCC->RCC_APB2RSTR |= (1<<4);
		RCC->RCC_APB2RSTR &= ~(1<<4);
	}
}

// Initialize EXTI
void EXTI_Init(uint8_t PortName, uint8_t pinNumber, uint8_t TriggerMode)
{
	// Step 1: Config Input
	GPIO_Handle_t pEXTI;
	if(PortName == PortA)
	{
		pEXTI.pGPIOx = GPIOA;
	}
	else if(PortName == PortB)
	{
		pEXTI.pGPIOx = GPIOB;
	}
	else if(PortName == PortC)
	{
		pEXTI.pGPIOx = GPIOC;
	}
	pEXTI.GPIO_PinNumber = pinNumber;
	pEXTI.GPIO_PinMode = GPIO_INPUT_MODE;
	pEXTI.GPIO_PinPurpose = GPIO_INPUT_PULLUP_PULLDOWN_MODE;
	GPIO_PeripheralClockControl(pEXTI.pGPIOx, ENABLE);
	GPIO_Init(&pEXTI);

	// Step 2: Choose pin as EXTI
	if(pinNumber < 4)
	{
		AFIO->AFIO_EXTICR1 |= (PortName << (pinNumber * 4));
	}
	else if((pinNumber >= 4) && (pinNumber < 8))
	{
		AFIO->AFIO_EXTICR2 |= (PortName << ((pinNumber - 4) * 4));
	}
	else if((pinNumber >= 8) && (pinNumber < 12))
	{
		AFIO->AFIO_EXTICR3 |= (PortName << ((pinNumber - 8) * 4));
	}
	else if((pinNumber >= 12) && (pinNumber < 16))
	{
		AFIO->AFIO_EXTICR4 |= (PortName << ((pinNumber - 12) * 4));
	}

	// Step 3: Config mask bit
	EXTI->EXTI_IMR = (1 << pinNumber);

	// Step 4: Config Trigger
	if(TriggerMode == RISING_EDGE)
	{
		EXTI->EXTI_RTSR = (1 << pinNumber);
	}
	else if(TriggerMode == FALLING_EDGE)
	{
		EXTI->EXTI_FTSR = (1 << pinNumber);
	}
	else if(TriggerMode == RISING_FALLING_EDGE)
	{
		EXTI->EXTI_RTSR = (1 << pinNumber);
		EXTI->EXTI_FTSR = (1 << pinNumber);
	}
}

// GPIO function
uint8_t GPIO_ReadPin(GPIO_Reg_t* pGPIOx, uint8_t pinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->GPIOx_IDR >> pinNumber) & 0x01);
	return value;
}

void GPIO_OutputHigh(GPIO_Reg_t* pGPIOx, uint8_t pinNumber)
{
	pGPIOx->GPIOx_ODR |= (1 << pinNumber);
}

void GPIO_OutputLow(GPIO_Reg_t* pGPIOx, uint8_t pinNumber)
{
	pGPIOx->GPIOx_ODR &= ~(1 << pinNumber);
}

void GPIO_TogglePin(GPIO_Reg_t* pGPIOx, uint8_t pinNumber)
{
	if(((pGPIOx->GPIOx_ODR >> pinNumber) & 0x01) == 0x01)			GPIO_OutputLow(pGPIOx, pinNumber);
	else if(((pGPIOx->GPIOx_ODR >> pinNumber) & 0x01) == 0x00)		GPIO_OutputHigh(pGPIOx, pinNumber);
}

// EXTI function
void GPIO_IRQConfigControl(uint8_t IRQn, uint8_t EnableOrDisable)
{
	if(EnableOrDisable == ENABLE)
	{
		if(IRQn < 32)
		{
			NVIC_ISER0->NVIC_ISERx |= (1 << IRQn);
		}
		else if((IRQn >= 32) && (IRQn < 64))
		{
			NVIC_ISER1->NVIC_ISERx |=  (1 << (IRQn - 32));
		}
		else if((IRQn >= 64) && (IRQn < 67))
		{
			NVIC_ISER2->NVIC_ISERx |=  (1 << (IRQn - 64));
		}
	}
	else if(EnableOrDisable == DISABLE)
	{
		if(IRQn < 32)
		{
			NVIC_ICER0->NVIC_ICERx |= (1 << IRQn);
		}
		else if((IRQn >= 32) && (IRQn < 64))
		{
			NVIC_ICER1->NVIC_ICERx |=  (1 << (IRQn - 32));
		}
		else if((IRQn >= 64) && (IRQn < 67))
		{
			NVIC_ICER2->NVIC_ICERx |=  (1 << (IRQn - 64));
		}
	}
}

void GPIO_IRQConfigPriority(uint8_t IRQn, uint8_t Priority)
{
	uint8_t IPR_Number = IRQn / 4;
	uint8_t byte_offset = IRQn % 4;
	((NVIC_IPR0 + IPR_Number)->NVIC_IPRx) |= (Priority << (byte_offset * 4));
}

uint8_t GPIO_IRQCheckActive(uint8_t IRQn)
{
	uint8_t check = 0;
	if(IRQn < 32)
	{
		uint8_t i = 0;
		while((((1 << i) & (NVIC_IABR0->NVIC_IABRx)) - IRQn) != 0)
		{
			i++;
		}
		check = 1;
	}
	else if((IRQn >= 32) && (IRQn < 64))
	{
		uint8_t i = 0;
		while((((1 << i) & (NVIC_IABR1->NVIC_IABRx)) - (IRQn - 32)) != 0)
		{
			i++;
		}
		check = 1;
	}
	else if((IRQn >= 64) && (IRQn < 67))
	{
		uint8_t i = 0;
		while((((1 << i) & (NVIC_IABR2->NVIC_IABRx)) - (IRQn - 64)) != 0)
		{
			i++;
		}
		check = 1;
	}
	return check;
}

void GPIO_ClearEXTIPending(uint8_t pinNumber)
{
	if(((EXTI->EXTI_PR >> pinNumber) & 0x01) == 0x01)
	{
		EXTI->EXTI_PR |= (1 << pinNumber);
	}
}
