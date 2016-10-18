/*
 * vrs_cv5.c
 *
 *  Created on: 18. 10. 2016
 *      Author: Lukáš
 */

#include <stddef.h>
#include "stm32l1xx.h"
#include "vrs_cv5.h"

GPIO_InitTypeDef gpioInit;
NVIC_InitTypeDef nvicInit;
ADC_InitTypeDef adcInit;
USART_InitTypeDef usartInit;
extern uint16_t ADC_Value;
extern uint8_t SendMode;

void Delay(uint32_t cycles)
{
	while(cycles--);
}

void GPIO_Inicializacia()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);

	//Nastavenie pinu pre ADC
	GPIO_StructInit(&gpioInit);
	gpioInit.GPIO_Mode=GPIO_Mode_AN;
	gpioInit.GPIO_Pin=GPIO_Pin_0;
	gpioInit.GPIO_Speed=GPIO_Speed_40MHz;
	GPIO_Init(GPIOA,&gpioInit);

	//Nastavenie pinu pre LED
	GPIO_StructInit(&gpioInit);
	gpioInit.GPIO_Mode=GPIO_Mode_OUT;
	gpioInit.GPIO_Pin=GPIO_Pin_5;
	GPIO_Init(GPIOA,&gpioInit);

	//Nastavenie pinu pre USART1
	GPIO_StructInit(&gpioInit);
	gpioInit.GPIO_Mode=GPIO_Mode_AF;
	gpioInit.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
	gpioInit.GPIO_Speed=GPIO_Speed_40MHz;
	GPIO_Init(GPIOA,&gpioInit);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

}

void ADC_Inicializacia()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);

	RCC_HSICmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

	ADC_StructInit(&adcInit);
	adcInit.ADC_Resolution = ADC_Resolution_12b;
	adcInit.ADC_ContinuousConvMode = ENABLE;
	adcInit.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	adcInit.ADC_DataAlign = ADC_DataAlign_Right;
	adcInit.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &adcInit);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_384Cycles);
	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
	ADC_ITConfig(ADC1,ADC_IT_OVR,ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	nvicInit.NVIC_IRQChannel=ADC1_IRQn;
	nvicInit.NVIC_IRQChannelCmd=ENABLE;
	nvicInit.NVIC_IRQChannelPreemptionPriority=0;
	nvicInit.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&nvicInit);


	ADC_Cmd(ADC1,ENABLE);
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_ADONS)==RESET);
	ADC_SoftwareStartConv(ADC1);

}

void USART_Inicializacia()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	USART_StructInit(&usartInit);
	usartInit.USART_BaudRate = 9600;
	usartInit.USART_WordLength = USART_WordLength_8b;
	usartInit.USART_StopBits = USART_StopBits_1;
	usartInit.USART_Parity = USART_Parity_No;
	usartInit.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2,&usartInit);
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);

	nvicInit.NVIC_IRQChannel=USART2_IRQn;
	nvicInit.NVIC_IRQChannelCmd=ENABLE;
	nvicInit.NVIC_IRQChannelPreemptionPriority=1;
	nvicInit.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&nvicInit);

	USART_Cmd(USART2,ENABLE);
}

void SendString(char *text)
{
	uint8_t i=0;
	while(text[i])
	{
		USART_SendData(USART2,text[i]);
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
		i++;
	}
	USART_SendData(USART2,'\n');
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
	USART_SendData(USART2,'\r');
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
}

void ADC1_IRQHandler()
{
	if(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC))
	{
		ADC_Value=ADC_GetConversionValue(ADC1); //Vyèítanie hodnoty po skonèení konverzie do premennej
	}

	if(ADC_GetFlagStatus(ADC1,ADC_FLAG_OVR))
	{
		ADC_ClearFlag(ADC1,ADC_FLAG_OVR); //Zmazanie Overrun Flagu
	}
}

void USART2_IRQHandler()
{
	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE))
	{
		if(USART_ReceiveData(USART2)=='m')
		{
			SendMode=!SendMode;
		}
	}
}
