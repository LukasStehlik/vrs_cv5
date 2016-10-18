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
extern uint16_t ADC_Value;

void Delay(uint32_t cycles)
{
	while(cycles--);
}

void GPIO_Inicializacia()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);

	GPIO_StructInit(&gpioInit);
	gpioInit.GPIO_Mode=GPIO_Mode_AN;
	gpioInit.GPIO_Pin=GPIO_Pin_0;
	gpioInit.GPIO_Speed=GPIO_Speed_40MHz;
	GPIO_Init(GPIOA,&gpioInit);

	GPIO_StructInit(&gpioInit);
	gpioInit.GPIO_Mode=GPIO_Mode_OUT;
	gpioInit.GPIO_Pin=GPIO_Pin_5;
	GPIO_Init(GPIOA,&gpioInit);

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
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_16Cycles);
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
