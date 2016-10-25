/*
 * vrs_cv5.h
 *
 *  Created on: 18. 10. 2016
 *      Author: Lukáš
 */

#ifndef VRS_CV5_H_
#define VRS_CV5_H_

void Delay(uint32_t cycles);
void GPIO_Inicializacia();
void ADC_Inicializacia();
void USART_Inicializacia();
void SendString(char* text);

#endif /* VRS_CV5_H_ */
