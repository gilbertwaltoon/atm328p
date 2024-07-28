/**
 * ADC Generated Driver File
 *
 * @file adc.c
 * 
 * @ingroup adc 
 * 
 * @brief This file contains the driver code for the ADC module.
 * 
 * @version ADC Driver Version 1.0.0
*/

/*
© [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

/**
  Section: Included Files
*/

#include <avr/io.h>
#include "../adc.h"


/** Function pointer to the callback function called by the IRQ when operating in Normal mode. The default value is set
    to NULL, which means that no callback function will be used.
*/
adc_irq_cb_t ADC_cb = NULL;

/**
 * \brief Initialize ADC interface
 */
int8_t ADC_Initialize(void) 
{
    //ADLAR disabled; MUX ADC5; REFS AVCC_WITH_EXTERNAL_CAPACITOR_AT_AREF_PIN; 
    ADMUX = 0x45;

    //ACME disabled; ADTS FREE_RUNNING_MODE; 
    ADCSRB = 0x0;

    //ADATE disabled; ADEN enabled; ADIE disabled; ADIF disabled; ADPS 64; ADSC disabled; 
    ADCSRA = 0x86;

    return 0;  
}

void ADC_Enable(void)
{
	ADCSRA |= (1 << ADEN);
}

void ADC_Disable(void)
{
	ADCSRA &= ~(1 << ADEN);
}

void ADC_StartConversion(adc_0_channel_t channel)
{
	ADMUX &= ~0x0f;
	ADMUX |= channel;
	ADCSRA |= (1 << ADSC);
}

bool ADC_IsConversionDone(void)
{
	return ((ADCSRA & (1 << ADIF)));
}

adc_result_t ADC_GetConversionResult(void)
{
	return (ADCL | ADCH << 8);
}

adc_result_t ADC_GetConversion(adc_0_channel_t channel)
{
	adc_result_t res;

	ADC_StartConversion(channel);
	while (!ADC_IsConversionDone());
	res = ADC_GetConversionResult();
	ADCSRA |= (1 << ADIF);
	return res;
}

uint8_t ADC_GetResolution(void)
{
	return 10;
}

void ADC_DigitalInputDisable(uint8_t DisableValue)
{
  DIDR0 = DisableValue;
}

void ADC_RegisterCallback(adc_irq_cb_t callback)
{
	ADC_cb = callback;
}

