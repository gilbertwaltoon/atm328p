/**
 * ADC Generated Driver API Header File
 * 
 * @file adc.h
 * 
 * @defgroup adc ADC
 * 
 * @brief This file contains the API prototypes and other data types for the Analog-to-Digital Converter (ADC) module.
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


#ifndef ADC_H_INCLUDED
#define ADC_H_INCLUDED

#include "../system/utils/compiler.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup adc
 * @typedef void adc_irq_cb_t
 * @brief Function pointer to the callback function called by the IRQ when operating in Normal mode. The default value is set to NULL, which means that no callback function will be used.
 */
typedef void (*adc_irq_cb_t)(void);

/**
 * @ingroup adc
 * @typedef uint16_t adc_result_t
 * @brief Data type for the result of the Analog-to-Digital Conversion (ADC).
 */
typedef uint16_t adc_result_t;

/**
 * @ingroup adc
 * @typedef uint8_t adc_0_channel_t
 * @brief Data type for the ADC Input Selection
 */
typedef uint8_t adc_0_channel_t;

/**
 * @ingroup adc
 * @brief Initializes the ADC interface. If the module is configured to the Disabled state, the clock is also disabled.
 * @param None.
 * @retval 0 - The ADC initialization is successful.
 * @retval 1 - The ADC initialization is not successful.
 */
int8_t ADC_Initialize(void);

/**
 * @ingroup adc
 * @brief Enables the ADC module and the clock system by setting the Enable bit in the ADC Control register.
 * @param None.
 * @return None.
 */
void ADC_Enable(void);

/**
 * @ingroup adc
 * @brief Disables the ADC module and the clock by clearing the Enable bit in the ADC Control register.
 * @param None.
 * @return None.
 */
void ADC_Disable(void);

/**
 * @ingroup adc
 * @brief Starts an Analog-to-Digital (A/D) conversion.
 * @param adc_0_channel_t channel - The ADC channel to start a conversion on.
 * @return None.
 */
void ADC_StartConversion(adc_0_channel_t channel);

/**
 * @ingroup adc
 * @brief Checks if the A/D conversion is done.
 * @param channel - None.
 * @retval true - The ADC conversion is done.
 * @retval false - The ADC converison is not done.
 */
bool ADC_IsConversionDone(void);

/**
 * @ingroup adc
 * @brief Reads the result of the A/D conversion.
 * @param None.
 * @return adc_result_t - The result of the conversion read from the  ADC module
 */
adc_result_t ADC_GetConversionResult(void);

/**
 * @ingroup adc
 * @brief Starts a conversion and returns the result.
 * @param adc_0_channel_t channel - The ADC channel to get the result of the conversion.
 * @return adc_result_t - The result of the conversion read from the ADC module.
 */
adc_result_t ADC_GetConversion(adc_0_channel_t channel);

/**
 * @ingroup adc
 * @brief Returns the number of bits from the result of the conversion.
 * @param None.
 * @return uint8_t - The number of bits in the result of the conversion.
 */
uint8_t ADC_GetResolution(void);

/**
 * @ingroup adc
 * @brief Disables Register 0 for ADC digital input.
 * @param uint8_t DisableValue - Value for the disabled digital input.
 * @return None.
 */
void ADC_DigitalInputDisable(uint8_t DisableValue);

/**
 * @ingroup adc
 * @brief Registers a callback function which is called if the conversion satisfies the window criteria.
 * @param adc_irq_cb_t callback - Pointer to the function callback.
 * @return None.
 */
void ADC_RegisterCallback(adc_irq_cb_t callback);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H_INCLUDED */