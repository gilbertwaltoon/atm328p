/**
 * USART0 Generated Driver API Header File
 * 
 * @file usart0.h
 * 
 * @defgroup usart0 USART0
 * 
 * @brief This file contains the API prototypes and other data types for the USART0 module.
 *
 * @version USART0 Driver Version 1.0.0
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

#ifndef USART0_H
#define USART0_H

/**
  Section: Included Files
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "../system/system.h"
#include "uart_drv_interface.h"

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

/* Normal Mode, Baud register value */
#define USART0_BAUD_RATE(BAUD_RATE) (((float)8000000 * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#define UART0_interface UART0


#define UART0_Initialize     USART0_Initialize
#define UART0_Deinitialize   USART0_Deinitialize
#define UART0_Write          USART0_Write
#define UART0_Read           USART0_Read
#define UART0__IsRxReady     USART0_IsRxReady
#define UART0_IsTxReady      USART0_IsTxReady
#define UART0_IsTxDone       USART0_IsTxDone

#define UART0_TransmitEnable       USART0_TransmitEnable
#define UART0_TransmitDisable      USART0_TransmitDisable
#define UART0_AutoBaudSet               (NULL)
#define UART0_AutoBaudQuery             (NULL)
#define UART0_BRGCountSet               (NULL)
#define UART0_BRGCountGet               (NULL)
#define UART0_BaudRateSet               (NULL)
#define UART0_BaudRateGet               (NULL)
#define UART0__AutoBaudEventEnableGet   (NULL)
#define UART0_ErrorGet             USART0_ErrorGet

#define UART0_TxCompleteCallbackRegister     (NULL)
#define UART0_RxCompleteCallbackRegister      (NULL)
#define UART0_TxCollisionCallbackRegister  (NULL)
#define UART0_FramingErrorCallbackRegister USART0_FramingErrorCallbackRegister
#define UART0_OverrunErrorCallbackRegister USART0_OverrunErrorCallbackRegister
#define UART0_ParityErrorCallbackRegister  USART0_ParityErrorCallbackRegister
#define UART0_EventCallbackRegister        (NULL)


/**
 @ingroup usart0
 @struct usart0_status_t
 @brief This is an instance of USART0_STATUS for USART0 module
 */
typedef union {
    struct {
        uint8_t perr : 1;     /**<This is a bit field for Parity Error status*/
        uint8_t ferr : 1;     /**<This is a bit field for Framing Error status*/
        uint8_t oerr : 1;     /**<This is a bit field for Overfrun Error status*/
        uint8_t reserved : 5; /**<Reserved*/
    };
    size_t status;            /**<Group byte for status errors*/
}usart0_status_t;



/**
 Section: Data Type Definitions
 */

/**
 * @ingroup usart0
 * @brief External object for usart0_interface.
 */
extern const uart_drv_interface_t UART0;

/**
 * @ingroup usart0
 * @brief Initializes the USART0 driver.
 *        This must be called only once, before any other routine is called.
 * @param None.
 * @return None.
 */
void USART0_Initialize(void);

/**
 * @ingroup usart0
 * @brief Deinitializes the USART0 driver.
 * @param None.
 * @return None.
 */
void USART0_Deinitialize(void);

/**
 * @ingroup usart0
 * @brief Enables the USART0 module.     
 * @param None.
 * @return None.
 */
void USART0_Enable(void);

/**
 * @ingroup usart0
 * @brief Disables the USART0 module.
 * @param None.
 * @return None.
 */
void USART0_Disable(void);

/**
 * @ingroup usart0
 * @brief Enables the USART0 transmitter.
 *        The driver sends bytes over to the TX pin.
 * @param None.
 * @return None.
 */
void USART0_TransmitEnable(void);

/**
 * @ingroup usart0
 * @brief Disables the USART0 transmitter.
 * @param None.
 * @return None.
 */
void USART0_TransmitDisable(void);

/**
 * @ingroup usart0
 * @brief Enables the USART0 receiver.
 *        The driver can receive bytes from the RX pin.
 * @param None.
 * @return None.
 */
void USART0_ReceiveEnable(void);

/**
 * @ingroup usart0
 * @brief Disables the USART0 Receiver.
 * @param None.
 * @return None.
 */
void USART0_ReceiveDisable(void);

/**
 * @ingroup usart0
 * @brief Gets data from the USART0.
 * @param None.
 * @return 8-bit data from the USART0 module.
 */
uint8_t USART0_GetData();



/**
 * @ingroup usart0
 * @brief Checks if the USART0 has received the data.
 * @param None.
 * @retval True - If the USART0 receiver has a data received.
 * @retval False - The USART0 FIFO receiver is empty.
 */
bool USART0_IsRxReady(void);

/**
 * @ingroup usart0
 * @brief Checks if the USART0 transmitter is ready to accept a data byte.
 * @param None.
 * @retval True - If the USART0 transmitter FIFO has at least one byte space.
 * @retval False - Is the USART0 transmitter FIFO is full.
 */
bool USART0_IsTxReady(void);

/**
 * @ingroup usart0
 * @brief Returns the status of the Transfer Shift Register (TSR).
 * @param None.
 * @retval True - If the data is completely shifted out from the TSR.
 * @retval False - If the data is present in the transmit FIFO and TSR.
 */
bool USART0_IsTxDone(void);

/**
 * @ingroup usart0
 * @brief Gets the error status of the last read byte. It is called before USART0_Read().
 * @param None.
 * @return Status of the last read byte. See usart0_status_t struct for more details.
 */
size_t USART0_ErrorGet(void);

/**
 * @ingroup usart0
 * @brief Reads the 8-bit value from the receiver First In First Out (FIFO) register.
 * @pre The transfer status must be checked to see if the receiver is not empty
 *      before calling this function. USART0_IsRxReady() must be checked in before calling this API.
 * @param None.
 * @return 8-bit data from the RX FIFO register.
 */
uint8_t USART0_Read(void);

/**
 * @ingroup usart0
 * @brief Writes a byte of data to the transmitter FIFO register.
 * @pre The transfer status must be checked to see if the transmitter is ready to accept a byte
 *      before calling this function. USART0_IsTxReady() must be checked in before calling this API.
 * @param txData  - Data byte to write to the TX FIFO.
 * @return None.
 */
void USART0_Write(uint8_t txData);

/**
 * @ingroup usart0
 * @brief Registers the function called upon USART0 framing error.
 * @param callbackHandler - A function pointer called upon the framing error condition.
 * @return None.
 */
void USART0_FramingErrorCallbackRegister(void (* callbackHandler)(void));

/**
 * @ingroup usart0
 * @brief Registers the function called upon USART0 overrun error.
 * @param callbackHandler - A function pointer called upon the overrun error condition.
 * @return None.
 */
void USART0_OverrunErrorCallbackRegister(void (* callbackHandler)(void));

/**
 * @ingroup usart0
 * @brief Registers the function called upon USART0 Parity error.
 * @param callbackHandler - A function pointer called upon the parity error condition.
 * @return None.
 */
void USART0_ParityErrorCallbackRegister(void (* callbackHandler)(void));


#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif  // USART0_H
