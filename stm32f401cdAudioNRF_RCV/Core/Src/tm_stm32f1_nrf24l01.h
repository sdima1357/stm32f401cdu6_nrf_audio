/**
 * @author  Tilen Majerle
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.com
 * @link    http://stm32f4-discovery.com/2014/06/library-17-nrf24l01-stm32f4xx/
 * @version v1.1.1
 * @ide     Keil uVision
 * @license GNU GPL v3
 * @brief   Library template 
 *	
@verbatim
   ----------------------------------------------------------------------
    Copyright (C) Tilen Majerle, 2015
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.
     
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
@endverbatim
 */
#ifndef TM_NRF24L01_H
#define TM_NRF24L01_H 111

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup TM_STM32F4xx_Libraries
 * @{
 */

/**
 * @defgroup TM_NRF24L01P
 * @brief    nRF24L01+ library for STM32F4xx devices - http://stm32f4-discovery.com/2014/06/library-17-nrf24l01-stm32f4xx/
 * @{
 *
 * This library allows you to work with nRF24L01+ modules.
 * 
 * You can send and receive data from nRF24L01+ modules.
 * 
 * \par Default pinout
 * 	
@verbatim
NRF24L01+	STM32F4xx	DESCRIPTION

Minor modified for audio project
view from up:

|----------------------------------|
|1   2		  ___			\	   |
|3   4		 |___|			 \	   |
|5   6		 ______			/	   |
|7   8		<______	>		\	   |
|----------------------------------|


1 GND			GND			Ground
2 VCC			3.3V		3.3V
3 CE			PA2			RF activated pin   			NRF_CE  in main.h
4 CSN			PA3			Chip select pin for SPI     NRF_CSN in main.h
5 SCK			PB3			SCK pin for SPI             SPI3_CLK
6 MOSI			PB5			MOSI pin for SPI			SPI3_MOSI
7 MISO			PB4			MISO pin for SPI			SPI3_MISO
8 IRQ			PA1			Interrupt pin 				NRF_IRQ in main.h Goes low when active.Allmost no used

@endverbatim 	
 *
 * IRQ pin is not used in this library, but it's functionality is enabled by this software.
 *
 * You can still set any pin on F4xx to be an external interrupt and handle interrupts from nRF24L01+ module.
 *
 * The easiest way to that is to use TM EXTI library and attach interrupt functionality to this pin
 * 
 * \par Custom pinout
 *
 * Add lines below in your defines.h file if you want to change default pinout:
 *
@verbatim
//Change SPI used. Refer to TM SPI library to check which pins are for SPI
#define NRF24L01_SPI				SPI3
#define NRF24L01_SPI_PINS			TM_SPI_PinsPack_2

//Change CSN pin. This is for SPI communication
#define NRF24L01_CSN_PORT			GPIOD
#define NRF24L01_CSN_PIN			GPIO_Pin_7

//Change CE pin. This pin is used to enable/disable transmitter/receiver functionality
#define NRF24L01_CE_PORT			GPIOD
#define NRF24L01_CE_PIN				GPIO_Pin_8
@endverbatim
 *
 * \par Changelog
 *
@verbatim
 Versio 1.1.1
  - June 21, 2015
  - Fixed buf with pin configuration
  
 Version 1.1
  - March 11, 2015
  - Added support for my new GPIO system

 Version 1.0.1
  - December 14, 2014
  - Activated all 3 interrupts in NRF24L01+ 
 
 Version 1.0
  - First release
@endverbatim
 *
 * \par Dependencies
 *
@verbatim
 - STM32F4xx
 - defines.h
 - TM SPI
 - TM GPIO
@endverbatim
 */
#include "main.h"
//~ #include "defines.h"
//~ #include "tm_stm32f4_spi.h"
//~ #include "tm_stm32f4_gpio.h"

/**
 * @defgroup TM_NRF24L01P_Macros
 * @brief    Library defines
 * @{
 */

/* Default SPI used */
#ifndef NRF24L01_SPI
#define NRF24L01_SPI				(&hspi3)
extern SPI_HandleTypeDef            hspi3;
//#define NRF24L01_SPI_PINS			TM_SPI_PinsPack_2
#endif

/* SPI chip enable pin */
#ifndef NRF24L01_CSN_PIN
#define NRF24L01_CSN_PORT			NRF_CSN_GPIO_Port
#define NRF24L01_CSN_PIN			NRF_CSN_Pin
#endif

/* Chip enable for transmitting */
#ifndef NRF24L01_CE_PIN
#define NRF24L01_CE_PORT			NRF_CE_GPIO_Port
#define NRF24L01_CE_PIN			    NRF_CE_Pin
#endif

#define  TM_GPIO_SetPinLow(Port,Pin)  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_RESET)
#define  TM_GPIO_SetPinHigh(Port,Pin)  HAL_GPIO_WritePin(Port, Pin, GPIO_PIN_SET)

/* Pins configuration */
//#define NRF24L01_CE_LOW				TM_GPIO_SetPinLow(NRF24L01_CE_PORT, NRF24L01_CE_PIN)
//#define NRF24L01_CE_HIGH			TM_GPIO_SetPinHigh(NRF24L01_CE_PORT, NRF24L01_CE_PIN)
#define NRF24L01_CE_LOW				NRF24L01_CE_PORT->BSRR =  NRF24L01_CE_PIN<<16u
#define NRF24L01_CE_HIGH			NRF24L01_CE_PORT->BSRR =  NRF24L01_CE_PIN

//#define NRF24L01_CSN_LOW			TM_GPIO_SetPinLow(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN)
#define NRF24L01_CSN_LOW            NRF24L01_CSN_PORT->BSRR = NRF24L01_CSN_PIN<<16u
#define NRF24L01_CSN_HIGH			NRF24L01_CSN_PORT->BSRR = NRF24L01_CSN_PIN
//TM_GPIO_SetPinHigh(NRF24L01_CSN_PORT, NRF24L01_CSN_PIN)


/**
 * @}
 */
 
/**
 * @defgroup TM_NRF24L01P_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  Transmission status enumeration
 */
typedef enum {
	TM_NRF24L01_Transmit_Status_Lost = 0x00,   /*!< Message is lost, reached maximum number of retransmissions */
	TM_NRF24L01_Transmit_Status_Ok = 0x01,     /*!< Message sent successfully */
	TM_NRF24L01_Transmit_Status_Sending = 0xFF /*!< Message is still sending */
} TM_NRF24L01_Transmit_Status_t;

/**
 * @brief  Data rate enumeration
 */
typedef enum {
	TM_NRF24L01_DataRate_2M,  /*!< Data rate set to 2Mbps */
	TM_NRF24L01_DataRate_1M,  /*!< Data rate set to 1Mbps */
	TM_NRF24L01_DataRate_250k /*!< Data rate set to 250kbps */
} TM_NRF24L01_DataRate_t;

/**
 * @brief  Output power enumeration
 */
typedef enum {
	TM_NRF24L01_OutputPower_M18dBm,	/*!< Output power set to -18dBm */
	TM_NRF24L01_OutputPower_M12dBm, /*!< Output power set to -12dBm */
	TM_NRF24L01_OutputPower_M6dBm,  /*!< Output power set to -6dBm */
	TM_NRF24L01_OutputPower_0dBm    /*!< Output power set to 0dBm */
} TM_NRF24L01_OutputPower_t;

/* Clear interrupt flags */
#define NRF24L01_CLEAR_INTERRUPTS   { TM_NRF24L01_WriteRegister(0x07, 0x70); }

/* Gets interrupt status from device */
#define NRF24L01_GET_INTERRUPTS     TM_NRF24L01_GetStatus()

/* Interrupt masks */
#define NRF24L01_IRQ_DATA_READY     0x40 /*!< Data ready for receive */
#define NRF24L01_IRQ_TRAN_OK        0x20 /*!< Transmission went OK */
#define NRF24L01_IRQ_MAX_RT         0x10 /*!< Max retransmissions reached, last transmission failed */

/**
 * @}
 */

/**
 * @defgroup TM_NRF24L01P_Functions
 * @brief    Library Functions
 *
 * Here are listed very basic functions to work with NRF modules
 *
 * @{
 */

/**
 * @brief  Initializes NRF24L01+ module
 * @param  channel: channel you will use for communication, from 0 to 125 eg. working frequency from 2.4 to 2.525 GHz
 * @param  payload_size: maximum data to be sent in one packet from one NRF to another.
 * @note   Maximal payload size is 32bytes
 * @retval 1
 */
uint8_t TM_NRF24L01_Init(uint8_t channel, uint8_t payload_size);

/**
 * @brief  Sets own address. This is used for settings own id when communication with other modules
 * @note   "Own" address of one device must be the same as "TX" address of other device (and vice versa),
 *         if you want to get successful communication
 * @param  *adr: Pointer to 5-bytes length array with address
 * @retval None
 */
void TM_NRF24L01_SetMyAddress(uint8_t* adr);

/**
 * @brief  Sets address you will communicate with
 * @note   "Own" address of one device must be the same as "TX" address of other device (and vice versa),
 *         if you want to get successful communication
 * @param  *adr: Pointer to 5-bytes length array with address
 * @retval None
 */
void TM_NRF24L01_SetTxAddress(uint8_t* adr);

/**
 * @brief  Gets number of retransmissions needed in last transmission
 * @param  None
 * @retval Number of retransmissions, between 0 and 15.
 */
uint8_t TM_NRF24L01_GetRetransmissionsCount(void);

/**
 * @brief  Sets NRF24L01+ to TX mode
 * @note   In this mode is NRF able to send data to another NRF module
 * @param  None
 * @retval None
 */
void TM_NRF24L01_PowerUpTx(void);

/**
 * @brief  Sets NRF24L01+ to RX mode
 * @note   In this mode is NRF able to receive data from another NRF module.
 *         This is default mode and should be used all the time, except when sending data
 * @param  None
 * @retval None
 */
void TM_NRF24L01_PowerUpRx(void);

/**
 * @brief  Sets NRF24L01+ to power down mode
 * @note   In power down mode, you are not able to transmit/receive data.
 *         You can wake up device using @ref TM_NRF24L01_PowerUpTx() or @ref TM_NRF24L01_PowerUpRx() functions
 * @param  None
 * @retval None
 */
void TM_NRF24L01_PowerDown(void);

/**
 * @brief  Gets transmissions status
 * @param  None
 * @retval Transmission status. Return is based on @ref TM_NRF24L01_Transmit_Status_t enumeration
 */
TM_NRF24L01_Transmit_Status_t TM_NRF24L01_GetTransmissionStatus(void);

/**
 * @brief  Transmits data with NRF24L01+ to another NRF module
 * @param  *data: Pointer to 8-bit array with data.
 *         Maximum length of array can be the same as "payload_size" parameter on initialization
 * @retval None
 */
void TM_NRF24L01_Transmit(uint8_t *data);
void TM_NRF24L01_Transmit_NoACK(uint8_t *data);

/**
 * @brief  Checks if data is ready to be read from NRF24L01+
 * @param  None
 * @retval Data ready status:
 *            - 0: No data available for receive in bufferReturns
 *            - > 0: Data is ready to be collected
 */
uint8_t TM_NRF24L01_DataReady(void);

/**
 * @brief  Gets data from NRF24L01+
 * @param  *data: Pointer to 8-bits array where data from NRF will be saved
 * @retval None
 */
void TM_NRF24L01_GetData(uint8_t *data);

/**
 * @brief  Sets working channel
 * @note   Channel value is just an offset in units MHz from 2.4GHz
 *         For example, if you select channel 65, then operation frequency will be set to 2.465GHz.
 * @param  channel: RF channel where device will operate
 * @retval None 
 */
void TM_NRF24L01_SetChannel(uint8_t channel);

/**
 * @brief  Sets RF parameters for NRF24L01+
 * @param  DataRate: Data rate selection for NRF module. This parameter can be a value of @ref TM_NRF24L01_DataRate_t enumeration
 * @param  OutPwr: Output power selection for NRF module. This parameter can be a value of @ref TM_NRF24L01_OutputPower_t enumeration
 * @retval None
 */
void TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_t DataRate, TM_NRF24L01_OutputPower_t OutPwr);

/**
 * @brief  Gets NRLF+ status register value
 * @param  None
 * @retval Status register from NRF
 */
uint8_t TM_NRF24L01_GetStatus(void);

/* Private */
void TM_NRF24L01_WriteRegister(uint8_t reg, uint8_t value);

uint8_t nRF24_Check(void);

/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif

