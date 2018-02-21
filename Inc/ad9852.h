/**
  ******************************************************************************
  * @file    ad9852.h
  * @brief   Header for ad9852.c file
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AD9852_H
#define __AD9852_H

/* Includes ------------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

// #################### AD9852_SPIx ##################################
#define AD9852_SPIx                               SPI3
#define AD9852_SPIx_CLK_ENABLE()                  __HAL_RCC_SPI3_CLK_ENABLE()

#define AD9852_SPIx_SCK_GPIO_PORT                 GPIOB             // PB.05
#define AD9852_SPIx_SCK_PIN                       GPIO_PIN_5
#define AD9852_SPIx_SCK_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define AD9852_SPIx_SCK_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE()

#define AD9852_SPIx_MISO_MOSI_GPIO_PORT           GPIOB
#define AD9852_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define AD9852_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()
#define AD9852_SPIx_MISO_PIN                      GPIO_PIN_4       /* PB.04*/
#define AD9852_SPIx_MOSI_PIN                      GPIO_PIN_3       /* PB.03*/


/*##################### AD9852 ###################################*/

/* Chip Select macro definition */
#define AD9852_CS_LOW()        HAL_GPIO_WritePin(AD9852_NCS_GPIO_PORT, AD9852_NCS_PIN, GPIO_PIN_RESET)
#define AD9852_CS_HIGH()       HAL_GPIO_WritePin(AD9852_NCS_GPIO_PORT, AD9852_NCS_PIN, GPIO_PIN_SET)
/* AD9852 OSK */
#define AD9852_OSK_LOW()       HAL_GPIO_WritePin(AD9852_OSK_GPIO_PORT, AD9852_OSK_PIN, GPIO_PIN_RESET)
#define AD9852_OSK_HIGH()      HAL_GPIO_WritePin(AD9852_OSK_GPIO_PORT, AD9852_OSK_PIN, GPIO_PIN_SET)
/* AD9852 RESET */
#define AD9852_RST_LOW()       HAL_GPIO_WritePin(AD9852_RST_GPIO_PORT, AD9852_RST_PIN, GPIO_PIN_RESET)
#define AD9852_RST_HIGH()      HAL_GPIO_WritePin(AD9852_RST_GPIO_PORT, AD9852_RST_PIN, GPIO_PIN_SET)
/* AD9852 MASTER RESET */
#define AD9852_MRST_LOW()      HAL_GPIO_WritePin(AD9852_MRST_GPIO_PORT, AD9852_MRST_PIN, GPIO_PIN_RESET)
#define AD9852_MRST_HIGH()     HAL_GPIO_WritePin(AD9852_MRST_GPIO_PORT, AD9852_MRST_PIN, GPIO_PIN_SET)
/* AD9852 UCLK */
#define AD9852_UCLK_LOW()      HAL_GPIO_WritePin(AD9852_UCLK_GPIO_PORT, AD9852_UCLK_PIN, GPIO_PIN_RESET)
#define AD9852_UCLK_HIGH()     HAL_GPIO_WritePin(AD9852_UCLK_GPIO_PORT, AD9852_UCLK_PIN, GPIO_PIN_SET)


/** 
  * @brief  AD9852 Control Interface pins 
  */ 

#define AD9852_NCS_PIN                             GPIO_PIN_0        /* PD.00 */
#define AD9852_NCS_GPIO_PORT                       GPIOD
#define AD9852_NCS_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()
#define AD9852_NCS_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOD_CLK_DISABLE()

#define AD9852_UCLK_PIN                            GPIO_PIN_2        /* PD.02*/
#define AD9852_UCLK_GPIO_PORT                      GPIOD
#define AD9852_UCLK_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOD_CLK_ENABLE()
#define AD9852_UCLK_GPIO_CLK_DISABLE()             __HAL_RCC_GPIOD_CLK_DISABLE()

#define AD9852_RST_PIN                             GPIO_PIN_4        /* PD.04*/
#define AD9852_RST_GPIO_PORT                       GPIOD
#define AD9852_RST_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()
#define AD9852_RST_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOD_CLK_DISABLE()

#define AD9852_MRST_PIN                            GPIO_PIN_5        /* PD.05*/
#define AD9852_MRST_GPIO_PORT                      GPIOD
#define AD9852_MRST_GPIO_CLK_ENABLE()              __HAL_RCC_GPIOD_CLK_ENABLE()
#define AD9852_MRST_GPIO_CLK_DISABLE()             __HAL_RCC_GPIOD_CLK_DISABLE()

#define AD9852_OSK_PIN                             GPIO_PIN_6        /* PD.06*/
#define AD9852_OSK_GPIO_PORT                       GPIOD
#define AD9852_OSK_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOD_CLK_ENABLE()
#define AD9852_OSK_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOD_CLK_DISABLE()


#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif


/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define AD9852_SPIx_TIMEOUT_MAX                   1000

#define BUFFERSIZE                             8


#define AD9852_REFCLK_FREQ              10000000l	// 10 Mhz from Trimble T-Bolt
#define Ad9852RefClkMult                20l		// AD9852 multiplier

enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};



/* Private function prototypes -----------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
void AD9852_IO_Init(void);
void AD9852_mDelay(uint32_t Delay);
void AD9852_uDelay(uint32_t Delay);
void AD9852_Error(void);

void AD9852_WriteByte(uint8_t ADdata);
uint8_t AD9852_ReadByte(void);
void AD9852_mrreset(void);
void AD9852_ioreset(void);
void AD9852_update(void);
void AD9852_select(void);
void AD9852_release(void);
uint32_t ad9852ReadControlReg(void);
void ad9852SetFreq1(uint32_t freqHz);   
void ad9852WriteFreq1(uint64_t ftw);
uint32_t ad9852ReadReg(uint8_t reg, uint8_t len);




#endif /* __AD9852_H */  


/******************* (C) COPYRIGHT 2016 Patoka Consulting Inc *****END OF FILE****/
