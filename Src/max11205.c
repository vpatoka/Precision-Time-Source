/**
  ******************************************************************************
  * @file    max11205.c
  * @brief   This file provide functions to manage ADC MAX11205
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


// #################### max11205_SPIx ##################################
#define MAX11205_SPIx_SCK_GPIO_PORT                 GPIOD             // PD.3
#define MAX11205_SPIx_SCK_PIN                       GPIO_PIN_3
#define MAX11205_CLK_ON()         HAL_GPIO_WritePin(MAX11205_SPIx_SCK_GPIO_PORT, MAX11205_SPIx_SCK_PIN, GPIO_PIN_SET)
#define MAX11205_CLK_OFF()        HAL_GPIO_WritePin(MAX11205_SPIx_SCK_GPIO_PORT, MAX11205_SPIx_SCK_PIN, GPIO_PIN_RESET)
#define MAX11205_CLK_TGL()        HAL_GPIO_TogglePin(MAX11205_SPIx_SCK_GPIO_PORT, MAX11205_SPIx_SCK_PIN)

#define MAX11205_SPIx_MISO_GPIO_PORT                 GPIOD            // PD.01
#define MAX11205_SPIx_MISO_PIN                       GPIO_PIN_1
#define MAX11205_MISO_ON()         HAL_GPIO_WritePin(MAX11205_SPIx_MISO_GPIO_PORT, MAX11205_SPIx_MISO_PIN, GPIO_PIN_SET)
#define MAX11205_MISO_OFF()        HAL_GPIO_WritePin(MAX11205_SPIx_MISO_GPIO_PORT, MAX11205_SPIx_MISO_PIN, GPIO_PIN_RESET)
#define MAX11205_MISO_TGL()        HAL_GPIO_TogglePin(MAX11205_SPIx_MISO_GPIO_PORT, MAX11205_SPIx_MISO_PIN)
#define MAX11205_MISO_READ()	   HAL_GPIO_ReadPin(MAX11205_SPIx_MISO_GPIO_PORT, MAX11205_SPIx_MISO_PIN)

#define MAX11205CLK		50	// 10ms / 100 cycles

//double MAX11205COEF	=	.0000274658203125;
#define MAX11205	18000000L	// 1.8V * 10^7



/*##################### max11205 ###################################*/


/* Private variables ---------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/
void max11205_Delay (uint32_t Delay);
uint32_t max_MAX11205_GPIO_SPI_read(uint8_t uchUseCalibrationMode);
void max11205_tick (void);


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  * @retval None
  */
void max11205_Delay (uint32_t Delay)
{
   uint32_t i=0;
   uint32_t a=0;

   for(i=0;i<Delay;i++) a=i;
}


uint32_t max_MAX11205_GPIO_SPI_read(uint8_t uchUseCalibrationMode)
/**
* \brief       Performs a SPI read using bit banging. Written specifically for the MAX11205 ADC.
*
* \par         Description
*              The MAX11205 sets the RDY# line high during sampling, then low when a sample is available.
*              The function waits for that transition, then provides the appropriate number of clocks
*              to the MAX11205 to complete the read.  Note that the 11205 is a 16 bit device, but requires
*              9 additional clocks (25 in total) to complete a read cycle
*              An optional self-calibration mode can be selected by setting uchUseCalibrationMode==TRUE
*              which will send a 26th clock to the MAX11205.
*
* The RDY/DOUT is used to signal data ready, as well as
* reading the data out when SCLK pulses are applied.
* RDY/DOUT is high by default. The MAX11205 pulls
* RDY/DOUT low when data is available at the end of conversion,
* and stays low until clock pulses are applied at
* SCLK input; on applying the clock pulses at SCLK, the
* RDY/DOUT outputs the conversion data on every SCLK
* positive edge. To monitor data availability, pull RDY/
* DOUT high after reading the 16 bits of data by supplying
* a 25th SCLK pulse.
*
* \param[in]   uchUseCalibrationMode       - TRUE = perform function in self-calibration mode. 
*
* \retval      adcValue                    - 16-bit value from ADC conversion
*/
{
	int32_t i=0;
	uint32_t adcValue=0;

	// - Enter to Sleep mode
	// Turn CLK HIGH
	//MAX11205_CLK_ON();
	// Wait until RDY# goes high (keep looping while low)
	// HIGH is default status for MAX11205 MISO
	//while(MAX11205_MISO_READ() == GPIO_PIN_RESET);

	// Exit from Sleep Mode
	MAX11205_CLK_OFF();

	// Now, wait until RDY# goes low (keep looping while high)
	// Check if there is any data. If its LOW  (RESET/ZERO) --> data is available
	while(MAX11205_MISO_READ() == GPIO_PIN_SET);

	// Now, send in clocks.
	for(i=0;i<16;i++) {
		MAX11205_CLK_ON();
//		if(MAX11205_MISO_READ() == GPIO_PIN_SET) {
//			adcValue |= 0x00000001;
//		}
		max11205_Delay(MAX11205CLK);
		MAX11205_CLK_OFF();
		if(MAX11205_MISO_READ() == GPIO_PIN_SET) {
			adcValue |= 0x00000001;
		}
		adcValue <<=1;
		max11205_Delay(MAX11205CLK);
	}

	// The 11205 device requires 25 clocks total to complete a read.
	// We have already sent (16) clocks, so send 9 more clocks.
	// (If self-calibration mode set then we send 10 more clocks.)
	if(!uchUseCalibrationMode) {
		for(i=0; i<7;i++) 
			max11205_tick();
	} else {
		for(i=0; i<9;i++) 
			max11205_tick();
	}
	max11205_Delay(MAX11205CLK);
	MAX11205_CLK_ON();

	return(adcValue);
}


/**
  * @brief  Clock Pulse
  * @param  None
  * @retval None
  */
void max11205_tick(void)
{
	MAX11205_CLK_ON();
	max11205_Delay(MAX11205CLK);
	MAX11205_CLK_OFF();
	max11205_Delay(MAX11205CLK);
}

