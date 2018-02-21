/**
  ******************************************************************************
  * @file    ad9852.c
  * @brief   This file provide functions to manage DDS AD9852
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "ad9852.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


/**
 * @brief BUS variables
 */
uint32_t SpixTimeout = AD9852_SPIx_TIMEOUT_MAX;        /*<! Value of Timeout when SPI communication fails */


/* Buffer used for transmission */
uint8_t aTxBuffer[BUFFERSIZE];

/* Buffer used for reception */
uint8_t aRxBuffer[BUFFERSIZE];


/* Private variables ---------------------------------------------------------*/

/* How to calculate desired frequency
   FTW = (Desired Output Frequency × 2^N)/SYSCLK,  where N = 48 bits, SYSCLK = (REF Clock * MULT) for AD9852
*/
//double FQ = 28147497.6710656;         // 10Mhz
double FQ = 1407374.88355328L;          // 10Mhz * MULT (20) = 200Mhz --> 2^48/200 = 1407374883553.28000 Hz

/* transfer state */
extern __IO uint32_t wTransferState;
extern __IO uint32_t rTransferState;

extern SPI_HandleTypeDef hspi3;
extern void DWT_Delay(uint32_t us);


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


/**
  * @brief  Configures the AD9852_SPI interface.
  * @retval None
  */
void AD9852_IO_Init(void)
{
    int8_t i, ret;
    uint8_t config[8];

    // HW Reset for AD9852
    AD9852_mrreset();

    // Set or Reset the control line 
    AD9852_CS_LOW();
    AD9852_CS_HIGH();

    hspi3.Instance = AD9852_SPIx;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;	// 21 Mhz
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi3.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi3);


    if(HAL_SPI_Init(&hspi3) != HAL_OK) {
        /* Initialization Error */
        AD9852_Error();
    }

    while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY);

    config[0] = 0x07; // The command to write to register
    config[1] = 0x10; // The most significant CR byte (0x10) [COMP PD]
    config[2] = 0x14; // The second CR byte (0x14) [NO PLL RANGE, NO PLL BYPASS, MULT=20 ]
    config[3] = 0x00; // The third CR byte (0x00) [MODE=0, EXTERNAL UPDATE]
    config[4] = 0x00; // The least significant CR byte (0x00) [ENABLE INV SYNC, NO OSK, NO MOSI]

    AD9852_ioreset();
    AD9852_select();
    AD9852_WriteByte(config[0]);   // Send Command
    AD9852_uDelay(10);      // Wait a little bit
    for(i=1;i<5;i++) {      // Send data
        AD9852_WriteByte(config[i]);
    }
    AD9852_WriteByte(0x00);
    AD9852_release();
    AD9852_update();

    AD9852_mDelay(100);

    ad9852ReadControlReg();   // Check the CTRL register setup

}


/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  * @retval None
  */
void AD9852_mDelay (uint32_t Delay)
{
  osDelay(Delay);
}

/**
  * @brief  Wait for loop in us.
  * @param  Delay in us.
  * @retval None
  */
void AD9852_uDelay (uint32_t Delay)
{
  DWT_Delay(Delay);
}


/**
  * @brief  Writes byte to the selected AD register.
  * @param  ADdata: data to write
  * @retval None
  */
void AD9852_WriteByte(uint8_t ADdata)
{
    // Deselect : Chip Select high 
    //AD9852_CS_LOW();

    wTransferState = TRANSFER_WAIT;
    HAL_SPI_Transmit_IT(&hspi3, &ADdata, 1);
    while (wTransferState == TRANSFER_WAIT);
    if(wTransferState != TRANSFER_COMPLETE) {
        AD9852_Error();
    }
    //AD9852_CS_HIGH();
}

/**
  * @brief  Writes word to the selected AD register.
  * @param  ADdata: data to write
  * @retval None
  */
void AD9852_WriteWord(uint16_t ADdata)
{
    // Deselect : Chip Select high 
    //AD9852_CS_LOW();

    aTxBuffer[0] = (uint8_t) (ADdata >> 8);     // HIGH Addr (MSB)
    aTxBuffer[1] = (uint8_t) (ADdata & 0x00FF); // LOW  Addr (LSB)

    wTransferState = TRANSFER_WAIT;
    HAL_SPI_Transmit_IT(&hspi3, (uint8_t*) aTxBuffer, 2);
    while (wTransferState == TRANSFER_WAIT);
    if(wTransferState != TRANSFER_COMPLETE) {
        AD9852_Error();
    }
    //AD9852_CS_HIGH();
}


/**
  * @brief  Read value from the AD9832
  * @param  LCDReg:      address of the selected register.
  * @retval None
  */
uint8_t AD9852_ReadByte(void)
{

    uint8_t ret;

    rTransferState = TRANSFER_WAIT;
    HAL_SPI_Receive_IT(&hspi3, &ret, 1);
    while (rTransferState == TRANSFER_WAIT);
    if(rTransferState != TRANSFER_COMPLETE) {
        AD9852_Error();
    }

    return(ret);
}



void AD9852_mrreset(void)
{
    AD9852_MRST_HIGH();
    AD9852_mDelay(100);
    AD9852_MRST_LOW();
    AD9852_mDelay(100);
}

void AD9852_update(void)
{
    AD9852_UCLK_HIGH();
    AD9852_uDelay(10);
    AD9852_UCLK_LOW();
    AD9852_uDelay(10);
}

void AD9852_ioreset(void)
{
    AD9852_RST_HIGH();
    AD9852_mDelay(1);
    AD9852_RST_LOW();
    AD9852_uDelay(10);
}

void AD9852_select(void)
{
    AD9852_CS_LOW();
    AD9852_uDelay(10);
}

void AD9852_release(void)
{
    AD9852_CS_HIGH();
}


uint32_t AD9852ReadReg(uint8_t reg, uint8_t len)   
{   
    uint8_t i;
    uint8_t fr[16];

    if(len > 15)
            return 0;
    
    AD9852_ioreset();
    AD9852_select();
    AD9852_WriteByte(reg);
    AD9852_update();

    for(i=0; i<len; i++) {
        fr[i] = AD9852_ReadByte();
    }
    AD9852_release();

    printf("# AD9852 Register [0x%X]:  ", reg);
    for(i=0; i<len; i++) {
            printf("0x%X:", fr[i]);
    }
    printf("\r\n");
}   

uint32_t ad9852ReadControlReg(void)   
{   
    uint8_t i;
    uint8_t ctrl[9];

    AD9852_ioreset();
    AD9852_select();
    AD9852_WriteByte(0x87);
    AD9852_update();

    for(i=0; i<4; i++) {
        ctrl[i] = AD9852_ReadByte();
    }
    AD9852_release();


    printf("# AD9852 CTRL Register:   ");
    for(i=0; i<4; i++) {
            printf("0x%X:", ctrl[i]);
    }
    printf("00\r\n");
}   

void ad9852WriteFreq1(uint64_t ftw)   
{   
    uint8_t i, rg;

    AD9852_ioreset();
    AD9852_select();
    AD9852_WriteByte(0x02);                     // Send Command
    AD9852_uDelay(10);                          // Wait a little bit
    for(i=40; i>0; i-=8) {                      // Send data
        rg = (uint8_t) (0xFF & (ftw >> i));
        AD9852_WriteByte(rg);
    }
    rg = (uint8_t) (0xFF & ftw);
    AD9852_WriteByte(rg);
    AD9852_WriteByte(0x00);                     // Finalise
    AD9852_release();
    AD9852_update();
}   

void AD9852SetFreq1(uint32_t freqHz)   
{   
    uint64_t ftw;  

    // Never go up to 80 mhz (Nyqist: 40% from Max Speed, which is 200 Mhz)
    if(freqHz > 80000000L)
        freqHz = 80000000L;
        
    // FTW = (Desired Output Frequency × 2^N)/SYSCLK   
    // where N = 48 bits   
       
    ftw = llround((double) (FQ *  freqHz)); // Calculate FTW: FTW = (FREQ * 2^48)/SysCLK and round it
    ad9852WriteFreq1(ftw);
}  


/**
  * @brief  SPIx error treatment function.
  * @param  None
  * @retval None
  */
void AD9852_Error(void)
{
    printf("# AD9258: SPI BUS ERROR\r\n");
    /* De-initialize the SPI communication BUS */
    HAL_SPI_DeInit(&hspi3);
  
    /* Re- Initialize the SPI communication BUS */
    AD9852_IO_Init();
}
