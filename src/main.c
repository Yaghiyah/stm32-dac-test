/* Get Cirrus DAC to beep using STM32F4 */

/* Includes ------------------------------------------------------------------*/
#include <math.h> 
/* This has to be included before core_cm4 because it tells it our interrupt
 * table */
#include "stm32f4xx.h"
#include <core_cm4.h> 
#include "stm32f4_discovery.h"
#include "stm32f4xx_conf.h" // again, added because ST didn't put it here ?

/* In combination with
 * PLLI2S_N  =  192
 * PLLI2S_R  =  5
 * these will achieve a sampling rate of 48000
 */
#define I2SDIV      12
#define I2SODD      1 
#define SAMPLING_RATE 48000 

/* Play sine tone at 440 Hz */
#define SINE_TONE_FREQ 440 


/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef    GPIO_InitStructure;
I2C_InitTypeDef     I2C_InitStruct;

void Delay(__IO uint32_t nCount);
double sineTone(double *phase, double freq, double sr);

/* Interrupt definition */
extern void SPI3_IRQHandler(void);
/* Another interrupt def */
extern void EXTI0_IRQHandler(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */

    /* more dumb interrupts */
    SYSCFG->EXTICR[0]   = 0x0000;
    EXTI->IMR          |= 0x00000001;
    EXTI->EMR          |= 0x00000001;
    EXTI->RTSR         |= 0x00000001;
    NVIC_EnableIRQ(EXTI0_IRQn);

    /* GPIOD Peripheral clock enable */
/*     RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

    /* I2C1 Peripheral clock enable and I2S3 Peripheral clock enable (SPI3) */
    RCC->APB1ENR |= RCC_APB1ENR_SPI3EN | RCC_APB1ENR_I2C1EN;

    /* Configure I2SDIV and ODD factor to get desired sampling rate */
    SPI3->I2SPR = (I2SODD << 8) | I2SDIV;
    
    /* Configure I2S instead of SPI, to master-transmit, to use Philips Standard, to
     * transmit data of 16-bit length, to expect 16-bit data (some are default
     * and therefore not set)*/
    SPI3->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_1;

    /* Enable SPI Transmit interrupt */
    SPI3->CR2 |= SPI_CR2_TXEIE;

    double somenumber = 0;
    somenumber = sin(M_PI / 2);


    /* Configure PD4 in output pushpull mode (reset pin of cirrus) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /* What happens with open/drain? */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    /* bring RESET high to enable chip */
    GPIO_SetBits(GPIOD, GPIO_Pin_4);

    /* Configure I2C1 */

    /* Initialize with defaults */    
    I2C_StructInit(&I2C_InitStruct);
    /* 50kHz */
    I2C_InitStruct.I2C_ClockSpeed     = 50000;
    /* I2C Mode */
    I2C_InitStruct.I2C_Mode           = I2C_Mode_I2C;

    /* Initialize I2C */
    I2C_Init(I2C1, &I2C_InitStruct);

    /* Enable I2C */
    I2C_Cmd(I2C1, ENABLE);

    /* Send some control data to make it beep (see Cirrus datasheet) */
    
    /* Generate start */
    I2C_GenerateSTART(I2C1, ENABLE);

    /* Write Chip Address, AD0 is low because it's connected to ground */
    I2C_Send7bitAddress(I2C1, 0x94, I2C_Direction_Transmitter);

    /* write address that we're starting to write at (beep freq/on-time), and
     * that we're incrementing addresses */
    I2C_SendData(I2C1, 0x1c | 0x80);

    /* write beep freq 1000hz, on-time 5.2 sec */
    I2C_SendData(I2C1, 0x7f);

    /* keep beep volume and time the same */
    I2C_SendData(I2C1, 0x00);

    /* set beep occurence to continuous, disable mixing, leave eq same */
    I2C_SendData(I2C1, 0xe0);

    /* close communication */
    I2C_GenerateSTOP(I2C1, ENABLE);

    /* configure for I2S */
    I2C_GenerateSTART(I2C1, ENABLE);
    I2C_SendData(I2C1, 0x06);
    I2C_SendData(I2C1, 0x07);
    I2C_GenerateSTOP(I2C1, ENABLE);

    /* configure headphones and speaker */
    I2C_GenerateSTART(I2C1, ENABLE);
    I2C_SendData(I2C1, 0x04);
    I2C_SendData(I2C1, 0xA5);
    I2C_GenerateSTOP(I2C1, ENABLE);

    /* Enable transmitter interrupt */
    NVIC_EnableIRQ(SPI3_IRQn);

    /* Enable I2S! */
    SPI3->I2SCFGR |= SPI_I2SCFGR_I2SE;

    /* start another communication */
    I2C_GenerateSTART(I2C1, ENABLE);

    /* Write Chip Address, AD0 is low because it's connected to ground */
    I2C_Send7bitAddress(I2C1, 0x94, I2C_Direction_Transmitter);

    /* We're just writing to one address */
    I2C_SendData(I2C1, 0x02);
    I2C_SendData(I2C1, 0x9e);
    I2C_GenerateSTOP(I2C1, ENABLE);

  while (1)
  {
    /* PD12 to be toggled */
    GPIO_SetBits(GPIOD, GPIO_Pin_13);
    
    /* Insert delay */
    Delay(0x3FFFFF);
    
    /* PD13 to be toggled */
    GPIO_SetBits(GPIOD, GPIO_Pin_12);
    
    /* Insert delay */
    Delay(0x7FFFFF);
  
    /* PD14 to be toggled */
    GPIO_SetBits(GPIOD, GPIO_Pin_14);
    
    /* Insert delay */
    Delay(0x3FFFFF);
    
    /* PD15 to be toggled */
    GPIO_SetBits(GPIOD, GPIO_Pin_15);
    
    /* Insert delay */
    Delay(0x7FFFFF);
    
    GPIO_ResetBits(GPIOD, GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
    
    /* Insert delay */
    Delay(0xFFFFFF);
  }
}

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

void EXTI0_IRQHandler(void)
{
    NVIC_ClearPendingIRQ(EXTI0_IRQn);
    NVIC_DisableIRQ(EXTI0_IRQn);
    GPIOD->ODR ^=  GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
    EXTI->PR = 0x00000001;
    NVIC_EnableIRQ(EXTI0_IRQn);
}

/* generates a sine tone at a specific frequency and sample rate */
double sineTone(double *phase, double freq, double sr)
{
    *phase += freq / sr;
    while (*phase > 1.) { *phase -= 1.; }
    return sin(2 * M_PI * (*phase));
}

/* Figures out why was called then remedies. So far probably just fills the
 * buffer with values to transmit */
void SPI3_IRQHandler(void)
{
    static double phaseL = 0, phaseR = 0;
/*     NVIC_DisableIRQ(SPI3_IRQn); */

    /* Check that transmit buffer empty */
    if (SPI3->SR & (uint32_t)SPI_SR_TXE) {
        /* If so, fill with data */
        if (SPI3->SR & (uint32_t)SPI_SR_CHSIDE) {
            SPI3->DR = (uint16_t)(0xffff * sineTone(&phaseR,
                SINE_TONE_FREQ,
                SAMPLING_RATE));
        } else {
            SPI3->DR = (uint16_t)(0xffff * sineTone(&phaseL,
                SINE_TONE_FREQ,
                SAMPLING_RATE));
        }
        /*
        SPI3->DR = (SPI3->SR & (uint32_t)SPI_SR_CHSIDE) ? 
            (uint16_t)(0xffff * sineTone(&phaseR,
                SINE_TONE_FREQ,
                SAMPLING_RATE)) :
            (uint16_t)(0xffff * sineTone(&phaseL,
                SINE_TONE_FREQ,
                SAMPLING_RATE));
        */
        /* set transmit buffer to not empty */
        SPI3->SR &= ~SPI_SR_TXE;
    } /* Otherwise do nothing for now */

/*     NVIC_EnableIRQ(SPI3_IRQn); */
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
