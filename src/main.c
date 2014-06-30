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

// this slave address belongs to the STM32F4-Discovery board's 
// CS43L22 Audio DAC
// connect PD4 to VDD in order to get the DAC out of reset and test the I2C
// interface
#define SLAVE_ADDRESS 0x4A // the slave address (example)

void I2C1_init(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	
	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* setup SCL and SDA pins
	 * You can connect the I2C1 functions to two different
	 * pins:
	 * 1. SCL on PB6 or PB8  
	 * 2. SDA on PB7 or PB9
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9; // we are going to use PB6 and PB9
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB
	
	// Connect I2C1 pins to AF  
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); // SDA
	
	// configure I2C1 
	I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1
	
	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

void I2S_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct, GPIOA_InitStruct;
    I2S_InitTypeDef I2S3_InitStruct;

    /* Enable APB1 peripheral clock for I2S3 (SPI3) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    /* Enable AHB1 peripheral clock for GPIOC */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    /* Enable AHB1 peripheral clock for GPIOA */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* Setup MCLK, SCLK and SDIN pins */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_12;
    /* set to use alternate function */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 
    /* Just chose the fastest speed */
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; 
    /* open/drain, line only has to be pulled low? using the same as for I2C*/
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    /* Enable pull up resistors, also don't know why, using the same as I2C */
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    /* initialize GPIOC */
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Setup LRCK/AINx pin */
    GPIOA_InitStruct.GPIO_Pin = GPIO_Pin_4;
    /* set to use alternate function */
    GPIOA_InitStruct.GPIO_Mode = GPIO_Mode_AF; 
    /* Just chose the fastest speed */
    GPIOA_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; 
    /* open/drain, line only has to be pulled low? using the same as for I2C*/
    GPIOA_InitStruct.GPIO_OType = GPIO_OType_OD;
    /* Enable pull up resistors, also don't know why, using the same as I2C */
    GPIOA_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    /* initialize GPIOC */
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* Connect I2S3 (SPI3) pins to GPIO's Alternate Function */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_SPI3); /* MCLK */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3); /* SCLK */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3); /* SDIN */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3); /* LRCK/AINx */

    /* Configure I2S3 (SPI3) */
    I2S3_InitStruct.I2S_Mode = I2S_Mode_MasterTx;
    I2S3_InitStruct.I2S_Standard = I2S_Standard_Phillips;
    I2S3_InitStruct.I2S_DataFormat = I2S_DataFormat_16b;
    I2S3_InitStruct.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
    I2S3_InitStruct.I2S_AudioFreq = I2S_AudioFreq_48k;
    I2S3_InitStruct.I2S_CPOL = I2S_CPOL_Low; /* I don't know, low I guess. */
    I2S_Init(SPI3, &I2S3_InitStruct);

    /* Enable transmit buffer empty interrupt */
    SPI_ITConfig(SPI3, SPI_I2S_IT_TXE, ENABLE);

    /* Enable transmitter interrupt in NVIC */
    NVIC_EnableIRQ(SPI3_IRQn);

}


/* This function issues a start condition and 
 * transmits the slave address + R/W bit
 * 
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the transmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy any more
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
  
	// Send I2C1 START condition 
	I2C_GenerateSTART(I2Cx, ENABLE);
	  
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
		
	// Send slave Address for write 
	I2C_Send7bitAddress(I2Cx, address, direction);
	  
	/* wait for I2Cx EV6, check if 
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */ 
	if(direction == I2C_Direction_Transmitter){
		while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)
                == ERROR);
	}
	else if(direction == I2C_Direction_Receiver){
		while(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)
                == ERROR);
	}
}

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1 
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	// wait for I2C1 EV8 --> last byte is still being transmitted (last byte in SR, buffer empty), next byte can already be written
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
	I2C_SendData(I2Cx, data);
}

/* This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the received data 
 * after that a STOP condition is transmitted
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	// disable acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
	
	// Send I2C1 STOP Condition after last byte has been transmitted
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/* CS43L22 needs reset pin high to work */
void CS43L22_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef GPIOD_InitStructure;
    GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIOD_InitStructure.GPIO_OType = GPIO_OType_PP; /* What happens with open/drain? */
    GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIOD_InitStructure);

    /* bring RESET low then high */
    GPIO_ResetBits(GPIOD, GPIO_Pin_4);
    GPIO_SetBits(GPIOD, GPIO_Pin_4);
}

/* configure LEDs to show some information */
void led_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef GPIOD_InitStructure;
    GPIOD_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIOD_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIOD_InitStructure.GPIO_OType = GPIO_OType_PP; /* What happens with open/drain? */
    GPIOD_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIOD_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIOD_InitStructure);
}
    
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

	I2C1_init(); // initialize I2C peripheral

    CS43L22_init(); // initialize dac periph

    led_init(); // init leds

	uint8_t received_data;
	
	I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, 0x01); // write one byte to the slave
	I2C_stop(I2C1); // stop the transmission
		
	I2C_start(I2C1, SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	received_data = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
	if(((received_data & (0x1f << 3)) >> 3) == (0x7 << 2)) {
        GPIO_SetBits(GPIOD, GPIO_Pin_12);
    } else {
        GPIO_SetBits(GPIOD, GPIO_Pin_13);
    }
	while(1);
	return 0;

//
//    /* Send some control data to make it beep (see Cirrus datasheet) */
//    
//    /* Generate start */
//    I2C_GenerateSTART(I2C1, ENABLE);
//
//    /* Write Chip Address, AD0 is low because it's connected to ground */
//    I2C_Send7bitAddress(I2C1, 0x94, I2C_Direction_Transmitter);
//
//    /* write address that we're starting to write at (beep freq/on-time), and
//     * that we're incrementing addresses */
//    I2C_SendData(I2C1, 0x1c | 0x80);
//
//    /* write beep freq 1000hz, on-time 5.2 sec */
//    I2C_SendData(I2C1, 0x7f);
//
//    /* keep beep volume and time the same */
//    I2C_SendData(I2C1, 0x00);
//
//    /* set beep occurence to continuous, disable mixing, leave eq same */
//    I2C_SendData(I2C1, 0xe0);
//
//    /* close communication */
//    I2C_GenerateSTOP(I2C1, ENABLE);
//
//    /* configure for I2S */
//    I2C_GenerateSTART(I2C1, ENABLE);
//    /* Write Chip Address, AD0 is low because it's connected to ground */
//    I2C_Send7bitAddress(I2C1, 0x94, I2C_Direction_Transmitter);
//    I2C_SendData(I2C1, 0x06);
//    I2C_SendData(I2C1, 0x07);
//    I2C_GenerateSTOP(I2C1, ENABLE);
//
//    /* configure headphones and speaker */
//    I2C_GenerateSTART(I2C1, ENABLE);
//    /* Write Chip Address, AD0 is low because it's connected to ground */
//    I2C_Send7bitAddress(I2C1, 0x94, I2C_Direction_Transmitter);
//    I2C_SendData(I2C1, 0x04);
//    I2C_SendData(I2C1, 0xA5);
//    I2C_GenerateSTOP(I2C1, ENABLE);
//
//
//    /* Enable I2S! */
//    SPI3->I2SCFGR |= SPI_I2SCFGR_I2SE;
//
//    /* start another communication */
//    I2C_GenerateSTART(I2C1, ENABLE);
//
//    /* Write Chip Address, AD0 is low because it's connected to ground */
//    I2C_Send7bitAddress(I2C1, 0x94, I2C_Direction_Transmitter);
//
//    /* We're just writing to one address */
//    I2C_SendData(I2C1, 0x02);
//    I2C_SendData(I2C1, 0x9e);
//    I2C_GenerateSTOP(I2C1, ENABLE);

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

int16_t simpleTone()
{
    static int16_t state = 1;
    state *= -1;
    return state;
}
    

/* Figures out why was called then remedies. So far probably just fills the
 * buffer with values to transmit */
void SPI3_IRQHandler(void)
{
    static double phaseL = 0, phaseR = 0;
    uint16_t data;
/*     NVIC_DisableIRQ(SPI3_IRQn); */

    /* Check that transmit buffer empty */
    if (SPI3->SR & (uint32_t)SPI_SR_TXE) {
        /* If so, fill with data */
        if (SPI3->SR & (uint32_t)SPI_SR_CHSIDE) {
            data = simpleTone();
            SPI3->DR = data;
            /*
            SPI3->DR = (uint16_t)(0xffff * sineTone(&phaseR,
                SINE_TONE_FREQ,
                SAMPLING_RATE));*/
        } else {
            data = simpleTone();
            SPI3->DR = data;
            /*
            SPI3->DR = (uint16_t)(0xffff * sineTone(&phaseL,
                SINE_TONE_FREQ,
                SAMPLING_RATE));
                */
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
//        SPI3->SR &= ~SPI_SR_TXE;
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
