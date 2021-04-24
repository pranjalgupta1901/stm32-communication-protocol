/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Pranjal Gupta
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* WHAT IS HAPPENING HERE IS DATA IS LOADED IN DR FROM TRANSMIT BUFFER AND THEN THROUGH
 * SHIFT REGISTER DATA IS TRANSMITTED TO DR AND THEN DATA IS LOADED INTO RX BUFFER
 * AND THEN WE READ IT
 */
#include <stdint.h>
#include <string.h>
#define SPI2_BASE_ADDR    0x40003800UL
#define GPIOB_BASE_ADDR   0x40020400UL
#define GPIOC_BASE_ADDR   0x40020800UL
#define RCC_BASE_ADDR     0x40023800UL
#define RCC_APB1ENR_OFFSET  0x40UL
#define RCC_APB1ENR_ADDR   (RCC_BASE_ADDR + RCC_APB1ENR_OFFSET)
#define RCC_AHB1ENR_OFFSET  0x30UL
#define RCC_AHB1ENR_ADDR   (RCC_BASE_ADDR + RCC_AHB1ENR_OFFSET)
#define delay   for(int i=0; i<500000;i++);


typedef struct
{
volatile uint32_t MODER;
volatile uint32_t OTYPER;
volatile uint32_t OSPEEDR;
volatile uint32_t PUPDR;
volatile uint32_t IDR;
volatile uint32_t ODR;
volatile uint32_t BSRR;
volatile uint32_t LCKR;
volatile uint32_t AFRL;
volatile uint32_t AFRH;
}GPIO_REGDEF_t;
 GPIO_REGDEF_t *pGPIOC = (GPIO_REGDEF_t*)GPIOC_BASE_ADDR;
 GPIO_REGDEF_t *pGPIOB = (GPIO_REGDEF_t*)GPIOB_BASE_ADDR;

 typedef struct
{
volatile uint32_t CR1;
volatile uint32_t CR2;
volatile uint32_t SR;
volatile uint32_t DR;
volatile uint32_t CRCPR;
volatile uint32_t RXCRCR;
volatile uint32_t TXCRCR;
volatile uint32_t I2SCFGR;
volatile uint32_t I2SPR;
}SPI_REGDEF_t;
SPI_REGDEF_t *pSPI2 = (SPI_REGDEF_t*)SPI2_BASE_ADDR;

void  SPI_SENDDATA(SPI_REGDEF_t *pSPI2, uint8_t *pTxBuffer, uint32_t Len);
void  SPI_RECIEVEDATA(SPI_REGDEF_t *pSPI2, uint8_t *pRxBuffer, uint32_t Len);

void SPI_SENDDATA(SPI_REGDEF_t *pSPI2, uint8_t *pTxBuffer, uint32_t Len)
{

	while(Len>0)
{  // wait until TXE is set

		while(!(pSPI2->SR & (1<<1)));

		pSPI2->DR=*((uint16_t*)pTxBuffer);

		Len--;
//Len--;

		(uint8_t*)pTxBuffer++;
    }
}


void SPI_RECIEVEDATA(SPI_REGDEF_t *pSPI2, uint8_t *pRxBuffer, uint32_t Len)
{
while(Len>0)
  {  // wait until RXNE is set

	while(!(pSPI2->SR & (1<<0)));

	*((uint32_t*)pRxBuffer)=pSPI2->DR;

	Len--;
//Len--;
        (uint8_t*)pRxBuffer++;
  }
}

int main(void)
{
	uint8_t ACKBYTE;
    char pin_num  = 9;
	char pin_state = 1;

    uint8_t arg[2];

    char data[] = "hello world";

    uint8_t datalen = strlen(data);
    uint8_t dummy_write=0xFF;
    uint8_t dummy_read, dummy_read1, dummy_read2;

    uint32_t *pAHB1ENR = (uint32_t*)RCC_AHB1ENR_ADDR;
    uint32_t *pAPB1ENR = (uint32_t*)RCC_APB1ENR_ADDR;

*pAHB1ENR|=(1<<2);   // GPIOC ENABLE
*pAHB1ENR|=(1<<1);   // GPIOB ENABLE
*pAPB1ENR|=(1<<14);  // SPI2 ENABLE


pGPIOB->MODER|=(1<<21)|(1<<31)|(1<<29)|(1<<19); // SCK ALTERNATE FUNCTION and MOSI
pGPIOB->AFRH|=(1<<30)|(1<<28)|(1<<26)|(1<<24)|(1<<6)|(1<<4); //AF5 AS PB15 MOSI AND MISO
pGPIOB->AFRH|=(1<<10)|(1<<8); //AF5 AS PB10 SCK
pGPIOC->PUPDR|=(1<<26);  //  PULL UP

     pSPI2->CR1|=(1<<2);                        //  Master configuration
     pSPI2->CR1|=(1<<3)|(1<<4);         // fPCLK/16    serial clock baud rate 1MHz
     pSPI2->CR1&=~(1<<0);                       // The first clock transition is the first data capture edge
     pSPI2->CR1&=~(1<<1);                       // CK to 0 when idle
     pSPI2->CR1&=~(1<<11);                      // 16-bit data frame format is selected for transmission/reception
     pSPI2->CR1&=~(1<<9);                       //  Software slave management disabled


while(1)
{

	if(!(pGPIOC->IDR & (1<<13)))
{
    delay;

    pSPI2->CR2|=(1<<2);                            // // SS output enable
    pSPI2->CR1|=(1<<6);                             //  Peripheral enabled

    uint8_t commandcode = 0x50;

    SPI_SENDDATA(pSPI2,&commandcode,1);

    SPI_RECIEVEDATA(pSPI2,&dummy_read,1);

	   SPI_SENDDATA(pSPI2,&dummy_write,1);

	   SPI_RECIEVEDATA(pSPI2,&ACKBYTE,1);



 if(ACKBYTE==0xF5)
    {
    	arg[0]=pin_num;
    	arg[1]=pin_state;
    	SPI_SENDDATA(pSPI2,arg,2);
    }


   // SPI_RECIEVEDATA(pSPI2,&dummy_read,1);
 SPI_SENDDATA(pSPI2,(uint8_t*)data,strlen(data));
 SPI_RECIEVEDATA(pSPI2,&dummy_read1,1);
 delay;
 SPI_SENDDATA(pSPI2,&datalen,1);
 SPI_RECIEVEDATA(pSPI2,&dummy_read2,1);




       while(pSPI2->SR & (1<<7));

       pSPI2->CR1&=~(1<<6);  //  Peripheral disabled

}


}


}

