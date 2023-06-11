/*
 * absolute_encoder.c
 *
 *  Created on: 2023. 5. 3.
 *      Author: NamJunHee
 */

#include <stdio.h>
#include "main.h"
#include "absolute_encoder.h"

spiAbsoluteEncoder absoluteEn_1;
spiAbsoluteEncoder absoluteEn_2;
spiAbsoluteEncoder absoluteEn_3;

int encoder_header_test = 0;

void spiAbsoluteEncoderInitialize(spiAbsoluteEncoder *dst)
{
  dst->txData=nop_a5;
  __HAL_SPI_ENABLE(&hspi1);
  __HAL_SPI_ENABLE(&hspi2);
  __HAL_SPI_ENABLE(&hspi3);
  __HAL_SPI_ENABLE(&hspi4);
  __HAL_SPI_ENABLE(&hspi5);
  __HAL_SPI_ENABLE(&hspi6);
  __HAL_RCC_SPI1_CLK_ENABLE();
  __HAL_RCC_SPI2_CLK_ENABLE();
  __HAL_RCC_SPI3_CLK_ENABLE();
  __HAL_RCC_SPI4_CLK_ENABLE();
  __HAL_RCC_SPI5_CLK_ENABLE();
  __HAL_RCC_SPI6_CLK_ENABLE();
}

void getPosSPI1(spiAbsoluteEncoder *dst)  //Timer
{
  SPI1_OPEN;
  HAL_SPI_TransmitReceive(&hspi1,&(dst->txData),&(dst->rxBuffer),1,1); ////////////1바이트씩 주고 받음
  SPI1_CLOSE;
  //HAL_Delay(2);

  if(dst->rxBuffer==ready_op && dst->spiFlag==0)
  {
	  encoder_header_test++;
    dst->txData=rd_pos;
  }

  if(dst->rxBuffer==rd_pos)
  {
     dst->txData=nop_a5;
     dst->spiFlag=1;
  }

 if(dst->spiFlag==2)        //LSB
  {
    dst->spiFlag=0;

    dst->spiPosTemp+=dst->rxBuffer;

    if(dst->spiPosTemp<=4095)
      dst->spiPos=dst->spiPosTemp;
  }

  else if(dst->spiFlag==1&&dst->rxBuffer!=rd_pos) //MSB
  {
    dst->spiFlag=2;
    dst->spiPosTemp=0;
    dst->spiPosTemp+=dst->rxBuffer<<8;
  }


}
void getPosSPI2(spiAbsoluteEncoder *dst)  //Timer
{
  SPI2_OPEN;
  HAL_SPI_TransmitReceive(&hspi2,&(dst->txData),&(dst->rxBuffer),1,1); ////////////1바이트씩 주고 받음
  SPI2_CLOSE;
  //HAL_Delay(2);

  if(dst->rxBuffer==ready_op && dst->spiFlag==0)
  {
	  encoder_header_test++;
    dst->txData=rd_pos;
  }

  if(dst->rxBuffer==rd_pos)
  {
     dst->txData=nop_a5;
     dst->spiFlag=1;
  }

 if(dst->spiFlag==2)        //LSB
  {
    dst->spiFlag=0;

    dst->spiPosTemp+=dst->rxBuffer;

    if(dst->spiPosTemp<=4095)
      dst->spiPos=dst->spiPosTemp;
  }

  else if(dst->spiFlag==1&&dst->rxBuffer!=rd_pos) //MSB
  {
    dst->spiFlag=2;
    dst->spiPosTemp=0;
    dst->spiPosTemp+=dst->rxBuffer<<8;
  }


}
void getPosSPI3(spiAbsoluteEncoder *dst)  //Timer
{
  SPI3_OPEN;
  HAL_SPI_TransmitReceive(&hspi3,&(dst->txData),&(dst->rxBuffer),1,1); ////////////1바이트씩 주고 받음
  SPI3_CLOSE;
  //HAL_Delay(2);

  if(dst->rxBuffer==ready_op && dst->spiFlag==0)
  {
	  encoder_header_test++;
    dst->txData=rd_pos;
  }

  if(dst->rxBuffer==rd_pos)
  {
     dst->txData=nop_a5;
     dst->spiFlag=1;
  }

 if(dst->spiFlag==2)        //LSB
  {
    dst->spiFlag=0;

    dst->spiPosTemp+=dst->rxBuffer;

    if(dst->spiPosTemp<=4095)
      dst->spiPos=dst->spiPosTemp;
  }

  else if(dst->spiFlag==1&&dst->rxBuffer!=rd_pos) //MSB
  {
    dst->spiFlag=2;
    dst->spiPosTemp=0;
    dst->spiPosTemp+=dst->rxBuffer<<8;
  }


}
void getPosSPI4(spiAbsoluteEncoder *dst)  //Timer
{
  SPI4_OPEN;
  HAL_SPI_TransmitReceive(&hspi4,&(dst->txData),&(dst->rxBuffer),1,1); ////////////1바이트씩 주고 받음
  SPI4_CLOSE;
  //HAL_Delay(2);

  if(dst->rxBuffer==ready_op && dst->spiFlag==0)
  {
	  encoder_header_test++;
    dst->txData=rd_pos;
  }

  if(dst->rxBuffer==rd_pos)
  {
     dst->txData=nop_a5;
     dst->spiFlag=1;
  }

 if(dst->spiFlag==2)        //LSB
  {
    dst->spiFlag=0;

    dst->spiPosTemp+=dst->rxBuffer;

    if(dst->spiPosTemp<=4095)
      dst->spiPos=dst->spiPosTemp;
  }

  else if(dst->spiFlag==1&&dst->rxBuffer!=rd_pos) //MSB
  {
    dst->spiFlag=2;
    dst->spiPosTemp=0;
    dst->spiPosTemp+=dst->rxBuffer<<8;
  }


}
void getPosSPI5(spiAbsoluteEncoder *dst)  //Timer
{
  SPI5_OPEN;
  HAL_SPI_TransmitReceive(&hspi5,&(dst->txData),&(dst->rxBuffer),1,1); ////////////1바이트씩 주고 받음
  SPI5_CLOSE;
  //HAL_Delay(2);

  if(dst->rxBuffer==ready_op && dst->spiFlag==0)
  {
	  encoder_header_test++;
    dst->txData=rd_pos;
  }

  if(dst->rxBuffer==rd_pos)
  {
     dst->txData=nop_a5;
     dst->spiFlag=1;
  }

 if(dst->spiFlag==2)        //LSB
  {
    dst->spiFlag=0;

    dst->spiPosTemp+=dst->rxBuffer;

    if(dst->spiPosTemp<=4095)
      dst->spiPos=dst->spiPosTemp;
  }

  else if(dst->spiFlag==1&&dst->rxBuffer!=rd_pos) //MSB
  {
    dst->spiFlag=2;
    dst->spiPosTemp=0;
    dst->spiPosTemp+=dst->rxBuffer<<8;
  }


}
void getPosSPI6(spiAbsoluteEncoder *dst)  //Timer
{
  SPI6_OPEN;
  HAL_SPI_TransmitReceive(&hspi6,&(dst->txData),&(dst->rxBuffer),1,1); ////////////1바이트씩 주고 받음
  SPI6_CLOSE;
  //HAL_Delay(2);

  if(dst->rxBuffer==ready_op && dst->spiFlag==0)
  {
	  encoder_header_test++;
    dst->txData=rd_pos;
  }

  if(dst->rxBuffer==rd_pos)
  {
     dst->txData=nop_a5;
     dst->spiFlag=1;
  }

 if(dst->spiFlag==2)        //LSB
  {
    dst->spiFlag=0;

    dst->spiPosTemp+=dst->rxBuffer;

    if(dst->spiPosTemp<=4095)
      dst->spiPos=dst->spiPosTemp;
  }

  else if(dst->spiFlag==1&&dst->rxBuffer!=rd_pos) //MSB
  {
    dst->spiFlag=2;
    dst->spiPosTemp=0;
    dst->spiPosTemp+=dst->rxBuffer<<8;
  }


}

void SetInitialPosSPI(spiAbsoluteEncoder *dst)
{
  dst->txData = set_zero_point;

  while(dst->rxBuffer != 0x80)
  {
    dst->rxBuffer++;
    SPI1_OPEN;SPI2_OPEN;SPI3_OPEN;SPI4_OPEN;SPI5_OPEN;SPI6_OPEN;
    HAL_SPI_TransmitReceive(&hspi1,&(dst->txData),&(dst->rxBuffer),1,1);
    HAL_SPI_TransmitReceive(&hspi2,&(dst->txData),&(dst->rxBuffer),1,1);
    HAL_SPI_TransmitReceive(&hspi3,&(dst->txData),&(dst->rxBuffer),1,1);
    HAL_SPI_TransmitReceive(&hspi4,&(dst->txData),&(dst->rxBuffer),1,1);
    HAL_SPI_TransmitReceive(&hspi5,&(dst->txData),&(dst->rxBuffer),1,1);
    HAL_SPI_TransmitReceive(&hspi6,&(dst->txData),&(dst->rxBuffer),1,1);
    SPI1_CLOSE;SPI2_CLOSE;SPI3_CLOSE;SPI4_CLOSE;SPI5_CLOSE;SPI6_CLOSE;
    //tx_data=nop_a5;
    //HAL_Delay(1);
  }


  while(1) //Power Off
  {
    HAL_Delay(1000);
  }
}
