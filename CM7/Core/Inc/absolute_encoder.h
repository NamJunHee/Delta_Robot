/*
 * motor_contorl.h
 *
 *  Created on: 2023. 5. 3.
 *      Author: NamJunHee
 */
#include <stdio.h>
//#ifndef INC_ABSOLUTE_ENCODER_H_
//#define INC_ABSOLUTE_ENCODER_H_

#define nop_a5 0x00   //no operation
#define rd_pos 0x10    // read position
#define set_zero_point 0x70  // zero set
#define ready_op 0xA5  // 대기

#define SPI1_OPEN   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,0)
#define SPI1_CLOSE  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,1)

#define SPI2_OPEN   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0)
#define SPI2_CLOSE  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,1)

#define SPI3_OPEN   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,0)
#define SPI3_CLOSE  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,1)

#define SPI4_OPEN   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,0)
#define SPI4_CLOSE  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,1)

#define SPI5_OPEN   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,0)
#define SPI5_CLOSE  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,1)

#define SPI6_OPEN   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,0)
#define SPI6_CLOSE  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,1)

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;
extern SPI_HandleTypeDef hspi6;

//int encoder_header_test;

typedef struct _spiAbsoluteEncoder
{
  uint8_t txData;
  uint8_t rxBuffer;

  int spiFlag;
  int spiPos;
  int spiPosTemp;

}spiAbsoluteEncoder;

extern spiAbsoluteEncoder absoluteEn_1;
extern spiAbsoluteEncoder absoluteEn_2;
extern spiAbsoluteEncoder absoluteEn_3;

void spiAbsoluteEncoderInitialize(spiAbsoluteEncoder *dst);

void getPosSPI1(spiAbsoluteEncoder *dst);  //Timer
void getPosSPI2(spiAbsoluteEncoder *dst);  //Timer
void getPosSPI3(spiAbsoluteEncoder *dst);  //Timer
void getPosSPI4(spiAbsoluteEncoder *dst);  //Timer
void getPosSPI5(spiAbsoluteEncoder *dst);  //Timer
void getPosSPI6(spiAbsoluteEncoder *dst);  //Timer

void SetInitialPosSPI(spiAbsoluteEncoder *dst);

