/*
 * SPI.h
 *
 *  Created on: 16Mar.,2018
 *      Author: jens0
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f4xx.h"
#include <string.h>

uint8_t SPI_Send(uint8_t Data);
void serial(uint8_t Data);
void burstSerial(char *Data, uint8_t Len);

#endif /* SPI_H_ */
