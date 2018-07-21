/*
 * SPI.c
 *
 *  Created on: 16Mar.,2018
 *      Author: jens0
 */

#include "SPI.h"

uint8_t SPI_Send(uint8_t Data){

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
	SPI2->CR1|= 1000000;					// enable SPI
	SPI2->DR = Data; 						// write data to be transmitted to the SPI data register
	while( !(SPI2->SR & SPI_FLAG_TXE) ); 	// wait until transmit complete
	while( !(SPI2->SR & SPI_FLAG_RXNE) ); 	// wait until receive complete
	while( SPI2->SR & SPI_FLAG_BSY ); 		// wait until SPI is not busy anymore
	return(SPI2->DR);						// return data

}

void serial(uint8_t Data){

	USART2->DR=Data;
	while( !(USART2->SR & UART_FLAG_TXE) ); 	// wait until transmit complete
}

void burstSerial(char *Data, uint8_t Len){
	for(int x=0;x<Len;x++){

		serial(*Data);
		Data++;
	}
	serial(13);
}
