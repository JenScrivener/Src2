/*
 * SPI.c
 *
 *  Created on: 16Mar.,2018
 *      Author: jens0
 */

#include "SPI.h"

/*
@brief:		Sends and recieves 8 bits on SPI2
@param:		Data - the data to be sent
@return:	The data that was recieved on SPI2
*/
uint8_t SPI_Send(uint8_t Data){

	SPI2->CR1|= 1000000;					// enable SPI
	SPI2->DR = Data; 						// write data to be transmitted to the SPI data register
	while( !(SPI2->SR & SPI_FLAG_TXE) ); 	// wait until transmit complete
	while( !(SPI2->SR & SPI_FLAG_RXNE) ); 	// wait until receive complete
	while( SPI2->SR & SPI_FLAG_BSY ); 		// wait until SPI is not busy anymore
	return(SPI2->DR);						// return data

}

/*
@brief:		Sends 8 bits on USART2
@param:		Data - the data to be sent
@return:	NA
*/
void serial(uint8_t Data){

	USART2->DR=Data;
	while( !(USART2->SR & UART_FLAG_TXE) ); 	// wait until transmit complete
}

/*
@brief:		Sends 'Len' bytes of 'Data'
@param:		Data - a pointer to the first byte of a string or array holding the data to be sent
			Len - the number of bytes to send
@return:	NA
*/
void burstSerial(char *Data, uint8_t Len){
	for(int x=0;x<Len;x++){

		serial(*Data);
		Data++;
	}
	serial(13);
}
