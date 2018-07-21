/*
 * RFM95.c
 *
 *  Created on: 16Mar.,2018
 *      Author: jens0
 */

#include "RFM95.h"

int timer=0;
uint8_t ADDRESS=0x01;

void RFM95_Reg_Write(uint8_t Reg, uint8_t* Data, uint8_t Len){		//Write len bytes of data to reg

	Reg |= RFM95_WRITE;												//Set first bit to write

	CS(0);															//Pull chip select low to begin communication
	SPI_Send(Reg);													//Send the write command

	for(int x=0;x<Len;x++){
		SPI_Send(*Data);											//Write the xth byte of data
		Data++;
	}

	CS(1);															//Set chip select high to end communication

}

void RFM95_Reg_Read(uint8_t Reg, uint8_t* Data, uint8_t Len){		//Read len bytes of data from reg

	CS(0);															//Pull chip select low to begin communication
	SPI_Send(Reg);													//Send the read command

	for(int x=0;x<Len;x++){
		*Data=SPI_Send(Reg);										//Write the xth byte of data
		Data++;
	}

	CS(1);															//Set chip select high to end communication

}

void CS(int state){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, state);
}

void RFM95_Set_Mode(uint8_t Mode){									//Set Mode reg (0x01)

	if(Mode&~RFM95_MODE){											//If setting the whole OP Mode
		RFM95_Reg_Write(RFM95_REG_01_OP_MODE, &Mode, 1);
	}
	else{
		uint8_t mode=0;
		mode = RFM95_Get_Mode();									//setting just the mode (last three bits)
		mode&=~RFM95_MODE;
		mode|=Mode;
		RFM95_Reg_Write(RFM95_REG_01_OP_MODE, &mode, 1);
	}

}

uint8_t RFM95_Get_Mode(void){										//Read Mode reg (0x01)

	uint8_t mode=0;
	RFM95_Reg_Read(RFM95_REG_01_OP_MODE, &mode, 1);
	return(mode);

}

void RFM95_Set_Freq(double Freq){									//Set freq reg (0x06,07&08) to Freq (freq should be in MHz i.e 915)

	uint8_t *freq =(uint8_t*) malloc(3);
	int temp1 = (Freq*524288/XOSC);
	int temp2 = 0;

	freq=freq+3;
	for(int x=0;x<3;x++){
		freq--;

		temp2 = temp1;
		temp2 &= 0xFF;
		*freq = temp2;

		temp1=temp1>>8;
		temp2=temp2>>8;
	}

	RFM95_Reg_Write(RFM95_REG_06_FRF_MSB, freq, 3);
	free(freq);
}

double RFM95_Get_Freq(void){										//Get freq from reg(0x06,07,08) in MHz

	uint8_t freq[3];
	RFM95_Reg_Read(RFM95_REG_06_FRF_MSB, &freq[0], 3);

	int temp=0;
	double ret=0;

	temp|=freq[0];
	for(int x=1;x<3;x++){
		temp=temp<<8;
		temp|=freq[x];
	}

	ret=(double)temp/524288;
	ret=ret*XOSC;
	return(ret);
}

void RFM95_Set_Payload_Length(uint8_t PLL){

	RFM95_Reg_Write(RFM95_REG_22_PAYLOAD_LENGTH, &PLL, 1);
}

uint8_t RFM95_Get_Payload_Length(void){

	uint8_t PLL=0;
	RFM95_Reg_Read(RFM95_REG_22_PAYLOAD_LENGTH, &PLL, 1);
	return(PLL);
}

void RFM95_Set_Coding_Rate(uint8_t CR){

	uint8_t temp=0;
	RFM95_Reg_Read(RFM95_REG_1D_MODEM_CONFIG1, &temp, 1);			//current value of the registor

	CR=CR<<1;
	CR&=RFM95_CODING_RATE;
	temp&=!RFM95_CODING_RATE;										//clear the current CR
	CR|=temp;														//set the new CR
	RFM95_Reg_Write(RFM95_REG_1D_MODEM_CONFIG1, &CR, 1);			//write to the radio
}

uint8_t RFM95_Get_Coding_Rate(void){

	uint8_t CR=0;
	RFM95_Reg_Read(RFM95_REG_1D_MODEM_CONFIG1, &CR, 1);
	CR&=RFM95_CODING_RATE;
	CR=CR>>1;
	return(CR);
}

void RFM95_Set_Spreading_Factor(uint8_t SF){

	uint8_t temp=0;
	RFM95_Reg_Read(RFM95_REG_1E_MODEM_CONFIG2, &temp, 1);			//current value of the registor

	SF=SF<<4;
	SF&=RFM95_SPREADING_FACTOR;
	temp&=!RFM95_SPREADING_FACTOR;									//clear the current SF
	SF|=temp;														//set the new SF
	RFM95_Reg_Write(RFM95_REG_1E_MODEM_CONFIG2, &SF, 1);
}

uint8_t RFM95_Get_Spreading_Factor(void){

	uint8_t SF=0;
	RFM95_Reg_Read(RFM95_REG_1E_MODEM_CONFIG2, &SF, 1);
	SF&=RFM95_SPREADING_FACTOR;
	SF=SF>>4;
	return(SF);
}

void RFM95_Set_Bandwidth(uint8_t BW){

	uint8_t temp=0;
	RFM95_Reg_Read(RFM95_REG_1D_MODEM_CONFIG1, &temp, 1);			//current value of the registor

	BW=BW<<4;
	BW&=RFM95_BW;
	temp&=!RFM95_BW;												//clear the current BW
	BW|=temp;														//set the new BW
	RFM95_Reg_Write(RFM95_REG_1D_MODEM_CONFIG1, &BW, 1);
}

uint8_t RFM95_Get_Bandwidth(void){

	uint8_t BW=0;
	RFM95_Reg_Read(RFM95_REG_1D_MODEM_CONFIG1, &BW, 1);				//current value of the registor
	BW&=RFM95_BW;
	BW=BW>>4;
	return(BW);
}

void RFM95_Set_Output_Power(uint8_t OPP){

	uint8_t temp=0;
	RFM95_Reg_Read(RFM95_REG_09_PA_CONFIG, &temp, 1);				//current value of the registor

	OPP&=RFM95_OUTPUT_POWER;
	temp&=!RFM95_OUTPUT_POWER;										//clear the current OPP
	OPP|=temp;														//set the new OPP
	OPP|=RFM95_PA_SELECT;											//set PA_BOOST
	RFM95_Reg_Write(RFM95_REG_09_PA_CONFIG, &OPP, 1);
}

uint8_t RFM95_Get_Output_Power(void){

	uint8_t OPP=0;
	RFM95_Reg_Read(RFM95_REG_09_PA_CONFIG, &OPP, 1);				//current value of the registor
	OPP&=RFM95_OUTPUT_POWER;
	return(OPP);
}

void RFM95_Set_Hop_Period(uint8_t HP){

	RFM95_Reg_Write(RFM95_REG_24_HOP_PERIOD , &HP, 1);
}

uint8_t RFM95_Get_Hop_Period(void){

	uint8_t HP=0;
	RFM95_Reg_Read(RFM95_REG_24_HOP_PERIOD , &HP, 1);
	return(HP);
}

void RFM95_DIO_MapReg1(uint8_t DIO, uint8_t Map){
	uint8_t map =0;

	RFM95_Reg_Read(RFM95_REG_40_DIO_MAPPING1 , &map, 1);
	map=map&!DIO;
	Map=Map&0x03;
	DIO=DIO/3;
	while(DIO!=1){
		DIO=DIO>>2;
		Map=Map<<2;
	}
	Map=Map|map;
	RFM95_Reg_Write(RFM95_REG_40_DIO_MAPPING1 , &Map, 1);

}

void RFM95_DIO_MapReg2(uint8_t DIO, uint8_t Map){
	uint8_t map =0;

	RFM95_Reg_Read(RFM95_REG_41_DIO_MAPPING2 , &map, 1);
	map=map&!DIO;
	Map=Map&0x03;
	DIO=DIO/3;
	while(DIO!=1){
		DIO=DIO>>2;
		Map=Map<<2;
	}
	Map=Map|map;
	RFM95_Reg_Write(RFM95_REG_41_DIO_MAPPING2 , &Map, 1);

}

uint8_t RFM95_Get_DIO_MapReg1(uint8_t DIO){
	uint8_t map =0;

	RFM95_Reg_Read(RFM95_REG_40_DIO_MAPPING1 , &map, 1);
	map=map&DIO;
	DIO=DIO/3;
	while(DIO!=1){
		DIO=DIO>>2;
		map=map>>2;
	}
	return (map);

}

uint8_t RFM95_Get_DIO_MapReg2(uint8_t DIO){
	uint8_t map =0;

	RFM95_Reg_Read(RFM95_REG_41_DIO_MAPPING2 , &map, 1);
	map=map&DIO;
	DIO=DIO/3;
	while(DIO!=1){
		DIO=DIO>>2;
		map=map>>2;
	}
	return (map);
}

void RFM95_LoRa_Init(double Freq, uint8_t PayloadLength, uint8_t CodingRate, uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t OutputPower){

	uint8_t mode = RFM95_LONG_RANGE_MODE;
	mode|=RFM95_MODE_SLEEP;

	RFM95_Set_Mode(mode);											//enter LoRa mode, first to enter sleep mode
	RFM95_Set_Mode(mode);											//then to enter LoRa mode.

	RFM95_Set_Freq(Freq);
	RFM95_Set_Payload_Length(PayloadLength);
	RFM95_Reg_Write(RFM95_REG_13_RX_NB_BYTES, &PayloadLength, 1);
	RFM95_Set_Coding_Rate(CodingRate);
	RFM95_Set_Spreading_Factor(SpreadingFactor);
	RFM95_Set_Bandwidth(Bandwidth);
	RFM95_Set_Output_Power(OutputPower);

	RFM95_Reg_Read(RFM95_REG_1D_MODEM_CONFIG1, &mode, 1);			//Implicit header mode
	mode|=RFM95_IMPLICIT_HEADER_MODE_ON;
	RFM95_Reg_Write(RFM95_REG_1D_MODEM_CONFIG1, &mode, 1);

	RFM95_DIO_MapReg1(RFM95_DIO0,3);
//	RFM95_DIO_MapReg1(RFM95_DIO3,1);
	RFM95_Set_Hop_Period(3);

}

void Clear_Flags1(void){

	uint8_t IRQ_Flags=RFM95_FHSS_CHANGE_CHANNEL_MASK;				//clear flags on LoRa Radio
	RFM95_Reg_Write(RFM95_REG_12_IRQ_FLAGS , &IRQ_Flags, 1);
	RFM95_Reg_Write(RFM95_REG_12_IRQ_FLAGS , &IRQ_Flags, 1);

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);										//clear flags on STM

}

void Clear_Flags2(void){

	uint8_t IRQ_Flags=RFM95_TX_DONE_MASK | RFM95_RX_DONE_MASK | RFM95_VALID_HEADER_MASK;											//clear flags on LoRa Radio
	RFM95_Reg_Write(RFM95_REG_12_IRQ_FLAGS , &IRQ_Flags, 1);
	RFM95_Reg_Write(RFM95_REG_12_IRQ_FLAGS , &IRQ_Flags, 1);

	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);										//clear flags on STM
}

void Hop (void){
	uint8_t hop;
	double freq;

	RFM95_Reg_Read(RFM95_REG_1C_HOP_CHANNEL, &hop, 1);
	hop&=RFM95_FHSS_PRESENT_CHANNEL;
	hop=hop%20;

	if(hop>1){
		freq = (915+LIPD_BW/2)+((LIPD_BW+LIPD_Gap)*(hop));
	}
	else{
		freq=915.25;
	}

	RFM95_Set_Freq(freq);
}

void RFM95_LoRa_Test_Send(uint8_t *Data, uint8_t Len){

	RFM95_Set_Mode(RFM95_LONG_RANGE_MODE|RFM95_MODE_STDBY);			//Enter stand by mode
	RFM95_Set_Freq(915.25);
	uint8_t txbase = 0;												//Set FifoPtrAddr to FifoTxPtrBase
	RFM95_Reg_Read(RFM95_REG_0E_FIFO_TX_BASE_ADDR,&txbase,1);
	RFM95_Reg_Write(RFM95_REG_0D_FIFO_ADDR_PTR , &txbase, 1);

	RFM95_Reg_Write(RFM95_REG_22_PAYLOAD_LENGTH, &Len, 1);			//Set the payload length


	RFM95_Reg_Write(RFM95_REG_00_FIFO , Data, Len);					//Write data to FIFO

	RFM95_Set_Mode(RFM95_LONG_RANGE_MODE|RFM95_MODE_TX);			//Enter Transmit mode

	uint8_t txdone=0;
	while(!txdone){
		RFM95_Reg_Read(0x12, &txdone, 1);
	}

	Clear_Flags2();
}
