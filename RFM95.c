/*
 * RFM95.c
 *
 *  Created on: 16Mar.,2018
 *      Author: jens0
 */

#include "RFM95.h"

int GOOD=0;
int BAD=0;

/*
 * Because a layer two transmision requires multiple Lora packets it can
 * be tricky to keep track of what has been sent, also between transmissions
 * the processor returns to the main loop to chill out. Thus each node has
 * three global parameters for handling the layer two transmissions.
 * When you want to send something reliably using the layer two protocol these
 * parameters will be defined for you. 'DATA' is a pointer to the data to be
 * transitted after the layer two header has been added. 'PACKETS' is how many
 * packets are required to send the whole message. 'LEN' is merely 'PACKETS'
 * times the number of bytes per packet. Because of the way I wrote the code
 * there is also a struct called "L2HEADER" which contains more infomation about
 * These three parameters should probably be put in that struct but if this
 * coment is still here it means I never bothered.
 */
uint8_t *DATA;
uint8_t PACKETS=0;
uint8_t LEN=0;

/*
@brief:		Writes 'Len' bytes of 'Data' to the register 'Reg' of the radio
@param: 	Reg 	- The address of the register to write to (see 'Register addresses from table 85 Semtech' RFM95.h for full list of registers)
			Data	- A pointer to the first byte of data to be sent (could be a string or an array)
			Len 	- How many bytes of data to send (for a string can use strlen() to get the length)
@return:	NA
*/
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

/*
@brief:		Reads 'Len' bytes of data from register 'Reg' of the radio and stores it at the address pointed to by 'Data'
@param: 	Reg 	- The address of the register to read from (see 'Register addresses from table 85 Semtech' RFM95.h for full list of registers)
			Data	- A pointer to store data in (this function doesn't return anything so the pointer is how the data gets out of the function)
			Len 	- How many bytes to read (this is mainly used when reading the RFM95W FiFo when data has been received otherwise it is usually 1)
@return:	NA
*/
void RFM95_Reg_Read(uint8_t Reg, uint8_t* Data, uint8_t Len){		//Read len bytes of data from reg

	CS(0);															//Pull chip select low to begin communication
	SPI_Send(Reg);													//Send the read command

	for(int x=0;x<Len;x++){
		*Data=SPI_Send(Reg);										//Write the xth byte of data
		Data++;
	}

	CS(1);															//Set chip select high to end communication

}

/*
@brief:		Pulls the chip select pin high or low for SPI transmissions
@param: 	state - 0 for low and 1 for high
@return:	NA
*/

void CS(int state){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, state);
}

/*
@brief:		Sets the mode of the radio
@param: 	Mode - used to specify which mode to enter (see 'RFM95_REG_01_OP_MODE ' in RFM95.h for a full list of modes)
@return:	NA
*/
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

/*
@brief:		Returns the current mode of the radio
@param: 	NA
@return:	The current value of the mode register of the RFM95W LoRa radio
*/
uint8_t RFM95_Get_Mode(void){										//Read Mode reg (0x01)

	uint8_t mode=0;
	RFM95_Reg_Read(RFM95_REG_01_OP_MODE, &mode, 1);
	return(mode);

}

/*
@brief:		Sets the frequency of the radio
@param: 	Freq - The desired frequency of the radio in MHz (the Australian ISM band I used was 915-928MHz)
@return:	NA
*/
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

/*
@brief:		Returns the current frequency of the radio in MHz
@param: 	NA
@return:	The current frequency of the radio in MHz (this is splint into 3 registers to allow for fine tuning of the frequency so requires some decoding)
*/
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

/*
@brief:		Sets the number of bytes in a radio transmission (packet)
@param: 	PLL - the desired pay load length in bytes (between 1-255)
@return:	NA
*/
void RFM95_Set_Payload_Length(uint8_t PLL){

	RFM95_Reg_Write(RFM95_REG_22_PAYLOAD_LENGTH, &PLL, 1);
}

/*
@brief:		Gets the number of bytes in a radio transmission (packet)
@param: 	NA
@return:	The value of the pay load length register
*/
uint8_t RFM95_Get_Payload_Length(void){

	uint8_t PLL=0;
	RFM95_Reg_Read(RFM95_REG_22_PAYLOAD_LENGTH, &PLL, 1);
	return(PLL);
}

/*
@brief:		Sets the coding rate of the radio transmission
@param: 	CR - the desired coding rate (see 'RFM95_CODING_RATE' in RF95.h for a list of avaliable coding rates)
@return:	NA
*/
void RFM95_Set_Coding_Rate(uint8_t CR){

	uint8_t temp=0;
	RFM95_Reg_Read(RFM95_REG_1D_MODEM_CONFIG1, &temp, 1);			//current value of the registor

	CR=CR<<1;
	CR&=RFM95_CODING_RATE;
	temp&=!RFM95_CODING_RATE;										//clear the current CR
	CR|=temp;														//set the new CR
	RFM95_Reg_Write(RFM95_REG_1D_MODEM_CONFIG1, &CR, 1);			//write to the radio
}

/*
@brief:		Gets the current coding rate of the radio transmission
@param: 	NA
@return:	The values of the coding rate bits in configuration register one
*/
uint8_t RFM95_Get_Coding_Rate(void){

	uint8_t CR=0;
	RFM95_Reg_Read(RFM95_REG_1D_MODEM_CONFIG1, &CR, 1);
	CR&=RFM95_CODING_RATE;
	CR=CR>>1;
	return(CR);
}

/*
@brief:		Sets the spreding factor of the radio transmission
@param: 	Sf - the desired spreading factor (see 'RFM95_SPREADING_FACTOR' in RF95.h for a list of avaliable spreading factors)
@return:	NA
*/
void RFM95_Set_Spreading_Factor(uint8_t SF){

	uint8_t temp=0;
	RFM95_Reg_Read(RFM95_REG_1E_MODEM_CONFIG2, &temp, 1);			//current value of the registor

	SF=SF<<4;
	SF&=RFM95_SPREADING_FACTOR;
	temp&=!RFM95_SPREADING_FACTOR;									//clear the current SF
	SF|=temp;														//set the new SF
	RFM95_Reg_Write(RFM95_REG_1E_MODEM_CONFIG2, &SF, 1);
}

/*
@brief:		Gets the spreding factor of the radio transmission
@param: 	NA
@return:	The spreading factor bits in configuration registor two
*/
uint8_t RFM95_Get_Spreading_Factor(void){

	uint8_t SF=0;
	RFM95_Reg_Read(RFM95_REG_1E_MODEM_CONFIG2, &SF, 1);
	SF&=RFM95_SPREADING_FACTOR;
	SF=SF>>4;
	return(SF);
}

/*
@brief:		Sets the bandwidth of the radio transmission
@param: 	BW - the desired bandwidth (see 'RFM95_BW' in RFM95.h for a list of avaliable bandwidths)
@return:	NA
*/
void RFM95_Set_Bandwidth(uint8_t BW){

	uint8_t temp=0;
	RFM95_Reg_Read(RFM95_REG_1D_MODEM_CONFIG1, &temp, 1);			//current value of the registor

	BW=BW<<4;
	BW&=RFM95_BW;
	temp&=!RFM95_BW;												//clear the current BW
	BW|=temp;														//set the new BW
	RFM95_Reg_Write(RFM95_REG_1D_MODEM_CONFIG1, &BW, 1);
}

/*
@brief:		Gets the bandwidth of the current radio transmission
@param: 	NA
@return:	The bandwidth bits in configuration registor one
*/
uint8_t RFM95_Get_Bandwidth(void){

	uint8_t BW=0;
	RFM95_Reg_Read(RFM95_REG_1D_MODEM_CONFIG1, &BW, 1);				//current value of the registor
	BW&=RFM95_BW;
	BW=BW>>4;
	return(BW);
}

/*
@brief:		Sets the output power of the radio transmission
@param: 	OPP - the desired output power (between 0-15)
@return:	NA
*/
void RFM95_Set_Output_Power(uint8_t OPP){

	uint8_t temp=0;
	RFM95_Reg_Read(RFM95_REG_09_PA_CONFIG, &temp, 1);				//current value of the registor

	OPP&=RFM95_OUTPUT_POWER;
	temp&=!RFM95_OUTPUT_POWER;										//clear the current OPP
	OPP|=temp;														//set the new OPP
	OPP|=RFM95_PA_SELECT;											//set PA_BOOST
	RFM95_Reg_Write(RFM95_REG_09_PA_CONFIG, &OPP, 1);
}

/*
@brief:		Gets the output power of the current radio transmission
@param: 	NA
@return:	The output power bits in the power amp configuration registor
*/
uint8_t RFM95_Get_Output_Power(void){

	uint8_t OPP=0;
	RFM95_Reg_Read(RFM95_REG_09_PA_CONFIG, &OPP, 1);				//current value of the registor
	OPP&=RFM95_OUTPUT_POWER;
	return(OPP);
}

/*
@brief:		Sets the hop period of the radio transmission
@param: 	HP - the desired hop period (something to do with 'symbols' I think)
@return:	NA
*/
void RFM95_Set_Hop_Period(uint8_t HP){

	RFM95_Reg_Write(RFM95_REG_24_HOP_PERIOD , &HP, 1);
}

/*
@brief:		Gets the hop period of the current radio transmission
@param: 	NA
@return:	The ovalue of the hop period registor
*/
uint8_t RFM95_Get_Hop_Period(void){

	uint8_t HP=0;
	RFM95_Reg_Read(RFM95_REG_24_HOP_PERIOD , &HP, 1);
	return(HP);
}

/*
@brief:		Gets the average RSSI of the last packet
@param: 	NA
@return:	The average RSSI of the last packet (dBm)
*/
int RFM95_Get_RSSI(void){

	int RSSI=0;
	uint8_t temp=0;
	RFM95_Reg_Read(RFM95_REG_1B_RSSI_VALUE , &temp, 1);
	RSSI=temp-164;
	return(RSSI);
}

/*
@brief:		Gets the SNR of the last packet
@param: 	NA
@return:	The SNR of the last packet
*/
int RFM95_Get_SNR(void){

	int SNR=0;
	uint8_t temp=0;
	RFM95_Reg_Read(RFM95_REG_19_PKT_SNR_VALUE , &temp, 1);
	if(temp&0b10000000){
		temp--;
		temp=~temp;
		SNR=(int)(-1*temp/4);
	}
	else{
		SNR=(int)(temp/4);
	}

	return(SNR);
}

/*
@brief:		Maps which interupts go will be seen on digital I/Os 0,1,2&3
@param: 	DIO - which DIO to map (0,1,2 or 3)
			Map - what value to map it to (0,1,2 or 3) (see 'Table 18 DIO Mapping LoRa Mode' in the data sheet for full list)
@return:	NA
*/
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

/*
@brief:		Maps which interupts go will be seen on digital I/Os 4&5
@param: 	DIO - which DIO to map (4 or 5)
			Map - what value to map it to (0,1,2 or 3) (see 'Table 18 DIO Mapping LoRa Mode' in the data sheet for full list)
@return:	NA
*/
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

/*
@brief:		Gets the DIO map for a DIO in  map registor one
@param: 	DIO - which DIO you want to get the mapping for (0,1,2 or 3)
@return:	The value of the mapping bits for that DIO
*/
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

/*
@brief:		Gets the DIO map for a DIO in  map registor one
@param: 	DIO - which DIO you want to get the mapping for (4 or 5)
@return:	The value of the mapping bits for that DIO
*/
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

/*
@brief:		Sets up the radio with the desired frequency, payload length,
			coding rate, spreading factor, bandwidth and output power.
			It also initialisies the global variables used in the layer two
			and three protocols. (it might be a good idea to put these in the
			main function and leave RFM95 to be purely physical layer)
@param: 	All the paraeters are directly calling the above functions
			so see those if you want to know more
@return:	NA
*/
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
	RFM95_Set_Hop_Period(3);

	L2HEADER.OLD_ID=8;

	BASE_DATA.NXT=NULL;

	BASE_ADDRESS.NXT=NULL;

	if(ADDRESS==GTW){
		L3NODE.WEIGHT=0;
		L3NODE.NXT_HOP_FOUND=1;
	}
	else{
		L3NODE.WEIGHT=255;
		L3NODE.NXT_HOP_FOUND=0;
	}
	L3NODE.LAST_RU=0;
	L3NODE.NXT_HOP=GTW;
	L3NODE.FIFO=&BASE_ADDRESS;
	L3NODE.DATA=&BASE_DATA;

}

/*
@brief:		Changes the frequency when performing frequency hopping
@param: 	NA
@return:	NA
*/
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

/*
@brief:		This is the first half of the layer two protocol. It splits the message into
			fixed length packets and then sends them one at a time. It also keeps track of
			the time to live of each packet and what to send next.
@param: 	Data - a variable length message
			Len  - the size of the message in bytes
@return:	NA
*/
void Layer2_Send(uint8_t *Data, uint8_t Len){

	//New Message
	if(PACKETS==0){
		PACKETS=Len/DATA_SIZE+((Len%DATA_SIZE)!=0);
		LEN=PACKETS*DATA_SIZE;
		DATA=(uint8_t*)malloc(LEN);
		memset(DATA, 0, LEN);
		LEN-=DATA_SIZE;
		L2HEADER.TTL=3;
		for(int x=0;x<Len;x++){
			*DATA=*Data;
			DATA++;
			Data++;
		}
		DATA=DATA-Len;
	}

	if(PACKETS==1){
		L2HEADER.ID=0;
	}
	else{
		L2HEADER.ID=(PACKETS%7)+1;
	}

	L2HEADER.DST=L3NODE.NXT_HOP;
	L2HEADER.SRC=ADDRESS;
	L2HEADER.CHECK=Check_CRC(DATA);
	TIM2->CNT=0;
	timing2(1);
	LoRa_Send(DATA);
}

/*
@brief:		This is the second half of the layer 2 send. It sends a fixed size message using LoRa
			This is also where the layer two header is added to the transmission. I thought this
			was a good idea because all messages are recieved by the layer two protocol and thus
			require the correct header or they will be ignored. In this way you can send a message
			without going through the full layer two send proccess and it will be recieved at the
			other end. Handy if you don't want to resend if it's not acknowledged etc.
@param: 	Data - a fixed length ('DATA_SIZE' in RFM95.h) message
@return:	NA
*/
void LoRa_Send(uint8_t *Data){

	uint8_t len =3+DATA_SIZE;
	uint8_t header[len];

	header[0]=L2HEADER.DST;
	header[1]=L2HEADER.SRC;
	header[2]= (L2HEADER.TTL) | (L2HEADER.ID <<2) | (L2HEADER.CHECK<< 5);
	for(int j=0;j<DATA_SIZE;j++){
		header[j+3]=*Data;
		Data++;
	}

	RFM95_Set_Mode(RFM95_LONG_RANGE_MODE|RFM95_MODE_STDBY);			//Enter stand by mode
	RFM95_Set_Freq(915.25);
	uint8_t txbase = 0;												//Set FifoPtrAddr to FifoTxPtrBase
	RFM95_Reg_Read(RFM95_REG_0E_FIFO_TX_BASE_ADDR,&txbase,1);
	RFM95_Reg_Write(RFM95_REG_0D_FIFO_ADDR_PTR , &txbase, 1);

	RFM95_Reg_Write(RFM95_REG_22_PAYLOAD_LENGTH, &len, 1);			//Set the payload length

	RFM95_Reg_Write(RFM95_REG_00_FIFO , &header[0], len);			//Write data to FIFO

	RFM95_Set_Mode(RFM95_LONG_RANGE_MODE|RFM95_MODE_TX);			//Enter Transmit mode

}

/*
@brief:		Creates a check on the data by counting how many ones are in the data and
			taking the remainder when divided by eight. (I use this in the layer two header)
@param: 	buf - the data to be checked (must be of length 'DATA_SIZE')
@return:	remainder 8 of the number of ones in buf
*/
uint8_t Check_CRC(uint8_t *buf){

	int temp1=0;
	uint8_t temp2=*buf;

	for(int x=0;x<DATA_SIZE;x++){
		for(int y=0;y<8;y++){
			if(temp2 & 0x01){
				temp1++;
			}
			temp2=temp2>>1;
		}
		buf++;
		temp2=*buf;
	}
	temp1=temp1%8;
	return(temp1);
}

/*
@brief:		Acknowledges a correctly recieved packet so the sender know it was recieved
@param: 	Address - The address of the sending node
			ID - the packet ID (used in the layer two header) of the packet being acknowledged
@return:	NA
*/
void Send_ACK(uint8_t Address, uint8_t ID){

	char blank[DATA_SIZE];
	blank[0]=(char)ID;
	L2HEADER.DST=Address;
	L2HEADER.SRC=ACK;
	L2HEADER.TTL=3;
	L2HEADER.CHECK=Check_CRC((uint8_t*)&blank[0]);
	LoRa_Send((uint8_t*)&blank[0]);
}

/*
@brief:		This is the layer two RX protocol. The radio handles the layer one RX and then sends an interupt which triggers
			this protocol. This protocol then triggers the layer three RX when the full message has been recieved.
@param: 	NA
@return:	NA
*/
void LoRa_RX(void){
	uint8_t rxbase = 0;												//Set FifoPtrAddr to FifoRxCurrentAddr
	RFM95_Reg_Read(RFM95_REG_10_FIFO_RX_CURRENT_ADDR,&rxbase,1);
	RFM95_Reg_Write(RFM95_REG_0D_FIFO_ADDR_PTR , &rxbase, 1);

	uint8_t len =0;													//How many bits of data have we received
	RFM95_Reg_Read(RFM95_REG_22_PAYLOAD_LENGTH,&len,1);

	uint8_t *buf = (uint8_t*) malloc(len);							//Make a buffer to stor the data
	RFM95_Reg_Read(RFM95_REG_00_FIFO, buf, len);

	struct L2Header header;
	header.DST = *buf;
	buf++;
	header.SRC = *buf;
	buf++;
	header.TTL = *buf & 0b11;
	header.ID = (*buf>>2) & 0b111;
	header.CHECK = (*buf>>5) & 0b111;
	buf++;

	if((header.CHECK==Check_CRC(buf)) & ((header.DST==ADDRESS) | (header.DST==BROADCAST)| (header.DST==ACK))){

		if (header.SRC==ACK){
			timing2(0);
			if(PACKETS>1){
				DATA+=DATA_SIZE;
				PACKETS--;
				L2HEADER.TTL=3;
				Layer2_Send(DATA,1);
			}
			else if(PACKETS==1){
				PACKETS--;
				DATA=DATA-LEN;
				free(DATA);
			}
		}
		else{

			if(header.ID==0){
				L2HEADER.OLD_ID=8;
				Set_L3Data(buf);
				L3_RX(header.SRC);
			}

			else if((L2HEADER.OLD_ID==8)|((header.ID%7+1)==L2HEADER.OLD_ID)){
				Send_ACK(header.SRC,header.ID);
				L2HEADER.OLD_ID=header.ID;
				Set_L3Data(buf);
			}
		}
	}

	buf-=3;
	free(buf);														//Free the buffer

	uint8_t IRQ_Flags=0xFF;											//clear flags on LoRa Radio
	RFM95_Reg_Write(RFM95_REG_12_IRQ_FLAGS , &IRQ_Flags, 1);
	RFM95_Reg_Write(RFM95_REG_12_IRQ_FLAGS , &IRQ_Flags, 1);

	RFM95_Set_Mode(RFM95_LONG_RANGE_MODE|RFM95_MODE_RXCONTINUOUS);	//Enter RX mode

}

void Test_LoRa_RX(void){
	uint8_t rxbase = 0;												//Set FifoPtrAddr to FifoRxCurrentAddr
	RFM95_Reg_Read(RFM95_REG_10_FIFO_RX_CURRENT_ADDR,&rxbase,1);
	RFM95_Reg_Write(RFM95_REG_0D_FIFO_ADDR_PTR , &rxbase, 1);

	uint8_t len =0;													//How many bits of data have we received
	RFM95_Reg_Read(RFM95_REG_22_PAYLOAD_LENGTH,&len,1);

	uint8_t *buf = (uint8_t*) malloc(len);							//Make a buffer to stor the data
	RFM95_Reg_Read(RFM95_REG_00_FIFO, buf, len);

	struct L2Header header;
	header.DST = *buf;
	buf++;
	header.SRC = *buf;
	buf++;
	header.TTL = *buf & 0b11;
	header.ID = (*buf>>2) & 0b111;
	header.CHECK = (*buf>>5) & 0b111;
	buf++;

	if(header.CHECK==Check_CRC(buf)){
		GOOD++;
	}
	else{
		BAD++;
	}

	char test[40];
	sprintf(test,"Good %d, Bad %d", GOOD,BAD);
	burstSerial(&test[0],strlen(test));

	SendTST();

	buf-=3;
	free(buf);														//Free the buffer

	uint8_t IRQ_Flags=0xFF;											//clear flags on LoRa Radio
	RFM95_Reg_Write(RFM95_REG_12_IRQ_FLAGS , &IRQ_Flags, 1);
	RFM95_Reg_Write(RFM95_REG_12_IRQ_FLAGS , &IRQ_Flags, 1);

	RFM95_Set_Mode(RFM95_LONG_RANGE_MODE|RFM95_MODE_RXCONTINUOUS);	//Enter RX mode

}

/*
@brief:		This is the layer three RX. It handels all the network related messages as well as sending messages
			to the gateway.
@param: 	Source - the address of the node that a message was just recieved from (not utilised as much as it could
			be. The four network messages double up this infomation so could probably make this more efficient).
@return:	NA
*/
void L3_RX(uint8_t Source){

	struct Data_Node data;
	data=BASE_DATA;

	int size=DATA_SIZE;
	while(data.NXT->NXT!=NULL){
		size+=DATA_SIZE;
		data=*data.NXT;
	}

	uint8_t *message =(uint8_t*) malloc(size);
	uint8_t *temp=message;

	data=BASE_DATA;
	for(int x=0;x<DATA_SIZE;x++){
		*temp=data.DATA[x];
		temp++;
	}
	while(data.NXT->NXT!=NULL){
		data=*data.NXT;
		for(int x=0;x<DATA_SIZE;x++){
			*temp=data.DATA[x];
			temp++;
		}
	}

	temp=message;
	uint8_t type = *temp;
	temp++;

	if(type==RU){
		uint8_t lastRU=*temp;
		temp++;
		uint8_t add=*temp;
		temp++;
		uint8_t weight=*temp;
		if(((L3NODE.LAST_RU==lastRU)&(weight+1<L3NODE.WEIGHT))|(L3NODE.LAST_RU!=lastRU)){
			L3NODE.WEIGHT=weight+1;
			L3NODE.NXT_HOP=add;
			L3NODE.NXT_HOP_FOUND=1;
			L3NODE.LAST_RU=lastRU;

			for(int x=0;x<20*ADDRESS;x++){
				while(SysTick->VAL!=0);
			}

			SendRUA(add);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		}
	}
	else if(type==RUA){
		struct Address_Node *next = (struct Address_Node *)malloc(sizeof(struct Address_Node));
		next->NXT=NULL;
		L3NODE.FIFO->ADD=*temp;
		L3NODE.FIFO->NXT=next;
		L3NODE.FIFO=next;

		char test[40];
		sprintf(test,"RUA from node %d",*temp);
		burstSerial(&test[0],strlen(test));
	}
	else if(type==TR){
		uint8_t add=*temp;
		if(add==L3NODE.NXT_HOP){
			SendRU(L3NODE.LAST_RU);
		}
		else{
			SendTRD(add);
		}


	}
	else if(type==TRD){
		struct Address_Node *tempNode = L3NODE.FIFO;

		char test[40];
		sprintf(test,"TRD from node %d",Source);
		burstSerial(&test[0],strlen(test));

		if(L3NODE.FIFO->NXT!=NULL){
			SendTR(L3NODE.FIFO->ADD);
			L3NODE.FIFO=L3NODE.FIFO->NXT;
		}
		else{
			SendTRD(L3NODE.NXT_HOP);
		}
		free(tempNode);
	}
	else if(type==TRNSFR){
		//transfer protocol
		Send_ACK(Source,0);
		if(ADDRESS==0x01){
			burstSerial((char*)temp,size-1);
		}
		else{
			L3_TX(temp,(size-1));
		}
	}
	else if(type==TST){
		//test message
	}
	else{
		//unknown message type
		Send_ACK(Source,0);
	}

	free(message);
	Clean(BASE_DATA);
}

/*
@brief:		This is the layer three send used for transmitting data to the gateway.
			It is pretty basic because layer two does most of the heavy lifting so
			only need to add the small layer three header which specifies which type
			of layer three message it is (this case is a transfer)
@param: 	Data - a variable length message
			Len  - the size of the message in bytes
@return:	NA
*/
void L3_TX(uint8_t *Data, uint8_t Len){

	if(L3NODE.NXT_HOP_FOUND){
		uint8_t *header =(uint8_t*) malloc(Len+1);
		uint8_t *temp=header;

		*temp=TRNSFR;
		temp++;
		for(int j=0;j<Len;j++){

			*temp=*Data;
			Data++;
			temp++;

		}

		Layer2_Send(header,Len+1);
		free(header);
	}
}

/*
@brief:		The original message gets split into several packets and then they each get received one at a time.
			To put the message back together a linked list is used so each each time a packet is received the
			header is removed and the packet added to the linked list. This protocol adds the packets to the linked list.
@param: 	Data - the data that should be added to the linked list.
@return:	NA
*/
void Set_L3Data(uint8_t *Data){
	for(int x=0;x<DATA_SIZE;x++){
		L3NODE.DATA->DATA[x]=*Data;
		Data++;
	}

	struct Data_Node *next = (struct Data_Node *)malloc(sizeof(struct Data_Node));
	next->NXT=NULL;
	L3NODE.DATA->NXT=next;
	L3NODE.DATA=next;

}

/*
@brief:		Each time Set_L3Data is called memory is allocated to be used in the linked list. After we are finished with
			the message we should clean up to avoid memory leaks. The way I coded it the first node in the linked list
			is a global variable so we don't free that one but we free all the other nodes.
@param: 	Node - the first node in the list to be cleaned. I didn't need this because the first node is always the same
			and it's a global variable (BASE_DATA) so might be a good idea to change this to void and just use BASE_DATA.
			Not sure though so I'll just leave it.
@return:	NA
*/
void Clean(struct Data_Node Node){
	struct Data_Node *temp;
	if(Node.NXT!=NULL){
		temp=Node.NXT;
		while(temp!=NULL){
			Node=*Node.NXT;
			free(temp);
			temp=Node.NXT;
		}
	}
	BASE_DATA.NXT=NULL;
	L3NODE.DATA=&BASE_DATA;
}

/*
@brief:		Creates and sends a routing update.
@param: 	RUID - the ID of the routing update. Used so that nodes know if it is a new update,
			if it is, it accept the new node even if it's not as good as it's current one.
@return:	NA
*/
void SendRU(uint8_t RUID){
	char test[40]="sending router update";
	burstSerial(&test[0],strlen(test));

	uint8_t data[DATA_SIZE];
	data[0]=RU;
	data[1]=RUID;
	data[2]=ADDRESS;
	data[3]=L3NODE.WEIGHT;
	L2HEADER.ID=0;
	L2HEADER.DST=BROADCAST;
	L2HEADER.SRC=ADDRESS;
	L2HEADER.CHECK=Check_CRC(data);

	timing3(1);

	LoRa_Send(data);
}

/*
@brief:		Creates and sends a routing update acknowledgement.
@param: 	Address -  the address of the node sending the RUA
			(this is redundent and could be removed as the receiver
			already knows who is sending from the L2 header)
@return:	NA
*/
void SendRUA(uint8_t Address){

	uint8_t data[DATA_SIZE];
	data[0]=RUA;
	data[1]=ADDRESS;
	L2HEADER.ID=0;
	L2HEADER.DST=Address;
	L2HEADER.SRC=ADDRESS;
	L2HEADER.CHECK=Check_CRC(&data[0]);
	LoRa_Send(&data[0]);
}

/*
@brief:		Creates and sends a top router message. (when you receive
			a TR message it means it's your turn to send out a RU)
@param: 	Address -  the address of the node sending the RUA
			(this is redundent and could be removed as the receiver
			already knows who is sending from the L2 header)
@return:	NA
*/
void SendTR(uint8_t Address){

	uint8_t data[DATA_SIZE];
	data[0]=TR;
	data[1]=ADDRESS;
	L2HEADER.ID=0;
	L2HEADER.DST=Address;
	L2HEADER.SRC=ADDRESS;
	L2HEADER.CHECK=Check_CRC(&data[0]);
	LoRa_Send(&data[0]);
	char serial[40];
	sprintf(serial,"%d is the top router",Address);
	burstSerial(&serial[0],strlen(serial));
}

/*
@brief:		Creates and sends a top router done message.
@param: 	Address -  the address of the node who sent you the TR
@return:	NA
*/
void SendTRD(uint8_t Address){

	uint8_t data[DATA_SIZE];
	data[0]=TRD;
	L2HEADER.ID=0;
	L2HEADER.DST=Address;
	L2HEADER.SRC=ADDRESS;
	L2HEADER.CHECK=Check_CRC(&data[0]);
	LoRa_Send(&data[0]);
	char serial[40]="Routing table up to date";
	burstSerial(&serial[0],strlen(serial));
}

/*
@brief:		Sends a test message
@param: 	NA
@return:	NA
*/
void SendTST(void){

	uint8_t data[DATA_SIZE];
	data[0]=TST;
	L2HEADER.ID=0;
	L2HEADER.DST=BROADCAST;
	L2HEADER.SRC=ADDRESS;
	L2HEADER.CHECK=Check_CRC(&data[0]);
	LoRa_Send(&data[0]);

	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
}

/*
@brief:		Prints out the list of nodes that responded to your router
			update and who you will eventually send TR messages to.
@param: 	NA
@return:	NA
*/
void PrintRUList(void){
	char serial[40];
	struct Address_Node addNode=BASE_ADDRESS;
	sprintf(serial,"The nodes that responded are:");
	burstSerial(&serial[0],strlen(serial));
	while(addNode.NXT!=NULL){
		sprintf(serial,"Node %d",addNode.ADD);
		burstSerial(&serial[0],strlen(serial));
		addNode=*addNode.NXT;
	}
	L3NODE.FIFO=&BASE_ADDRESS;
}

/*
@brief:		Checks if you have any more nodes in your RU list,
			if you do then you send one of those nodes a TR
			else you send a TRD to the node who sent you your TR
@param: 	NA
@return:	NA
*/
void UpdateTR(void){
	struct Address_Node *temp;

	if(L3NODE.FIFO->NXT==NULL){
		if(L3NODE.FIFO!=&BASE_ADDRESS){
			free(L3NODE.FIFO);
		}
		SendTRD(L3NODE.NXT_HOP);
	}
	else{
		SendTR(L3NODE.FIFO->ADD);
		temp=L3NODE.FIFO;
		L3NODE.FIFO=L3NODE.FIFO->NXT;
		if(temp!=&BASE_ADDRESS){
			free(temp);
		}
	}
}
