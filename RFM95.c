/*
 * RFM95.c
 *
 *  Created on: 16Mar.,2018
 *      Author: jens0
 */

#include "RFM95.h"

uint8_t *DATA;
uint8_t PACKETS=0;
uint8_t LEN=0;
uint8_t RXNEXT=0;

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

uint8_t RFM95_Get_RX_NBYTES(void){
	uint8_t PLL=0;
	RFM95_Reg_Read(RFM95_REG_13_RX_NB_BYTES, &PLL, 1);
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

int RFM95_Get_RSSI(void){

	int RSSI=0;
	uint8_t temp=0;
	RFM95_Reg_Read(RFM95_REG_1B_RSSI_VALUE , &temp, 1);
	RSSI=temp-164;
	return(RSSI);
}

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

void RFM95_Set_CRC(uint8_t SET){

	uint8_t temp=0;
	RFM95_Reg_Read(RFM95_REG_1E_MODEM_CONFIG2, &temp, 1);			//current value of the registor

	SET=SET<<2;
	SET&=RFM95_PAYLOAD_CRC_ON;
	temp&=!RFM95_PAYLOAD_CRC_ON;									//clear the current CRC
	SET|=temp;														//set the new CRC
	RFM95_Reg_Write(RFM95_REG_1E_MODEM_CONFIG2, &SET, 1);
}

uint8_t RFM95_Get_CRC(void){

	uint8_t SET=0;
	RFM95_Reg_Read(RFM95_REG_1E_MODEM_CONFIG2, &SET, 1);
	SET&=RFM95_PAYLOAD_CRC_ON;
	SET=SET>>2;
	return(SET);
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

void Send_ACK(uint8_t Address, uint8_t ID){

	char blank[DATA_SIZE];
	blank[0]=(char)ID;
	L2HEADER.DST=Address;
	L2HEADER.SRC=ACK;
	L2HEADER.TTL=3;
	L2HEADER.CHECK=Check_CRC((uint8_t*)&blank[0]);
	LoRa_Send((uint8_t*)&blank[0]);
}

uint8_t Get_DST(void){

	return(L3NODE.NXT_HOP);
}

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
				Test3_L3_RX(header.SRC);
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
				Test3_L3_RX(header.SRC);
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

void Ping_Test(void){
	char test[DATA_SIZE];
	sprintf(test,"%061d",123456);
	L2HEADER.DST=Get_DST();
	L2HEADER.SRC=ADDRESS;
	L2HEADER.CHECK=Check_CRC((uint8_t*)&test[0]);
	L2HEADER.TTL=3;
	L2HEADER.ID=0;
	LoRa_Send((uint8_t*)&test[0]);
}

void Ping_Test2(void){
	char test[20]="Hello World!";
	PACKETS=0;
	Layer2_Send((uint8_t*)&test[0],strlen(test));
}

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

	L2HEADER.DST=Get_DST();
	L2HEADER.SRC=ADDRESS;
	L2HEADER.CHECK=Check_CRC(DATA);
	TIM2->CNT=0;
	timing2(1);
	LoRa_Send(DATA);

}

void Test3_L3_RX(uint8_t Source){

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
			Test_L3_TX(temp,(size-1));
		}
	}
	else{
		//unknown message type
		Send_ACK(Source,0);
	}

	free(message);
	Clean(BASE_DATA);
}

void Test2_L3_RX(uint8_t Source){

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

	if(type==TRNSFR){
		//transfer protocol
		if(ADDRESS==0x03){
			burstSerial((char*)temp,size-1);
		}
		else{
			Test_L3_TX(temp,(size-1));
		}
	}
	else if(type==RU){
		//router update protocol
	}
	else if(type==RUA){
		//router update acknowledge protocol
	}
	else if(type==TR){
		//top router protocol
	}
	else if(type==TRD){
		//top router done protocol
	}
	else{
		//unknown message type
	}

	free(message);
	Clean(BASE_DATA);
}

void Test_L3_RX(void){

	struct Data_Node data;
	data=BASE_DATA;
	for(int x=0;x<DATA_SIZE;x++){
		serial(data.DATA[x]);
	}
	while(data.NXT->NXT!=NULL){
		data=*data.NXT;
		for(int x=0;x<DATA_SIZE;x++){
			serial(data.DATA[x]);
		}
	}
	serial(13);
	Clean(BASE_DATA);
}

void Test_L3_TX(uint8_t *Data, uint8_t Len){

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
}

void UpdateTR(void){
	struct Address_Node *temp;

	if(L3NODE.FIFO->NXT==NULL){
		if(L3NODE.FIFO!=&BASE_ADDRESS){
			free(L3NODE.FIFO);
		}

		SendTRD(L3NODE.NXT_HOP);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
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

void Power_Test(void){
	uint8_t txbase=0x80;
	RFM95_Reg_Write(RFM95_REG_0D_FIFO_ADDR_PTR , &txbase, 1);
	RFM95_Set_Mode(RFM95_LONG_RANGE_MODE|RFM95_MODE_TX);

}
