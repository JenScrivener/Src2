/*
 * RFM95.h
 *
 *  Created on: 16Mar.,2018
 *      Author: jens0
 */

#ifndef RFM95_H_
#define RFM95_H_

/* Includes ------------------------------------------------------------------*/
#include "SPI.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "main.h"

struct L2Header{

	uint8_t CHECK:3;
	uint8_t ID:3;
	uint8_t TTL:2;
	uint8_t SRC;
	uint8_t DST;

};

volatile uint8_t ACK_TRUE;
volatile uint8_t WAIT;
struct L2Header L2HEADER;

//functions

void RFM95_Reg_Write(uint8_t Reg, uint8_t* Data, uint8_t Len);		//Write len bytes of data to reg
void RFM95_Reg_Read(uint8_t Reg, uint8_t* Data, uint8_t Len);		//Read len bytes of data from reg

void Init_SPI_Handle(void);

void CS(int state);

void RFM95_Set_Mode(uint8_t Mode);									//Set Mode reg (0x01)
uint8_t RFM95_Get_Mode(void);										//Read Mode reg (0x01)

void RFM95_Set_Freq(double Freq);									//Set freq reg (0x06,07&08) to Freq (freq should be in MHz i.e 915)
double RFM95_Get_Freq(void);										//Get freq from reg(0x06,07,08) in MHz

void RFM95_Set_Payload_Length(uint8_t PLL);
uint8_t RFM95_Get_Payload_Length(void);

void RFM95_Set_Coding_Rate(uint8_t CR);
uint8_t RFM95_Get_Coding_Rate(void);

void RFM95_Set_Spreading_Factor(uint8_t SF);
uint8_t RFM95_Get_Spreading_Factor(void);

void RFM95_Set_Bandwidth(uint8_t BW);
uint8_t RFM95_Get_Bandwidth(void);

void RFM95_Set_Output_Power(uint8_t OPP);
uint8_t RFM95_Get_Output_Power(void);

void RFM95_Set_Hop_Period(uint8_t HP);
uint8_t RFM95_Get_Hop_Period(void);

void RFM95_Set_CRC(uint8_t SET);
uint8_t RFM95_Get_CRC(void);

void RFM95_DIO_Map(uint8_t DIO, uint8_t Map);
uint8_t RFM95_Get_DIO_Map(uint8_t DIO);

void RFM95_LoRa_Init(double Freq, uint8_t PayloadLength, uint8_t CodingRate, uint8_t SpreadingFactor, uint8_t Bandwidth, uint8_t OutputPower);

void Hop(void);
void RFM95_LoRa_Test_Send(uint8_t *Data, uint8_t Len);

void Layer2_Send(uint8_t *Data, uint8_t Len);
void LoRa_Send(uint8_t *Data);
uint8_t Check_CRC(uint8_t *buf);

void Send_ACK(uint8_t address);
uint8_t Get_DST(void);
void Wait(void);

void LoRa_RX(void);
void Test_LoRa_RX(void);


//unique address for this node
#define ADDRESS  										0xAB
//global address for all nodes
#define GLBADD  										0x00
//used as the source address in acknowledgments
#define ACK  											0xA5

//Register addresses from table 85 Semtech (HopeRF doesn't have an RX current 0x10)
#define RFM95_REG_00_FIFO                                0x00
#define RFM95_REG_01_OP_MODE                             0x01
#define RFM95_REG_02_RESERVED                            0x02
#define RFM95_REG_03_RESERVED                            0x03
#define RFM95_REG_04_RESERVED                            0x04
#define RFM95_REG_05_RESERVED                            0x05
#define RFM95_REG_06_FRF_MSB                             0x06
#define RFM95_REG_07_FRF_MID                             0x07
#define RFM95_REG_08_FRF_LSB                             0x08
#define RFM95_REG_09_PA_CONFIG                           0x09
#define RFM95_REG_0A_PA_RAMP                             0x0a
#define RFM95_REG_0B_OCP                                 0x0b
#define RFM95_REG_0C_LNA                                 0x0c
#define RFM95_REG_0D_FIFO_ADDR_PTR                       0x0d
#define RFM95_REG_0E_FIFO_TX_BASE_ADDR                   0x0e
#define RFM95_REG_0F_FIFO_RX_BASE_ADDR                   0x0f
#define RFM95_REG_10_FIFO_RX_CURRENT_ADDR                0x10
#define RFM95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RFM95_REG_12_IRQ_FLAGS                           0x12
#define RFM95_REG_13_RX_NB_BYTES                         0x13
#define RFM95_REG_14_RX_HEADER_CNT_VALUE_MSB             0x14
#define RFM95_REG_15_RX_HEADER_CNT_VALUE_LSB             0x15
#define RFM95_REG_16_RX_PACKET_CNT_VALUE_MSB             0x16
#define RFM95_REG_17_RX_PACKET_CNT_VALUE_LSB             0x17
#define RFM95_REG_18_MODEM_STAT                          0x18
#define RFM95_REG_19_PKT_SNR_VALUE                       0x19
#define RFM95_REG_1A_PKT_RSSI_VALUE                      0x1a
#define RFM95_REG_1B_RSSI_VALUE                          0x1b
#define RFM95_REG_1C_HOP_CHANNEL                         0x1c
#define RFM95_REG_1D_MODEM_CONFIG1                       0x1d
#define RFM95_REG_1E_MODEM_CONFIG2                       0x1e
#define RFM95_REG_1F_SYMB_TIMEOUT_LSB                    0x1f
#define RFM95_REG_20_PREAMBLE_MSB                        0x20
#define RFM95_REG_21_PREAMBLE_LSB                        0x21
#define RFM95_REG_22_PAYLOAD_LENGTH                      0x22
#define RFM95_REG_23_MAX_PAYLOAD_LENGTH                  0x23
#define RFM95_REG_24_HOP_PERIOD                          0x24
#define RFM95_REG_25_FIFO_RX_BYTE_ADDR                   0x25
#define RFM95_REG_26_MODEM_CONFIG3                       0x26

#define RFM95_REG_27_PPM_CORRECTION                      0x27
#define RFM95_REG_28_FEI_MSB                             0x28
#define RFM95_REG_29_FEI_MID                             0x29
#define RFM95_REG_2A_FEI_LSB                             0x2a
#define RFM95_REG_2C_RSSI_WIDEBAND                       0x2c
#define RFM95_REG_31_DETECT_OPTIMIZ                      0x31
#define RFM95_REG_33_INVERT_IQ                           0x33
#define RFM95_REG_37_DETECTION_THRESHOLD                 0x37
#define RFM95_REG_39_SYNC_WORD                           0x39

#define RFM95_REG_40_DIO_MAPPING1                        0x40
#define RFM95_REG_41_DIO_MAPPING2                        0x41
#define RFM95_REG_42_VERSION                             0x42

#define RFM95_REG_4B_TCXO                                0x4b
#define RFM95_REG_4D_PA_DAC                              0x4d
#define RFM95_REG_5B_FORMER_TEMP                         0x5b
#define RFM95_REG_61_AGC_REF                             0x61
#define RFM95_REG_62_AGC_THRESH1                         0x62
#define RFM95_REG_63_AGC_THRESH2                         0x63
#define RFM95_REG_64_AGC_THRESH3                         0x64

//Registor Masks

// RFM95_REG_01_OP_MODE                             0x01
#define RFM95_LONG_RANGE_MODE                       0x80
#define RFM95_ACCESS_SHARED_REG                     0x40
#define RFM95_LOW_FREQUENCY_MODE                    0x08
#define RFM95_MODE                                  0x07
#define RFM95_MODE_SLEEP                            0x00
#define RFM95_MODE_STDBY                            0x01
#define RFM95_MODE_FSTX                             0x02
#define RFM95_MODE_TX                               0x03
#define RFM95_MODE_FSRX                             0x04
#define RFM95_MODE_RXCONTINUOUS                     0x05
#define RFM95_MODE_RXSINGLE                         0x06
#define RFM95_MODE_CAD                              0x07

// RFM95_REG_09_PA_CONFIG                           0x09
#define RFM95_PA_SELECT                             0x80
#define RFM95_MAX_POWER                             0x70
#define RFM95_OUTPUT_POWER                          0x0f

// RFM95_REG_0A_PA_RAMP                             0x0a
#define RFM95_LOW_PN_TX_PLL_OFF                     0x10
#define RFM95_PA_RAMP                               0x0f
#define RFM95_PA_RAMP_3_4MS                         0x00
#define RFM95_PA_RAMP_2MS                           0x01
#define RFM95_PA_RAMP_1MS                           0x02
#define RFM95_PA_RAMP_500US                         0x03
#define RFM95_PA_RAMP_250US                         0x0
#define RFM95_PA_RAMP_125US                         0x05
#define RFM95_PA_RAMP_100US                         0x06
#define RFM95_PA_RAMP_62US                          0x07
#define RFM95_PA_RAMP_50US                          0x08
#define RFM95_PA_RAMP_40US                          0x09
#define RFM95_PA_RAMP_31US                          0x0a
#define RFM95_PA_RAMP_25US                          0x0b
#define RFM95_PA_RAMP_20US                          0x0c
#define RFM95_PA_RAMP_15US                          0x0d
#define RFM95_PA_RAMP_12US                          0x0e
#define RFM95_PA_RAMP_10US                          0x0f

// RFM95_REG_0B_OCP                                 0x0b
#define RFM95_OCP_ON                                0x20
#define RFM95_OCP_TRIM                              0x1f

// RFM95_REG_0C_LNA                                 0x0c
#define RFM95_LNA_GAIN                              0xe0
#define RFM95_LNA_GAIN_G1                           0x20
#define RFM95_LNA_GAIN_G2                           0x40
#define RFM95_LNA_GAIN_G3                           0x60
#define RFM95_LNA_GAIN_G4                           0x80
#define RFM95_LNA_GAIN_G5                           0xa0
#define RFM95_LNA_GAIN_G6                           0xc0
#define RFM95_LNA_BOOST_LF                          0x18
#define RFM95_LNA_BOOST_LF_DEFAULT                  0x00
#define RFM95_LNA_BOOST_HF                          0x03
#define RFM95_LNA_BOOST_HF_DEFAULT                  0x00
#define RFM95_LNA_BOOST_HF_150PC                    0x11

// RFM95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RFM95_RX_TIMEOUT_MASK                       0x80
#define RFM95_RX_DONE_MASK                          0x40
#define RFM95_PAYLOAD_CRC_ERROR_MASK                0x20
#define RFM95_VALID_HEADER_MASK                     0x10
#define RFM95_TX_DONE_MASK                          0x08
#define RFM95_CAD_DONE_MASK                         0x04
#define RFM95_FHSS_CHANGE_CHANNEL_MASK              0x02
#define RFM95_CAD_DETECTED_MASK                     0x01

// RFM95_REG_12_IRQ_FLAGS                           0x12
#define RFM95_RX_TIMEOUT                            0x80
#define RFM95_RX_DONE                               0x40
#define RFM95_PAYLOAD_CRC_ERROR                     0x20
#define RFM95_VALID_HEADER                          0x10
#define RFM95_TX_DONE                               0x08
#define RFM95_CAD_DONE                              0x04
#define RFM95_FHSS_CHANGE_CHANNEL                   0x02
#define RFM95_CAD_DETECTED                          0x01

// RFM95_REG_18_MODEM_STAT                          0x18
#define RFM95_RX_CODING_RATE                        0xe0
#define RFM95_MODEM_STATUS_CLEAR                    0x10
#define RFM95_MODEM_STATUS_HEADER_INFO_VALID        0x08
#define RFM95_MODEM_STATUS_RX_ONGOING               0x04
#define RFM95_MODEM_STATUS_SIGNAL_SYNCHRONIZED      0x02
#define RFM95_MODEM_STATUS_SIGNAL_DETECTED          0x01

// RFM95_REG_1C_HOP_CHANNEL                         0x1c
#define RFM95_PLL_TIMEOUT                           0x80
#define RFM95_RX_PAYLOAD_CRC_IS_ON                  0x40
#define RFM95_FHSS_PRESENT_CHANNEL                  0x3f

// RFM95_REG_1D_MODEM_CONFIG1                       0x1d
#define RFM95_BW                                    0xf0
#define RFM95_BW_7_8KHZ                             0x00
#define RFM95_BW_10_4KHZ                            0x01
#define RFM95_BW_15_6KHZ                            0x02
#define RFM95_BW_20_8KHZ                            0x03
#define RFM95_BW_31_25KHZ                           0x04
#define RFM95_BW_41_7KHZ                            0x05
#define RFM95_BW_62_5KHZ                            0x06
#define RFM95_BW_125KHZ                             0x07
#define RFM95_BW_250KHZ                             0x08
#define RFM95_BW_500KHZ                             0x09

#define RFM95_CODING_RATE                           0x0e
#define RFM95_CODING_RATE_4_5                       0x01
#define RFM95_CODING_RATE_4_6                       0x02
#define RFM95_CODING_RATE_4_7                       0x03
#define RFM95_CODING_RATE_4_8                       0x04

#define RFM95_IMPLICIT_HEADER_MODE_ON               0x01

// RFM95_REG_1E_MODEM_CONFIG2                       0x1e
#define RFM95_SPREADING_FACTOR                      0xf0
#define RFM95_SPREADING_FACTOR_64CPS                0x06
#define RFM95_SPREADING_FACTOR_128CPS               0x07
#define RFM95_SPREADING_FACTOR_256CPS               0x08
#define RFM95_SPREADING_FACTOR_512CPS               0x09
#define RFM95_SPREADING_FACTOR_1024CPS              0x0a
#define RFM95_SPREADING_FACTOR_2048CPS              0x0b
#define RFM95_SPREADING_FACTOR_4096CPS              0x0c

#define RFM95_TX_CONTINUOUS_MODE                    0x08

#define RFM95_PAYLOAD_CRC_ON                        0x04

#define RFM95_SYM_TIMEOUT_MSB                       0x03

// RFM95_REG_40_DIO_MAPPING1
#define RFM95_DIO0									0xc0
#define RFM95_DIO1									0x30
#define RFM95_DIO2									0x0c
#define RFM95_DIO3									0x03

// RFM95_REG_41_DIO_MAPPING2
#define RFM95_DIO4									0xc0
#define RFM95_DIO5									0x30
#define RFM95_Map_Preamble_Detect					0x01

// RFM95_REG_4B_TCXO                                0x4b
#define RFM95_TCXO_TCXO_INPUT_ON                    0x10

// RFM95_REG_4D_PA_DAC                              0x4d
#define RFM95_PA_DAC_DISABLE                        0x04
#define RFM95_PA_DAC_ENABLE                         0x07

// Constants
#define XOSC										32		//32MHz
#define RFM95_WRITE									0x80
#define LIPD_BW										0.5		//MHz
#define LIPD_Gap									0.158	//MHz

////Hopping Centre Frequencies (MHz)
//#define Channel1									915.3
//#define Channel2									915.9
//#define Channel3									916.6
//#define Channel4									917.2
//#define Channel5									917.9
//#define Channel6									918.5
//#define Channel7									919.2
//#define Channel8									919.9
//#define Channel9									920.5
//#define Channel10									921.2
//#define Channel11									921.8
//#define Channel12									922.5
//#define Channel13									923.1
//#define Channel14									923.8
//#define Channel15									924.5
//#define Channel16									925.1
//#define Channel17									925.8
//#define Channel18									926.4
//#define Channel19									927.1
//#define Channel20									927.7

#endif /* RFM95_H_ */
