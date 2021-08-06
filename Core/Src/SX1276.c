/*
 * SX1276.c
 *
 *  Created on: Sep 12, 2020
 *      Author: koh yoshimoto
 */

#include "SX1276.h"

//hardware
void SX1276_hw_init(SX1276_hw_t * hw) {
	SX1276_hw_SetNSS(hw, 1);
	HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_SET);
}

void SX1276_hw_SetNSS(SX1276_hw_t * hw, int value) {
	HAL_GPIO_WritePin(hw->nss.port, hw->nss.pin,
			(value == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SX1276_hw_Reset(SX1276_hw_t * hw) {
	SX1276_hw_SetNSS(hw, 1);
	HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_RESET);

	SX1276_hw_DelayMs(1);

	HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_SET);

	SX1276_hw_DelayMs(100);
}

void SX1276_hw_SPICommand(SX1276_hw_t * hw, uint8_t cmd) {
	SX1276_hw_SetNSS(hw, 0);
	HAL_SPI_Transmit(hw->spi, &cmd, 1, 1000);
	while (HAL_SPI_GetState(hw->spi) != HAL_SPI_STATE_READY)
		;
}

uint8_t SX1276_hw_SPIReadByte(SX1276_hw_t * hw) {
	uint8_t txByte = 0x00;
	uint8_t rxByte = 0x00;

	SX1276_hw_SetNSS(hw, 0);
	HAL_SPI_TransmitReceive(hw->spi, &txByte, &rxByte, 1, 1000);
	while (HAL_SPI_GetState(hw->spi) != HAL_SPI_STATE_READY);
	return rxByte;
}

void SX1276_hw_DelayMs(uint32_t msec) {
	HAL_Delay(msec);
}

int SX1276_hw_GetDIO0(SX1276_hw_t * hw) {
	return (HAL_GPIO_ReadPin(hw->dio0.port, hw->dio0.pin) == GPIO_PIN_SET);
}
int SX1276_hw_GetDIO1(SX1276_hw_t * hw) {
	return (HAL_GPIO_ReadPin(hw->dio1.port, hw->dio1.pin) == GPIO_PIN_SET);
}
int SX1276_hw_GetDIO2(SX1276_hw_t * hw) {
	return (HAL_GPIO_ReadPin(hw->dio2.port, hw->dio2.pin) == GPIO_PIN_SET);
}

//////////////////////////////////
// logic
//////////////////////////////////
uint8_t SX1276_SPIRead(SX1276_t * module, uint8_t addr) {
	uint8_t tmp;
	SX1276_hw_SPICommand(module->hw, addr);
	tmp = SX1276_hw_SPIReadByte(module->hw);
	SX1276_hw_SetNSS(module->hw, 1);
	return tmp;
}

void SX1276_SPIWrite(SX1276_t * module, uint8_t addr, uint8_t cmd) {
	SX1276_hw_SetNSS(module->hw, 0);
	SX1276_hw_SPICommand(module->hw, addr | 0x80);
	SX1276_hw_SPICommand(module->hw, cmd);
	SX1276_hw_SetNSS(module->hw, 1);
}

void SX1276_SPIBurstRead(SX1276_t * module, uint8_t addr, uint8_t* rxBuf,uint8_t length) {
	uint8_t i;
	if (length <= 1) {
		return;
	} else {
		SX1276_hw_SetNSS(module->hw, 0);
		SX1276_hw_SPICommand(module->hw, addr);
		for (i = 0; i < length; i++) {
			*(rxBuf + i) = SX1276_hw_SPIReadByte(module->hw);
		}
		SX1276_hw_SetNSS(module->hw, 1);
	}
}

void SX1276_SPIBurstWrite(SX1276_t * module, uint8_t addr, uint8_t* txBuf, uint8_t length) {
	unsigned char i;
	if (length <= 1) {
		return;
	} else {
		SX1276_hw_SetNSS(module->hw, 0);
		SX1276_hw_SPICommand(module->hw, addr | 0x80);
		for (i = 0; i < length; i++) {
			SX1276_hw_SPICommand(module->hw, *(txBuf + i));
		}
		SX1276_hw_SetNSS(module->hw, 1);
	}
}

void SX1276_defaultConfig(SX1276_t * module) {
	SX1276_config(module, module->frequency, module->power, module->LoRa_Rate, module->LoRa_BW);
}

void SX1276_config(SX1276_t * module, uint8_t frequency,
		           uint8_t power, uint8_t LoRa_Rate, uint8_t LoRa_BW) {
	SX1276_sleep(module); //Change modem mode Must in Sleep mode
	SX1276_hw_DelayMs(15);
	SX1276_entryLoRa(module);
	//SX1276_SPIWrite(module, 0x5904); //?? Change digital regulator form 1.6V to 1.47V: see extra note

	SX1276_SPIBurstWrite(module, LR_RegFrMsb,
			(uint8_t*) SX1276_Frequency[frequency], 3); //setting frequency parameter

	//setting base parameter
	SX1276_SPIWrite(module, LR_RegPaConfig, SX1276_Power[power]); //Setting output power parameter
	SX1276_SPIWrite(module, LR_RegOcp, 0x0B); //RegOcp = 0x0B Ocp disabled(0x1B Ocp enabled)
	SX1276_SPIWrite(module, LR_RegLna, 0x23); //LNA gain setting: G1(= maximum gain)

	if (SX1276_SpreadFactor[LoRa_Rate] == 6) {	//SFactor=6
		uint8_t tmp;
		SX1276_SPIWrite(module,
						LR_RegModemConfig1,
						((SX1276_LoRaBandwidth[LoRa_BW] << 4) + (SX1276_CR << 1) + 0x01)
						); //Implicit Enable & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX1276_SPIWrite(module,
						LR_RegModemConfig2,
						((SX1276_SpreadFactor[LoRa_Rate] << 4) + (SX1276_CRC << 2) + 0x03)
						);//CRC Enable & SymbTimeout>=768

		tmp = SX1276_SPIRead(module, 0x31);
		tmp &= 0xF8;
		tmp |= 0x05;
		SX1276_SPIWrite(module, 0x31, tmp); //Detection Optimize
		SX1276_SPIWrite(module, 0x37, 0x0C); //Detection Threshold
	} else {
		SX1276_SPIWrite(module,
						LR_RegModemConfig1,
						((SX1276_LoRaBandwidth[LoRa_BW] << 4) + (SX1276_CR << 1) + 0x00)
						); //Explicit Enable & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

//		SX1276_SPIWrite(module,
//						LR_RegModemConfig2,
//						((SX1276_SpreadFactor[LoRa_Rate] << 4) + (1 << 3) + (SX1276_CRC << 2) + 0x03)
//						);
		//wrong setting

		//CRC Enable(0x02) & TxContinuousMode & SymbTimeout>=768
		SX1276_SPIWrite(module,
						LR_RegModemConfig2,
						((SX1276_SpreadFactor[LoRa_Rate] << 4) + (SX1276_CRC << 2) + 0x03)
						);
		//SFactor &  LNA gain set by the internal AGC loop
	}

	//Common Process//
	SX1276_SPIWrite(module, LR_RegSymbTimeoutLsb, 0xFF); //RegSymbTimeoutLsb Timeout = 0x3FF(Max)
	SX1276_SPIWrite(module, LR_RegPreambleMsb, 0x00); //RegPreambleMsb
	SX1276_SPIWrite(module, LR_RegPreambleLsb, 12); //RegPreambleLsb 8+4=12symbols Preamble
	SX1276_SPIWrite(module, REG_LR_DIOMAPPING2, 0x01); //RegDioMapping2 DIO5=00, DIO4=01
	module->readBytes = 0;

	SX1276_standby(module); //Entry standby mode
}

void SX1276_standby(SX1276_t * module) {
	SX1276_SPIWrite(module, LR_RegOpMode, 0x01);
	module->status = STANDBY;
}

void SX1276_sleep(SX1276_t * module) {
	SX1276_SPIWrite(module, LR_RegOpMode, 0x00);
	module->status = SLEEP;
}

//void SX1276_cad(SX1276_t * module) {
//	SX1276_SPIWrite(module, LR_RegOpMode, 0x07);
//	module->status = CAD;
//}

void SX1276_entryLoRa(SX1276_t * module) {
	SX1276_SPIWrite(module, LR_RegOpMode, 0x80);
	//set LongRangeMode to 1
}
//0x88(LowFrequency LoRa)
//0x80(HighFrequency LoRa)

void SX1276_clearLoRaIrq(SX1276_t * module) {
	SX1276_SPIWrite(module, LR_RegIrqFlags, 0xFF);
}

int SX1276_LoRaEntryRxtest(SX1276_t * module, uint8_t length, uint32_t timeout) {
	uint8_t addr = 0;

	module->packetLength = length;

	SX1276_defaultConfig(module); //Setting base parameter
	SX1276_SPIWrite(module, REG_LR_PADAC, 0x84); //Default and RX
	SX1276_SPIWrite(module, LR_RegHopPeriod, 0xFF); //No FHSS
	SX1276_SPIWrite(module, REG_LR_DIOMAPPING1, 0x01); //DIO=00,DIO1=00,DIO2=00, DIO3=01
	SX1276_SPIWrite(module, LR_RegIrqFlagsMask, 0x3F); //Open RxDone interrupt & Timeout
	SX1276_clearLoRaIrq(module);
	SX1276_SPIWrite(module, LR_RegPayloadLength, length);
	//Payload Length 21byte(this register must define when the data long of one byte in SF is 6)

	SX1276_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RxBaseAddr->FiFoAddrPtr
	SX1276_SPIWrite(module, LR_RegOpMode, 0x85); //Mode HighFrequency Continuous Rx Mode
	//SX1276_SPIWrite(module, LR_RegOpMode, 0x8d);		//Continuous Rx Mode //Low Frequency Mode
	//SX1276_SPIWrite(module, LR_RegOpMode, 0x05);		//Continuous Rx Mode //High Frequency Mode

	module->readBytes = 0;

	while (1) {
		if ((SX1276_SPIRead(module, LR_RegModemStat) & 0x04) == 0x04) {
			//Rx-on going RegModemStat
			//more details refer to page111
			module->status = RX;
			return 1;
		}
		if (--timeout == 0) {
			SX1276_hw_Reset(module->hw);
			SX1276_defaultConfig(module);
			return 0;
		}
		SX1276_hw_DelayMs(1);
	}
}
int SX1276_LoRaEntryCad(SX1276_t * module) {
	SX1276_defaultConfig(module); //Setting base parameter
	SX1276_SPIWrite(module, REG_LR_PADAC, 0x84); //Default and RX
	SX1276_SPIWrite(module, LR_RegHopPeriod, 0xFF); //No FHSS
	SX1276_SPIWrite(module, REG_LR_DIOMAPPING1, 0xA9); //10101001
													   //DIO0=10 CadDone
													   //DIO1=10 CadDetected
													   //DIO2=10 FHSSChangeChannel
	    											   //DIO3=01 ValidHeader

	SX1276_SPIWrite(module, LR_RegIrqFlagsMask, 0xFE); //Open CadDone, FHSSChangeChannel & CadDetected
	SX1276_clearLoRaIrq(module);

	SX1276_SPIWrite(module, LR_RegOpMode, 0x87); //Mode HighFrequency CAD Mode
	return 1;
}
int SX1276_LoRaCadNOT(SX1276_t * module, uint32_t timeout) {
	while (1) {
		if ( !SX1276_hw_GetDIO1(module->hw) ) { //if NOT CadDetected

			//need?
			//SX1276_SPIRead(module, LR_RegIrqFlags);

			SX1276_hw_DelayMs(1);
			timeout--;
//			memset(Data, 0, 256);
//			size = sprintf((char *)Data, "%02X", SX1276_SPIRead(module, LR_RegIrqFlags));
//			HAL_UART_Transmit(&huart2, Data, size, 1000);

			if (timeout == 0) {
				//SX1276_hw_Reset(module->hw);
				//SX1276_defaultConfig(module);
				return 1;
			}

			continue;
		} else {
			SX1276_clearLoRaIrq(module); //Clear IRQ
			return 0;
		}
	}
}

int SX1276_LoRaCadDetected(SX1276_t * module, uint32_t timeout) {
	while (1) {
		if ( SX1276_hw_GetDIO1(module->hw) ) { //if CadDetected

			//need?
			SX1276_SPIRead(module, LR_RegIrqFlags);
//			memset(Data, 0, 256);
//			size = sprintf((char *)Data, "%02X", SX1276_SPIRead(module, LR_RegIrqFlags));
//			HAL_UART_Transmit(&huart2, Data, size, 1000);
			//SX1276_hw_DelayMs(100);
			timeout--;

			if (--timeout == 0) {
				SX1276_hw_Reset(module->hw);
				SX1276_defaultConfig(module);
				return 1;
			}
			continue;
		} else {
			HAL_Delay(1);
			return 0;
		}
	}
}


int SX1276_LoRaEntryRx(SX1276_t * module, uint8_t length, uint32_t timeout) {
	uint8_t addr;
	module->packetLength = length;

	SX1276_defaultConfig(module); //Setting base parameter
	SX1276_SPIWrite(module, REG_LR_PADAC, 0x84); //Default and RX
	SX1276_SPIWrite(module, LR_RegHopPeriod, 0xFF); //No FHSS
	SX1276_SPIWrite(module, REG_LR_DIOMAPPING1, 0x01); //DIO=00,DIO1=00,DIO2=00, DIO3=01

	SX1276_SPIWrite(module, LR_RegIrqFlagsMask, 0x1F); //Open RxDone interrupt & Timeout, Open PayloadCrcError
	SX1276_clearLoRaIrq(module);

	SX1276_SPIWrite(module, LR_RegPayloadLength, length);
	//Payload Length 21byte(this register must define when the data long of one byte in SF is 6)

	addr = SX1276_SPIRead(module, LR_RegFifoRxBaseAddr); //Read RxBaseAddr
	SX1276_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RxBaseAddr->FiFoAddrPtr
	SX1276_SPIWrite(module, LR_RegOpMode, 0x85); //Mode HighFrequency Continuous Rx Mode
	//SX1276_SPIWrite(module, LR_RegOpMode, 0x8d);		//Continuous Rx Mode //Low Frequency Mode
	//SX1276_SPIWrite(module, LR_RegOpMode, 0x05);		//Continuous Rx Mode //High Frequency Mode

	module->readBytes = 0;

	while (1) {
		if ( (SX1276_SPIRead(module, LR_RegModemStat) & 0x04) == 0x04 ) {
			//Rx-on going RegModemStat
			//more details refer to page111

			module->status = RX;

			uint8_t PayloadCrcError = (SX1276_SPIRead(module,  LR_RegIrqFlags) & 0x20) >> 5;
			if( PayloadCrcError == 1 )return 0; //CHECK CRC ERROR
			return 1;
		}
		if (--timeout == 0) {
			SX1276_hw_Reset(module->hw);
			SX1276_defaultConfig(module);
			return 0;
		}
		SX1276_hw_DelayMs(1);
	}
}

uint8_t SX1276_LoRaRxPacket(SX1276_t * module) {
	unsigned char addr;
	unsigned char packet_size;

	uint8_t PayloadCrcError = (SX1276_SPIRead(module,  LR_RegIrqFlags) & 0x20) >> 5;

	if ( SX1276_hw_GetDIO0(module->hw) && !PayloadCrcError ) {	//check RxDone && PayloadCrcError
		memset(module->rxBuffer, 0x00, SX1276_MAX_PACKET);
		addr = SX1276_SPIRead(module, LR_RegFifoRxCurrentaddr);	//Start address of last packet received
		SX1276_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RxBaseAddr -> FiFoAddrPtr

		if ( module->LoRa_Rate == SX1276_LORA_SF_6 ) {
			//When SpreadFactor is 6, will used Implicit Header mode(Excluding internal packet length)
			packet_size = module->packetLength;
		} else {
			packet_size = SX1276_SPIRead(module, LR_RegRxNbBytes); //Number for received bytes
		}

		SX1276_SPIBurstRead(module, 0x00, module->rxBuffer, packet_size);
		module->readBytes = packet_size;
		//SX1276_clearLoRaIrq(module);
	}
	SX1276_clearLoRaIrq(module);
	//when module->readBytes is 0, function error
	return module->readBytes;
}


int SX1276_LoRaEntryTx(SX1276_t * module, uint8_t length, uint32_t timeout) {
	uint8_t addr;
	uint8_t temp;

	module->packetLength = length;
	SX1276_defaultConfig(module); //setting base parameter
	SX1276_SPIWrite(module, REG_LR_PADAC, 0x87); //Tx +20dBm on PA_BOOST when OutputPower=1111
	SX1276_SPIWrite(module, LR_RegHopPeriod, 0x00); //RegHopPeriod NO FHSS
	SX1276_SPIWrite(module, REG_LR_DIOMAPPING1, 0x41); //DIO0=01, DIO1=00, DIO2=00, DIO3=01
	SX1276_clearLoRaIrq(module);
	SX1276_SPIWrite(module, LR_RegIrqFlagsMask, 0xF7); //Open TxDone interrupt
	SX1276_SPIWrite(module, LR_RegPayloadLength, length); //PayloadLength
	addr = SX1276_SPIRead(module, LR_RegFifoTxBaseAddr); //FiFoTxBaseAddr
	SX1276_SPIWrite(module, LR_RegFifoAddrPtr, addr);  //FiFoTxBaseAddr -> RegFifoAddrPtr
	while (1) {
		temp = SX1276_SPIRead(module, LR_RegPayloadLength);
		if (temp == length) {
			module->status = TX;
			return 1;
		}

		if (--timeout == 0) {
			SX1276_hw_Reset(module->hw);
			SX1276_defaultConfig(module);
			return 0;
		}
	}
}

int SX1276_LoRaTxPacket(SX1276_t * module, uint8_t* txBuffer, uint8_t length, uint32_t timeout) {
	SX1276_SPIBurstWrite(module, 0x00, txBuffer, length);
	SX1276_SPIWrite(module, LR_RegOpMode, 0x83);	//Tx Mode //0x8b = Low
														      //0x83 = High
	while (1) {
		if (SX1276_hw_GetDIO0(module->hw)) { //if(Get_NIRQ()) //Packet send over
			SX1276_SPIRead(module, LR_RegIrqFlags);
			SX1276_clearLoRaIrq(module); //Clear irq
			SX1276_standby(module); //Entry Standby mode
			return 1;
		}

		if (--timeout == 0) {
			SX1276_hw_Reset(module->hw);
			SX1276_defaultConfig(module);
			return 0;
		}
		SX1276_hw_DelayMs(1);
	}
}

void SX1276_begin(SX1276_t * module, uint8_t frequency, uint8_t power,
		uint8_t LoRa_Rate, uint8_t LoRa_BW, uint8_t packetLength) {
	SX1276_hw_init(module->hw);
	module->frequency = frequency;
	module->power = power;
	module->LoRa_Rate = LoRa_Rate;
	module->LoRa_BW = LoRa_BW;
	module->packetLength = packetLength;
	SX1276_defaultConfig(module);
}

int SX1276_receive(SX1276_t * module, uint8_t length, uint32_t timeout) {
	return SX1276_LoRaEntryRx(module, length, timeout);
}

uint8_t SX1276_available(SX1276_t * module) {
	return SX1276_LoRaRxPacket(module);
}

uint8_t SX1276_read(SX1276_t * module, uint8_t* rxBuf, uint8_t length) {
	if (length != module->readBytes)
		length = module->readBytes;
	memcpy(rxBuf, module->rxBuffer, length);
	rxBuf[length] = '\0';
	module->readBytes = 0;
	return length;
}

int SX1276_RSSI_LoRa(SX1276_t * module) {
//	uint32_t rssi = 10;
	int rssi = SX1276_SPIRead(module, LR_RegRssiValue);	//RegRssiValue Current RSSI
//	rssi = rssi + 127 - 137;		//127:Max RSSI, 137:RSSI offset
	return rssi;
}

int SX1276_PacketStrength(SX1276_t * module) {
	uint8_t pkt_rssi = SX1276_SPIRead(module, LR_RegPktRssiValue);	//RegPktRssiValue (RSSI of last packet)
	uint8_t pkt_snr = SX1276_SPIRead(module, LR_RegPktSnrValue); 	//Read RegPktSnrValue
	int rssi = -164 + pkt_rssi + pkt_snr * 0.25; //compute the signal strength of the received packet
	return rssi;
}

uint8_t SX1276_RSSI(SX1276_t * module) {
	uint8_t rssi = 0xff;
	rssi = SX1276_SPIRead(module, 0x11);
	rssi = 127 - (rssi >> 1);	//127:Max RSSI
	return rssi;
}
