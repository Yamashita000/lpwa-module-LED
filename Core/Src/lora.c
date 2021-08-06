/*
 * lora.c
 *
 *  Created on: 2021/06/04
 *      Author: kazuaki
 */

#include "main.h"
#include "SX1276.h"
#include "stm32l0xx_hal.h"

#define RADIO_DIO_0_Pin GPIO_PIN_4
#define RADIO_DIO_1_Pin GPIO_PIN_1
#define RADIO_DIO_2_Pin GPIO_PIN_0
#define RADIO_NSS_Pin GPIO_PIN_15
#define RADIO_RESET_Pin GPIO_PIN_0

void LoRaHwInit(SX1276_hw_t *hw, SX1276_t *module) {
	hw->dio0.port = GPIOB;
	hw->dio0.pin = RADIO_DIO_0_Pin;
	hw->dio1.port = GPIOB;
	hw->dio1.pin = RADIO_DIO_1_Pin;
	hw->dio2.port = GPIOB;
	hw->dio2.pin = RADIO_DIO_2_Pin;
	hw->nss.port = GPIOA;
	hw->nss.pin = RADIO_NSS_Pin;
	hw->reset.port = GPIOC;
	hw->reset.pin = RADIO_RESET_Pin;
	hw->spi = &hspi1;
	module->hw = hw;
}

void LoRaSetup(SX1276_t *module, int sf, int bw) {

	int error = 1;
	if ((0 <= sf) && (sf <= 7)) {
		if ((0 <= bw) && (bw <= 9)) {
			error = 0;
		}
	}
	if (error == 0) {
		SX1276_begin(module, SX1276_923MHZ, SX1276_POWER_13DBM, sf, bw, 255);
		return;
	}
	SX1276_begin(module, SX1276_923MHZ, SX1276_POWER_13DBM, SX1276_LORA_SF_10,
			SX1276_LORA_BW_125KHZ, 255);
}

void LoRaSendMessage(SX1276_t *module, uint8_t *txBuffer, uint8_t txBufferSize) {
	LEDCounterTx = 3;
	SX1276_LoRaEntryTx(module, txBufferSize, 1000);
	SX1276_SPIBurstWrite(module, 0x00, txBuffer, txBufferSize);
	SX1276_SPIWrite(module, LR_RegOpMode, 0x83);
	int timeout = 3000;
	while (1) {
		if (SX1276_hw_GetDIO0(module->hw)) {
			SX1276_SPIRead(module, LR_RegIrqFlags);
			SX1276_clearLoRaIrq(module);
			SX1276_standby(module);
			break;
		}
		if (--timeout == 0) {
			SX1276_hw_Reset(module->hw);
			SX1276_defaultConfig(module);
			break;
		}
		SX1276_hw_DelayMs(1);
	}
	SX1276_hw_DelayMs(40);
	SX1276_LoRaEntryRx(module, 255, 2000);
}


int LoRaReceiveMessage(SX1276_t *module, uint8_t *rxBuffer){
	uint8_t PayloadCrcError = (SX1276_SPIRead(module, LR_RegIrqFlags) & 0x20) >> 5;
	//CHECK "RxDone" && "PayloadCrcError"
	if (SX1276_hw_GetDIO0(module->hw) && !PayloadCrcError) {
		LEDCounterRx = 3;
		uint8_t rx_addr;
		int msg_size;

	    memset(module->rxBuffer, 0x00, SX1276_MAX_PACKET);
	    rx_addr = SX1276_SPIRead(module, LR_RegFifoRxCurrentaddr);
	    SX1276_SPIWrite(module, LR_RegFifoAddrPtr, rx_addr);
	    //WHEN SF = 6, EXPLICT HEADER MODE
	    if ( module->LoRa_Rate == SX1276_LORA_SF_6 ) {
	    	msg_size = module->packetLength;
	    } else {
	       msg_size = SX1276_SPIRead(module, LR_RegRxNbBytes); //Number for received bytes
	    }
	    SX1276_SPIBurstRead(module, 0x00, module->rxBuffer, msg_size);
	    module->readBytes = msg_size;
	    SX1276_clearLoRaIrq(module);

	    memcpy(rxBuffer, module->rxBuffer, msg_size+1);

	    return msg_size;
	} else {
		module->readBytes = 0;
       SX1276_hw_DelayMs(10);
       SX1276_clearLoRaIrq(module);

		return 0;
	}
}

