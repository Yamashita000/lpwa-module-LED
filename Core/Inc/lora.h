/*
 * lora.h
 *
 *  Created on: 2021/06/04
 *      Author: kazuaki
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_

#include "SX1276.h"

void LoRaHwInit(SX1276_hw_t *hw, SX1276_t *module);
void LoRaSetup(SX1276_t *module, int sf, int bw);
void LoRaSendMessage(SX1276_t *module, uint8_t *txBuffer, uint8_t txBufferSize);
int LoRaReceiveMessage(SX1276_t *module, uint8_t *rxBuffer);

#endif /* INC_LORA_H_ */
