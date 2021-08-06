/*
 * packet.h
 *
 *  Created on: 2021/06/08
 *      Author: kazuaki
 */

#ifndef INC_PACKET_H_
#define INC_PACKET_H_

#include <stdio.h>

enum PACKET_CMD_TYPE {
	CMD_NOP        = 0x00,
	CMD_BROADCAST  = 0x01,
	CMD_UPLINK     = 0x02,
	CMD_SENDTO_1   = 0x03,
	CMD_SENDTO_2   = 0x03,
	CMD_INITIALIZE = 0x7F,
};


enum PACKET_FIFO_TYPE {
	PACKET_RX_FIFO,
	PACKET_TX_FIFO,
	MESSAGE_FIFO,
};


// パケットの送受信用バッファ
// [len][packet data] の繰り返し
//   [len] は、パケット長さ（コマンドなどすべてを含む長さ）
//   [packet data] は、LoRaで送信される内容
#define PACKET_FIFO_SIZE 512
struct PACKET_FIFO {
	int pos;
	uint8_t data[512];
};

void PacketInit(void);
void PacketExecute(uint8_t packet[], int packet_size);

int PacketSize(enum PACKET_FIFO_TYPE t);
void PacketEnqueue(enum PACKET_FIFO_TYPE, uint8_t packet[], int packet_size);
int PacketDequeue(enum PACKET_FIFO_TYPE, uint8_t packet[]);


#endif /* INC_PACKET_H_ */
