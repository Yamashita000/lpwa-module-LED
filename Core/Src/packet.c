/*
 * packet.c
 *
 *  Created on: 2021/06/08
 *      Author: kazuaki
 */

#include <string.h>
#include "packet.h"
#include "console.h"
#include "eeprom.h"


extern int DebugEnable;

// 送受信用のFIFO
static struct PACKET_FIFO rx_fifo;
static struct PACKET_FIFO tx_fifo;
// メッセージ保存用のFIFO
static struct PACKET_FIFO msg_fifo;


void PacketInit(void){
	// FIFOを初期化する
	rx_fifo.pos = 0;
	tx_fifo.pos = 0;
	msg_fifo.pos = 0;
}

// CMD 00 初期化
//
static void packet_initialize(uint8_t packet[], int packet_size){
	uint32_t my_id = ConfigReadDeviceID();
	uint8_t my_area = (my_id >> 16) & 0xff;
	// HQであれば何もしない
	if( (my_id & 0xffff) == 0 ) return;
	// areaをチェックする
	uint8_t area = packet[1];
	if( area != 0 && area != my_area ){
		if( DebugEnable ){
			SerialOutput("[DEBUG] No Area (area=%02x)\r\n", area);
		}
		return;  // エリアが違うので無視する
	}
	// Uplinkを設定する
	uint32_t uplink_id = (packet[1] << 16) | (packet[2] << 8) | packet[3];
	// すでにUPLINKが設定済みならば処理なし
	if( uplink_id == ConfigReadUplinkID() ) return;
	// UPLINKが自分ならば処理なし
	if( uplink_id == my_id ) return;
	// UPLINKを設定する
	ConfigWriteUplinkID(uplink_id);
	// ダウンリンクを継続する
	// senderIDを書き換える
	// packet[1] = (my_id >> 16) & 0xff;  // areaは書き換えない
	packet[2] = (my_id >> 8) & 0xff;
	packet[3] = (my_id) & 0xff;
	if( DebugEnable ){
		SerialOutput("[DEBUG] Repeat Initialize\r\n");
	}
	PacketEnqueue(PACKET_TX_FIFO, packet, packet_size);
}


// CMD 01 一斉配信
// CMD 03 ID指定したメッセージ送信（ダウン側）
// パケットのsender情報を書き換えて再送信する
static void packet_downlink(enum PACKET_CMD_TYPE cmd, uint8_t packet[], int packet_size){
	uint32_t my_id = ConfigReadDeviceID();
	uint8_t my_area = (my_id >> 16) & 0xff;
	// areaをチェックする
	uint8_t area = packet[1];
	if( area != 0 && area != my_area ){
		if( DebugEnable ){
			SerialOutput("[DEBUG] No Area (area=%02x)\r\n", area);
		}
		return;  // エリアが違うので無視する
	}
	// seq番号をチェックする
	// 過去の seq であれば無視する
	uint16_t packet_seq = packet[4] << 8 | packet[5];
	if( ConfigReadSeq() > 0 && packet_seq <= ConfigReadSeq() ){
		if( DebugEnable ){
			SerialOutput("[DEBUG] No Repeat (seq=%d)\r\n", packet_seq);
		}
		return;
	}
	ConfigWriteSeq(packet_seq);

	// BROADCASTの場合は、senderIDを保存する
	if (cmd == CMD_BROADCAST) {
		uint32_t uplink_id = (packet[1] << 16) | (packet[2] << 8) | packet[3];
		ConfigWriteUplinkID(uplink_id);
		// senderIDを書き換える
		// packet[1] = (my_id >> 16) & 0xff;  // areaは書き換えない
		packet[2] = (my_id >> 8) & 0xff;
		packet[3] = (my_id) & 0xff;
	}

	// パケットの処理
	if( cmd == CMD_BROADCAST ){
		if( DebugEnable ){
			SerialOutput("[DEBUG] Repeat (seq=%d)\r\n", packet_seq);
		}
		PacketEnqueue(PACKET_TX_FIFO, packet, packet_size);
		// 受信メッセージを保存する
		PacketEnqueue(MESSAGE_FIFO, packet, packet_size);
	} else	if( cmd == CMD_SENDTO_1 ){
		uint32_t receiver_id = (packet[6] << 16) | (packet[7] << 8) | packet[8];
		if( receiver_id == my_id ){
			// 受信メッセージを保存する
			if( DebugEnable ){
				SerialOutput("[DEBUG] Receive (seq=%d)\r\n", packet_seq);
			}
			PacketEnqueue(MESSAGE_FIFO, packet, packet_size);
		} else {
			if( DebugEnable ){
				SerialOutput("[DEBUG] Repeat (seq=%d)\r\n", packet_seq);
			}
			PacketEnqueue(PACKET_TX_FIFO, packet, packet_size);
		}
	}
}


// CMD 02 アップリンク
// HQ：データを受信バッファに入れる
// HQ以外：送信先をアップリンク側として、データを再送信する
static void packet_uplink(uint8_t packet[], int packet_size){
	uint32_t my_id = ConfigReadDeviceID();
	uint32_t packet_uplink_id = (packet[1]<<16) | (packet[2]<<8) | packet[3];
	// 自分が受信すべきかどうかを判断する
	if( packet_uplink_id != my_id ) return;
	// HQかどうか
	if( (my_id & 0x00FFFF) == 0 ){
		// HQの場合
		if( DebugEnable ){
			SerialOutput("[DEBUG] Uplink, receive message\r\n");
		}
		PacketEnqueue(MESSAGE_FIFO, packet, packet_size);
	} else {
		// HQ以外の場合
	    uint32_t uplink_id = ConfigReadUplinkID();
		// packet[1] = (uplink_id >> 16) & 0xff;  // areaは書き換えない
		packet[2] = (uplink_id >> 8 ) & 0xff;
		packet[3] = (uplink_id      ) & 0xff;
		if( DebugEnable ){
			SerialOutput("[DEBUG] Uplink, repeat message\r\n");
		}
		PacketEnqueue(PACKET_TX_FIFO, packet, packet_size);
	}
}


// CMD 04 メッセージ送信
// case-1: 自分がHQで、宛先がHQならば、受信処理
// case-2: 自分がHQで、宛先がHQでなければ、ダウンリンク処理に変換する
// case-3: 自分がHQでなければ、 uplink と同じ処理
static void packet_sendto2(uint8_t packet[], int packet_size) {
	if ((ConfigReadDeviceID() & 0x00FFFF) == 0) {
		// HQの処理
		uint32_t my_id = ConfigReadDeviceID();
		uint32_t receiver_id = (packet[4] << 16) | (packet[5] << 8) | packet[6];
		if (receiver_id == my_id) {
			// case-1: 受信する
			PacketEnqueue(MESSAGE_FIFO, packet, packet_size);
		} else {
			// case-2: ダウンリンクに変換する
			// パケットを再構成する
			if( DebugEnable ){
				SerialOutput("[DEBUG] sendto, re-build packet\r\n");
			}
			// CMD=0x03 で再送信する
			// cmd
			packet[0] = 0x03;
			// receiver id
			memcpy(packet + 1, packet + 4, 3);
			// Seq
			uint16_t seq = ConfigReadSeq();
			packet[4] = (seq >> 8) & 0xff;
			packet[5] = (seq) & 0xff;
			seq++;
			ConfigWriteSeq(seq);
			// sender id
			memcpy(packet + 6, packet + 7, 3);
			// payload
			memcpy(packet + 9, packet + 10, packet[9]);
			// 送信する
			// コマンドが変わるのでパケットサイズも変わる
			PacketEnqueue(PACKET_TX_FIFO, packet, packet_size);
		}
	} else {
		// case-3: HQ以外の処理
	}
}


// パケットを解析して、それぞれの処理を実行する
void PacketExecute(uint8_t packet[], int packet_size) {
	uint8_t cmd = packet[0];
	// debug
	if( DebugEnable ){
		int i;
		SerialOutput("[DEBUG] Rx packet: ");
		for( i=0 ; i<packet_size ; i++ ){
			SerialOutput("%02x ", packet[i]);
		}
		SerialOutput("\r\n");
	}
	// dispatch
	switch(cmd){
	case 0x00:
		packet_initialize(packet, packet_size);
		break;
	case 0x01:
		packet_downlink(CMD_BROADCAST, packet, packet_size);
		break;
	case 0x02:
		packet_uplink(packet, packet_size);
		break;
	case 0x03:
		packet_downlink(CMD_SENDTO_1, packet, packet_size);
		break;
	case 0x04:
		packet_sendto2(packet, packet_size);
		break;
	default:
		SerialOutput("[DEBUG] Packet error (%02x).\r\n", cmd);
		break;
	}
}


static struct PACKET_FIFO *get_pfifo(enum PACKET_FIFO_TYPE t)
{
	switch( t ){
	case PACKET_RX_FIFO:
		return &rx_fifo;
	case PACKET_TX_FIFO:
		return &tx_fifo;
	case MESSAGE_FIFO:
		return &msg_fifo;
	default:
		return NULL;
	}
}

// 送受信バッファのデータ数を返す
int PacketSize(enum PACKET_FIFO_TYPE t) {
	struct PACKET_FIFO *fifo = get_pfifo(t);
	if (fifo == NULL) return 0;

	return fifo->pos;
}


// 送受信バッファにパケットを追加する
void PacketEnqueue(enum PACKET_FIFO_TYPE t, uint8_t packet[], int packet_size)
{
	struct PACKET_FIFO *fifo = get_pfifo(t);
	if( fifo == NULL ) return;

	// Enqできなければ、古いメッセージを消去する
	while( fifo->pos + packet_size > PACKET_FIFO_SIZE ){
		if( DebugEnable ){
			SerialOutput("[DEBUG] delete a queue item\r\n");
		}

		int len = fifo->data[0];
		int src = len+1;
		int dst = 0;
		while( src < PACKET_FIFO_SIZE ){
			fifo->data[dst++] = fifo->data[src++];
		}
		fifo->pos -= len+1;
	}

	// copy
	int src = 0;
	int dst = fifo->pos;
	int len = packet_size;
	fifo->data[dst++] = len;
	while( len > 0 ){
		fifo->data[dst++] = packet[src++];
		len--;
	}
	fifo->pos += packet_size+1;
}

// 送受信バッファからパケットを取り出す
// パケットがなければ 0 を返す
int PacketDequeue(enum PACKET_FIFO_TYPE t, uint8_t packet[])
{
	struct PACKET_FIFO *fifo = get_pfifo(t);
	if( fifo == NULL ) return 0;
	if( fifo->pos <= 0 ) return 0;
	// copy and move
	uint8_t src = 0;
	uint8_t dst = 0;

	// copy fifo to packet[]
	int len = fifo->data[src++];
	int packet_len = len;
	while( len > 0 ){
		packet[dst++] = fifo->data[src++];
		len--;
	}

	// move fifo
	dst = 0;
	fifo->pos -= packet_len + 1;
	len = fifo->pos;
	while( len > 0 ){
		fifo->data[dst++] = fifo->data[src++];
		len--;
	}
	return packet_len;
}


