/*
 * command.c
 *
 *  Created on: 2021/06/04
 *      Author: kazuaki
 */

#include <ctype.h>
#include <string.h>

#include "main.h"
#include "console.h"
#include "eeprom.h"
#include "packet.h"


// extern SX1276_t SX1276;

// s1 が s2 で始まるならば 1 を返す
// 大文字小文字を区別しない
static int check_substr(const uint8_t *s1, const char *s2){
	while( *s2 ){
		const char c1 = *s1++;
		const char c2 = *s2++;
		if( toupper(c1) != toupper(c2) ) return 0;
	}
	return 1;
}


static int command_help(void){
	const char *msg[] = {
			"Use AT command",
			"  ATB<msg>     : send <msg> to all devices (HQ)",
			"  ATD<f>       : f=0 disable debug mode",
			"               : f=1 enable debug mode",
			"  ATF<F>=<f>   : f=0 disable function F",
			"               : f=1 enable function F",
			"  ATG          : get message",
			"  ATI          : show config",
			"  ATS<id><msg> : send <msg> to device <id>",
			"  ATU<msg>     : send uplink message",
			"  ATZ          : initialize whole system (HQ)",
			"  AT/<k>=<v>   : set config key:<k>, value=<v>",
			"  AT/<k>       : get config key:<k>",
			"",
			NULL,
	};
	const char **p = msg;
	while( *p ){
		SerialOutput("[HELP] %s\r\n", *p);
		p++;
	}
	return 0;
}

// （管理者用コマンド）EEPROMを初期化する
static int command_cleareeprom(void){
	ConfigClearEEPROM();
	SerialOutput("[ADMIN] EEPROM cleared\r\n");
	return 0;
}

// （管理者用コマンド）送受信バッファをクリアする
static int command_clearbuffer(void){
	PacketInit();
	SerialOutput("[ADMIN] FIFO cleared\r\n");
	return 0;
}

// システムの初期化(HQ)
// 空のBROADCASTを行い、SEQとUPLINK情報を初期化する
static int command_initialize(void){
	if( (ConfigReadDeviceID() & 0x0000FFFF) != 0 ){
		return -1;
	}
	uint8_t tx_buffer[256];
	// Build header
	tx_buffer[0] = 0x00;   // CMD=INITIALIZE
	// SenderID
	uint32_t id = ConfigReadDeviceID();
	tx_buffer[1] = (id >> 16) & 0xff;
	tx_buffer[2] = (id >> 8 ) & 0xff;
	tx_buffer[3] = (id      ) & 0xff;
	// TX
	PacketEnqueue(PACKET_TX_FIFO, tx_buffer, 4);
	// SEQを初期化する
	ConfigWriteSeq(0);
	return 1;
}


static int command_info(void){
    uint8_t product[9], version[9];
    uint32_t my_id = ConfigReadDeviceID();
    uint32_t uplink_id = ConfigReadUplinkID();
    ConfigReadVersion(product, version);
    product[8] = 0;
    version[8] = 0;
    // 文字列作成
    SerialOutput("Product: %s\r\n", product);
    SerialOutput("Version: %s\r\n", version);
    SerialOutput("DeviceID: %02X %04X %s\r\n", my_id>>16, my_id&0xffff, (my_id&0xffff)?"":"(HQ)");
    SerialOutput("UplinkID: %02X %04X %s\r\n", uplink_id>>16, uplink_id&0xffff, (uplink_id&0xffff)?"":"(HQ)");
    SerialOutput("Seq: %d\r\n", ConfigReadSeq());
	return 0;
}

static int command_getdata(void){
	int message_size;
	uint8_t message[256];
	message_size = PacketDequeue(MESSAGE_FIFO, message) - 1;   // remove [len]
	// メッセージ無し
	if( message_size <= 0 ){
		SerialOutput("0\r\n");
		return 0;
	}
	// メッセージ取得
	// CMD
	int i;
	uint8_t cmd = message[0];
	SerialOutput("%02X", message[0]);
	// コマンドごとの出力
	switch( cmd ){
	case 0x01:  // BROADCAST
	case 0x03:  // SENDTO1
		SerialOutput("%02X%02X%02X", message[6], message[7], message[8]);
		for( i=9 ; i<message_size ; i++ ){
			SerialOutput("%c", message[i+1]);
		}
		SerialOutput("\r\n");
		return 0;
	case 0x02:  // UPLINK
		SerialOutput("%02X%02X%02X", message[4], message[5], message[6]);
		for( i=7 ; i<message_size ; i++ ){
			SerialOutput("%c", message[i+1]);
		}
		SerialOutput("\r\n");
		return 0;
	case 0x04:  // SENDTO2
		SerialOutput("%02X%02X%02X", message[7], message[8], message[9]);
		for( i=10 ; i<message_size ; i++ ){
			SerialOutput("%c", message[i+1]);
		}
		SerialOutput("\r\n");
		return 0;
	default:    // Error
		SerialOutput("*\r\n");
		return -1;
	}
}


extern int DebugEnable;
static int command_debug(int debug_enable){
	DebugEnable = debug_enable;
	SerialOutput("DEBUG MODE O%s\r\n", debug_enable?"N":"FF");
	return 0;
}




static int command_settings(const uint8_t *param){
	char *value = strchr((char *)param, '=');
	int ret = 0;
	int isSetValue = (value!=NULL);
	if( isSetValue ) value++;
	if( check_substr(param, "ID") ){
		// Device ID
		int id;
		if( isSetValue ){
			sscanf(value, "%x", &id);
			ConfigWriteDeviceID(id);
			ret = 1;
		} else {
			id = ConfigReadDeviceID();
			SerialOutput("%06X\r\n", id);
			ret = 0;
		}
	} else	if( check_substr(param, "UP") ){
		uint32_t id;
		if( isSetValue ){
			sscanf(value, "%lx", &id);
			ConfigWriteUplinkID(id);
			ret = 1;
		} else {
			id = ConfigReadUplinkID();
			SerialOutput("%06X\r\n", id);
			ret = 0;
		}
	} else	if( check_substr(param, "SEQ") ){
		// Seq
		int seq;
		if( isSetValue ){
			sscanf(value, "%x", &seq);
			ConfigWriteSeq(seq);
			ret = 1;
		} else {
			seq = ConfigReadSeq();
			SerialOutput("0x%04x (%d)\r\n", seq, seq);
			ret = 0;
		}

	} else {
		ret = -1;
	}
	return ret;
}

static int command_function(const uint8_t *param){
	return 1;
}

static uint8_t Hex1toNum1(uint8_t c)
{
	if( c>='0' && c<='9' ){
		return c - '0';
	} else if( c>='A' && c<='F' ){
		return c - 'A' + 10;
	} else if( c>='a' && c<='f' ){
		return c - 'a' + 10;
	}
	return 0;
}

static int expand_message(uint8_t *dst, const uint8_t *src)
{
	uint8_t *start = dst;
	int isRawString = 0;
	while( *src ){
		if( *src == '\"' ){
			isRawString = !isRawString;
			src++;
		} else {
			if( isRawString ){
				*dst++ = *src++;
			} else {
				uint8_t c = Hex1toNum1(*src++);
				if( *src == 0 ) return -1;
				*dst++ = (c << 4) + Hex1toNum1(*src++);
			}
		}
	}
	*dst++ = 0;
	return dst - start - 1;
}

static int command_broadcast(const uint8_t *param){
	if( (ConfigReadDeviceID() & 0x0000FFFF) != 0 ){
		return -1;
	}
	uint8_t tx_buffer[256];
	// Build header
	tx_buffer[0] = 0x01;   // CMD=BROADCAST
	// SenderID
	uint32_t id = ConfigReadDeviceID();
	tx_buffer[1] = (id >> 16) & 0xff;
	tx_buffer[2] = (id >> 8 ) & 0xff;
	tx_buffer[3] = (id      ) & 0xff;
	// Seq
	uint16_t seq = ConfigReadSeq();
	tx_buffer[4] = (seq >> 8) & 0xff;
	tx_buffer[5] = (seq     ) & 0xff;
	// sender id (same as [1..3])
	tx_buffer[6] = (id >> 16) & 0xff;
	tx_buffer[7] = (id >> 8 ) & 0xff;
	tx_buffer[8] = (id      ) & 0xff;
	// expand message
	int len = expand_message(tx_buffer+10, param);
	tx_buffer[9] = len;
	// TX
	PacketEnqueue(PACKET_TX_FIFO, tx_buffer, len+10);
	// Seqを更新する
	seq++;
	ConfigWriteSeq(seq);
	return 1;
}


int command_uplink(const uint8_t *param){
	uint8_t tx_buffer[256];
	// Build header
	tx_buffer[0] = 0x02;   // CMD=UPLINK
	// UplinkID
	uint32_t uplink_id = ConfigReadUplinkID();
	tx_buffer[1] = (uplink_id >> 16) & 0xff;
	tx_buffer[2] = (uplink_id >> 8 ) & 0xff;
	tx_buffer[3] = (uplink_id      ) & 0xff;
	// SenderID (my_id)
	uint32_t my_id = ConfigReadDeviceID();
	tx_buffer[4] = (my_id >> 16) & 0xff;
	tx_buffer[5] = (my_id >> 8 ) & 0xff;
	tx_buffer[6] = (my_id      ) & 0xff;
	// expand message
	int len = expand_message(tx_buffer+8, param);
	tx_buffer[7] = len;
	// TX
	PacketEnqueue(PACKET_TX_FIFO, tx_buffer, len+8);
	return 1;
}

static int command_sendto1(const uint8_t *param) {
	uint8_t tx_buffer[256];
	// Build header
	tx_buffer[0] = 0x03;   // CMD=SENDTO1
	// SenderID
	uint32_t id = ConfigReadDeviceID();
	tx_buffer[1] = (id >> 16) & 0xff;
	tx_buffer[2] = (id >> 8) & 0xff;
	tx_buffer[3] = (id) & 0xff;
	// Seq
	uint16_t seq = ConfigReadSeq();
	tx_buffer[4] = (seq >> 8) & 0xff;
	tx_buffer[5] = (seq) & 0xff;
	// receiver id
	tx_buffer[6] = (Hex1toNum1(param[0]) << 4 | Hex1toNum1(param[1])) & 0xff;
	tx_buffer[7] = (Hex1toNum1(param[2]) << 4 | Hex1toNum1(param[3])) & 0xff;
	tx_buffer[8] = (Hex1toNum1(param[4]) << 4 | Hex1toNum1(param[5])) & 0xff;
	// expand message
	int len = expand_message(tx_buffer + 10, param + 6);
	tx_buffer[9] = len;
	// TX
	PacketEnqueue(PACKET_TX_FIFO, tx_buffer, len + 10);
	// Seqを更新する
	seq++;
	ConfigWriteSeq(seq);
	return 1;
}

static int command_sendto2(const uint8_t *param) {
	uint8_t tx_buffer[256];
	// Build header
	tx_buffer[0] = 0x04;   // CMD=SENDTO2
	// UplinkID
	uint32_t uplink_id = ConfigReadUplinkID();
	tx_buffer[1] = (uplink_id >> 16) & 0xff;
	tx_buffer[2] = (uplink_id >> 8 ) & 0xff;
	tx_buffer[3] = (uplink_id      ) & 0xff;
	// Receiver ID
	tx_buffer[4] = (Hex1toNum1(param[0]) << 4 | Hex1toNum1(param[1])) & 0xff;
	tx_buffer[5] = (Hex1toNum1(param[2]) << 4 | Hex1toNum1(param[3])) & 0xff;
	tx_buffer[6] = (Hex1toNum1(param[4]) << 4 | Hex1toNum1(param[5])) & 0xff;
	// SenderID (my_id)
	uint32_t my_id = ConfigReadDeviceID();
	tx_buffer[7] = (my_id >> 16) & 0xff;
	tx_buffer[8] = (my_id >> 8) & 0xff;
	tx_buffer[9] = (my_id) & 0xff;
	// expand message
	int len = expand_message(tx_buffer + 11, param+6);
	tx_buffer[10] = len;
	// TX
	PacketEnqueue(PACKET_TX_FIFO, tx_buffer, len + 11);
	return 1;
}


static int command_sendto(const uint8_t *param){
	// HQ か HQ以外かで動作が変わる
	if( (ConfigReadDeviceID() & 0x0000FFFF) == 0 ){
		// HQ
		return command_sendto1(param);
	} else {
		// HQ以外
		return command_sendto2(param);
	}
}

void CommandExecute(const uint8_t *cmd) {
	int ret = -1;  // ret<0: ERROR, ret>0: OK, ret=0: no message
	if (check_substr(cmd, "AT/1")){
		ret = command_cleareeprom();
	} else if (check_substr(cmd, "AT/2")){
			ret = command_clearbuffer();
	} else if (check_substr(cmd, "AT/")) {  // STTINGS
		ret = command_settings(cmd+3);
	} else if (check_substr(cmd, "AT?")) {  // HELP
		ret = command_help();
	} else if (check_substr(cmd, "ATZ")) {  // INITIALIZE SYSTEM
		ret = command_initialize();
	} else if (check_substr(cmd, "ATI")) {  // INFO
		ret = command_info();
	} else if (check_substr(cmd, "ATG")) {  // GET DATA
		ret = command_getdata();
	} else if (check_substr(cmd, "ATD")) {  // DEBUG MODE
		ret = command_debug( cmd[3]=='1' );
	} else if (check_substr(cmd, "ATB")) {  // BROADCAST
		ret = command_broadcast(cmd+3);
	} else if (check_substr(cmd, "ATU")) {  // UPLINK
		ret = command_uplink(cmd+3);
	} else if (check_substr(cmd, "ATF")) {  // FUNCTION
		ret = command_function(cmd+3);
	} else if (check_substr(cmd, "ATS")) {  // SENDTO
		ret = command_sendto(cmd+3);
	} else if( check_substr(cmd, "AT")){   // AT only
		ret = (cmd[2]==0) ? 1 : -1;
	} else {
		ret = -1;  // error message
	}

	// Message
	if( ret < 0 ){
		SerialOutput("ERROR (help = AT?)\r\n", cmd);
	} else if( ret > 0 ){
		SerialOutput("OK\r\n", cmd);
	}
}

