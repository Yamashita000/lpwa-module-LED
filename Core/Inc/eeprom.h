/*
 * eeprom.h
 *
 *  Created on: 2021/06/07
 *      Author: kazuaki
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#define MEMORY_MAGIC_STR     "CMN/IGSS"
#define MEMORY_VERSION_STR   "20210710"

#define MEMORY_MAGIC         0x08080000	// 4bytes
#define MEMORY_VERSION       0x08080008	// 4bytes
#define MEMORY_DEVICE_ID     0x08080010	// 3bytes
#define MEMORY_UPLINK_ID     0x08080014	// 3bytes
// #define MEMORY_SEQ           0x08080018   // SEQ は揮発性

void ConfigClearEEPROM(void);
void ConfigReadVersion(uint8_t product[], uint8_t version[]);

uint32_t ConfigReadDeviceID(void);
void ConfigWriteDeviceID(uint32_t id);

uint32_t ConfigReadUplinkID(void);
void ConfigWriteUplinkID(uint32_t id);

uint16_t ConfigReadSeq(void);
void ConfigWriteSeq(uint16_t seq);



#endif /* INC_EEPROM_H_ */
