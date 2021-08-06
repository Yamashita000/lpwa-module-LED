/*
 * eeprom.c
 *
 *  Created on: 2021/06/07
 *      Author: kazuaki
 */

#include <stdio.h>
#include <string.h>
#include "stm32l0xx_hal.h"
#include "eeprom.h"

static void write_Halfword_to_eeprom(uint32_t address, uint16_t value) {
	if (!IS_FLASH_DATA_ADDRESS(address)) {
		return;
	}
	HAL_FLASHEx_DATAEEPROM_Unlock();
	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_HALFWORD, address, value);
	HAL_FLASHEx_DATAEEPROM_Lock();
}

static void write_Word_to_eeprom(uint32_t address, uint32_t value) {
	if (!IS_FLASH_DATA_ADDRESS(address)) {
		return;
	}
	HAL_FLASHEx_DATAEEPROM_Unlock();
	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, address, value);
	HAL_FLASHEx_DATAEEPROM_Lock();
}

static uint16_t read_Halfword_from_eeprom(uint32_t address) {
	return (*(__IO uint16_t*) address);
}

static uint32_t read_Word_from_eeprom(uint32_t address) {
	return (*(__IO uint32_t*) address);
}


void ConfigClearEEPROM(void){
	uint32_t *p;
	// Product
	p = (uint32_t *)MEMORY_MAGIC_STR;
	write_Word_to_eeprom(MEMORY_MAGIC,   *p++);
	write_Word_to_eeprom(MEMORY_MAGIC+4, *p);
	// Version
	p = (uint32_t *)MEMORY_VERSION_STR;
	write_Word_to_eeprom(MEMORY_VERSION,   *p++);
	write_Word_to_eeprom(MEMORY_VERSION+4, *p);
	// DeviceID
	ConfigWriteDeviceID(0x000000);
	// Seq
	ConfigWriteSeq(0);
}

void ConfigReadVersion(uint8_t product[], uint8_t version[]){
	// Product
	uint32_t *p = (uint32_t *)product;
	*p++ = read_Word_from_eeprom(MEMORY_MAGIC);
	*p   = read_Word_from_eeprom(MEMORY_MAGIC+4);
	// Version
    p = (uint32_t *)version;
	*p++ = read_Word_from_eeprom(MEMORY_VERSION);
	*p   = read_Word_from_eeprom(MEMORY_VERSION+4);
}

uint32_t ConfigReadDeviceID(void) {
	return read_Word_from_eeprom(MEMORY_DEVICE_ID) & 0x00FFFFFF;
}

void ConfigWriteDeviceID(uint32_t id) {
	if ((id & 0x00FFFFFF) != ConfigReadDeviceID()) {
		write_Word_to_eeprom(MEMORY_DEVICE_ID, id & 0x00FFFFFF);
	}
}

uint32_t ConfigReadUplinkID(void){
	return read_Word_from_eeprom(MEMORY_UPLINK_ID) & 0x00FFFFFF;
}

void ConfigWriteUplinkID(uint32_t id){
	if ((id & 0x00FFFFFF) != ConfigReadUplinkID()) {
		write_Word_to_eeprom(MEMORY_UPLINK_ID, id & 0x00FFFFFF);
	}
}


static uint16_t config_value_seq = 0;

uint16_t ConfigReadSeq(void){
	return config_value_seq;
}

void ConfigWriteSeq(uint16_t seq) {
	if (seq == 65535){
		seq = 0;
	}
	config_value_seq = seq;
}
