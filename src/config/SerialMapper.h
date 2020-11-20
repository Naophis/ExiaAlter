/*
 * SerialMapper.h
 *
 *  Created on: 2017/08/17
 *      Author: Naoto
 */

#ifndef SERIALMAPPER_H_
#define SERIALMAPPER_H_

#include "string.h"
#include <stdlib.h>

#define STAY 0
#define KEY 1
#define VALUE 2
#define END 3
volatile int recieveMode = STAY;
volatile char lastData = 0;
#define MAX_BUFFER 20
typedef struct {
	int index;
	int length;
	char buffer[MAX_BUFFER];
} s_key;
s_key keys, values;

void pushKey(char key) {
	if (keys.length < MAX_BUFFER) {
		keys.buffer[keys.length] = key;
		keys.length++;
	}
}
void pushValue(char val) {
	if (values.length < MAX_BUFFER) {
		values.buffer[values.length] = val;
		values.length++;
	}
}
void flushData() {
	for (int i = 0; i < MAX_BUFFER; i++) {
		keys.buffer[i] = values.buffer[i] = 0;
	}
	keys.index = keys.length = values.index = values.length = 0;
}

char writeParam(int key, float val);
void assing(long id, float val);
void setA(long key, long id, float val);
void mapping() {
	const char *keyBuffer = &(keys.buffer);
	const char *valueBuffer = &(values.buffer);
	long key = atoi(keyBuffer);
	double value = atof(valueBuffer);
	assing(key, value);
}
void applyRecieveData(char type, char data) {
	if (type == KEY) {
		pushKey(data);
	} else if (type == VALUE) {
		pushValue(data);
	} else if (type == END) {
		mapping();
	} else if (type == STAY) {
	}
}
void detectChar() {
	SCI1.SSR.BIT.RDRF = 0; // 受信フラグを解除
	char recieveData = SCI1.RDR;
	lastData = recieveData;
	if (!enableSciUpdate) {
		return;
	}
	skipPrint = true;
	switch (recieveData) {
	case '{':
		recieveMode = KEY;
		flushData();
		break;
	case '}':
		recieveMode = END;
		applyRecieveData(recieveMode, recieveData);
		break;
	case ':':
		recieveMode = VALUE;
		break;
	default:
		applyRecieveData(recieveMode, recieveData);
		break;
	}
	skipPrint = false;
}

void setA(long key, long id, float val) {
	float backup[16];
	uint32_t address = key;
	while (true) {
		for (char i = 0; i < 16; i++) {
			backup[i] = *(float *) (address + 4 * i);
			if (id == address + 4 * i) {
				backup[i] = val;
			}
		}
		flash_err_t ret = R_FLASH_Erase((flash_block_address_t) address, 1);
		flash_err_t ret2 = R_FLASH_Write((uint32_t) backup, key,
				sizeof(backup));
		char check = true;
		for (char i = 0; i < 16; i++) {
			if (backup[i] != *(float *) (address + 4 * i)) {
				check = false;
			}
		}
		break;
		cmt_wait(10);
	}
	cmtMusic(C3_, 20);
}
void assing(long id, float val) {
	for (unsigned long i = FLASH_DF_BLOCK_1023; i >= FLASH_DF_BLOCK_10; i -=
			0x40)
		if (id >= i)
			setA(i, id, val);
}

#endif /* SERIALMAPPER_H_ */
