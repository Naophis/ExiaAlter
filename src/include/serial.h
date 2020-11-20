/*
 * serial.h
 *
 *  Created on: 2016/06/02
 *      Author: Naoto
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdio.h>
#include <stdarg.h>
#include "iodefine.h"

void putnbyte(char *buf, int len);
int myprintf(const char *fmt, ...);
void sci_recv(void);
char INT_RXI1(void);
int dequeue();
int enqueue(char enq_data);

void queClear();
int enqueue(char enq_data);
void SERIAL_EnableSerialRecive(void);
void SERIAL_DisableSerialRecive(void);
void put1byte(char c);
char INT_RXI1(void);      // 正常に受信できた場合の処理
int get1byte(void);
void putnbyte(char *buf, int len);    //nbyte
int myprintf(const char *fmt, ...);    //可変長引数・・・データ値が違っても使える
void intprg_rxi0(void);
short SCI_read(unsigned char *c);
int dequeue();
char charget(void);

#endif /* SERIAL_H_ */
