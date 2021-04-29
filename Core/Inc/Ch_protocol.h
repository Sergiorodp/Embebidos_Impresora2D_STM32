#ifndef Ch_protocol_h
#define Ch_protocol_h
#include "stm32f1xx_hal.h"

// commands
#define START 0xff
#define END 0xfe
#define Ack 0x20 // Ack
#define Clock 0x21 // Update Clock
#define SETCLOCK 0x25 // Update Clock
#define SETALARM 0x26
#define Cron 0x22 // Update Cron
#define Alarm 0x23 // Update
#define ALL 0x24 // all

GPIO_PinState repeat_m;

// variables
GPIO_PinState resive_protocol,wait,check,ACK;
uint16_t  waitTime;
uint8_t _data_protocol_save[125];

// functions
void ArmarPack(uint8_t* data, uint8_t command, uint8_t packLen);
void AnalisePack( uint8_t *package , uint8_t *numData);
void SendACK( GPIO_PinState check, uint8_t *data );
void communication( uint8_t *package, GPIO_PinState *rxData,
		            uint8_t *data, uint8_t *_commd, uint8_t *_len, uint8_t *numData);
void XORData(uint8_t* data);
void protocolInit( GPIO_PinState repeat );

#endif
