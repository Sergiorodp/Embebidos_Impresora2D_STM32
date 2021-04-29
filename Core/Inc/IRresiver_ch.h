#ifndef IRresiver_ch_h
#define IRresiver_ch_h

#include "stm32f1xx_hal.h"

// defines

#define IR_SONY_BITS                12
#define IR_SONY_MARK             	2400
#define IR_SONY_SPACE             	600
#define IR_SONY_ONE_MARK            1200
#define IR_SONY_ZERO_MARK           600
#define IR_SONY_RPT_LENGTH          45000


#define IR_STATE_IDLE       2
#define IR_STATE_MARK       3
#define IR_STATE_SPACE      4
#define IR_STATE_STOP       5
#define IR_STATE_OVERFLOW   6
#define IR_STATE_LONG_SPACE 7


#define IR_MAXBUFF         101

// tolerancia en la medida

#define IR_TOLERANCE       25
#define IR_LTOL            0.75
#define IR_HTOL            1.25

#define IR_MARK   0
#define IR_SPACE  1

union Data_store {
    char myByte[4];
    uint32_t mylong;
} data;


typedef struct
{

	uint8_t       rcvstate;
	uint16_t      recvpin;
	uint8_t       rawlen;
	GPIO_TypeDef* recvpinport;
	unsigned int  timer;
	uint8_t       rawbuf[IR_MAXBUFF];
	uint8_t       overflow;
	uint8_t 	  rawbuf_space[IR_MAXBUFF];

}parametros_ir;


typedef struct
{

	unsigned int           address;
	union Data_store       value;
	int                    bits;
	uint8_t                *rawbuf;
	int                    rawlen;
	int                    overflow;

} ir_decode_results;


volatile parametros_ir params;
volatile ir_decode_results irresults;

uint8_t Compress[2];
GPIO_PinState resive;

GPIO_PinState numero_selec_flag;
uint32_t numero_seleccionado, numero_temp;
uint8_t tecla_actual;


void reciveData(void);
uint8_t decodeData(volatile ir_decode_results *record );
void InitResiver (GPIO_TypeDef* recvpinport, uint16_t recvpin);
void resume (void);
uint8_t identify( uint8_t *save, uint8_t pos, uint8_t fin);

void detectar_tecla(void);
void leer_num(void);

#endif
