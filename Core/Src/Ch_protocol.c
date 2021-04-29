/*
 *
 * Ch_protocol - Comunication protocol Library for STM32 ARM microcontrollers
 *
 *  Created on: April 7, 2021
 *      Author: Sergio Rodriguez (sergio.rod.perez@outlook.es.com)
 *      Published to: Github (https://github.com/SayidHosseini/STM32LiquidCrystal)
 */

#include "stm32f1xx_hal.h"
#include "Ch_protocol.h"
#include <stdio.h>
#include <string.h>

uint8_t paquete[125],
        data_Ch[125],
		sum = 0,
		i;

volatile GPIO_PinState
		NoACK = GPIO_PIN_RESET;


// functions

void protocolInit( GPIO_PinState repeat ){
	resive_protocol = GPIO_PIN_RESET;
	wait = GPIO_PIN_RESET;
	waitTime = 0;
	check = GPIO_PIN_RESET;
	ACK = GPIO_PIN_SET;
	repeat_m = repeat; // reset repeat
}

void ArmarPack(uint8_t* data, uint8_t command, uint8_t packLen){

	paquete[0] = START; // inicio
	paquete[1] = command; // comando
	paquete[2] = packLen; // tamaño 1
	for(i = 0; i < packLen; i++){
		paquete[i + 3] = data[i]; // data
	}
	XORData(&paquete);
	paquete[packLen + 3] = sum;
	paquete[packLen + 4] = END; // final
}

void XORData(uint8_t* data){
	sum = data[1];
	for (i = 2; i < (data[2] + 3); i++){
		sum ^= data[i];
	}
}

void AnalisePack( uint8_t *package , uint8_t *numData){

	sum = package[1];
	for(i = 2; i < (package[2] + 3); i++){
		sum ^= package[i];
	}

	if(sum == package[package[2] + 3]){

		if(package[1] == Ack){

			if(package[3] == 1){

				ACK = GPIO_PIN_SET;
				NoACK = GPIO_PIN_RESET;
				*numData += 1;

			}else{

				NoACK = GPIO_PIN_SET;
				ACK = GPIO_PIN_RESET;
			}

		}else{

			SendACK(GPIO_PIN_SET, &data_Ch);
			check = GPIO_PIN_SET;

		}
	}else{
		SendACK(GPIO_PIN_RESET, &data_Ch);
		check = GPIO_PIN_RESET;
	}
}

void SendACK( GPIO_PinState check, uint8_t *data ){
	if(check){
		data[0] = 0x01;
		ArmarPack( data, Ack , 1);
	}else{
		data[0] = 0x00;
		ArmarPack( data, Ack , 1);
	}
	CDC_Transmit_FS(paquete, 6);
}

/*
 * Paquete
 * resivir data
 * enviar data
 */


void communication( uint8_t *package, GPIO_PinState *rxData,
		            uint8_t *data, uint8_t *_commd, uint8_t *_len, uint8_t *numData){

	if( *rxData ){
		*rxData = GPIO_PIN_RESET;
		if(package[0] == START && (package[package[2] + 4]) == END){
			AnalisePack(package, numData);
		}
	}

	if( waitTime == 1 && repeat_m){
		NoACK = GPIO_PIN_SET;
		waitTime = 0;
	}

	if(ACK){
		//memcpy( _data_protocol_save, data , (*_len));
		wait = GPIO_PIN_RESET;
		waitTime = 0;
		ArmarPack(data, *_commd, *_len);
	}

	if(ACK || NoACK){
		CDC_Transmit_FS(paquete, (paquete[2]) + 5);
		ACK = GPIO_PIN_RESET;
		NoACK = GPIO_PIN_RESET;
		wait = GPIO_PIN_SET;
	}

}

