#include "stm32f1xx_hal.h"
#include "IRresiver_ch.h"
#include "stm32f1xx_hal_tim.h"


uint8_t count = 0, value = 0, adress = 0;


void InitResiver (GPIO_TypeDef* recvpinport, uint16_t recvpin)
{

	params.recvpinport = recvpinport;
	params.recvpin = recvpin;
	params.rcvstate = IR_STATE_IDLE;
	params.rawlen = 0;
	params.timer = 0;
	resive = GPIO_PIN_RESET;
	numero_selec_flag = GPIO_PIN_RESET;
	numero_seleccionado = 0;
	numero_temp = 0;
}


void reciveData(){

	uint8_t irdata = (uint8_t) HAL_GPIO_ReadPin(params.recvpinport, params.recvpin);

	params.timer++;

	if (params.rawlen >= IR_MAXBUFF) params.rcvstate = IR_STATE_OVERFLOW ;

	switch(params.rcvstate)
	{
		case IR_STATE_IDLE:
			if (irdata == IR_MARK)
			{

				if (params.timer < 26)
				{
					params.timer = 0;
				}
				else
				{
					params.overflow                  = 0;
					params.rawlen                    = 0;
					params.timer                     = 0;
					params.rcvstate                  = IR_STATE_MARK;
					Compress[0]                      = 0;
					Compress[1]                      = 0xaa;
				}
			}

			break;

			//------------------------------------------------------------------------------

		case IR_STATE_MARK:

			if (irdata == IR_SPACE && params.timer != 0)
			{
				params.rawbuf[params.rawlen++] = params.timer;
				params.timer                     = 0;
				params.rcvstate                  = IR_STATE_SPACE;
			}
			break;
			//-------------------------------------------------------------------------------

		case IR_STATE_SPACE:

			if (irdata == IR_MARK && params.timer != 0)
			{
				if( params.timer < 36){

					params.rcvstate                  = IR_STATE_MARK;
					params.timer                     = 0;

				}else{

					resume();

				}
			}
			else if (params.timer > 700)
			{

				params.rcvstate = IR_STATE_STOP;

			}
			break;

		//......................................................................
		case IR_STATE_STOP:  // stop read
			if (irdata == IR_MARK)  {

				params.timer = 0 ;

				/*
				CDC_Transmit_FS(params.rawbuf,params.rawlen);
				params.rcvstate = IR_STATE_IDLE;
				*/
			}
			break;

		/*......................................................................
		case IR_STATE_OVERFLOW:  // overflow

			params.overflow = 1;
			params.rcvstate = IR_STATE_STOP;

			break;
			*/
	}


	if(params.rcvstate == IR_STATE_MARK || params.rcvstate == IR_STATE_SPACE){
		if (irdata == IR_MARK) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		}
		else if((irdata == IR_SPACE)){
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); }
	}

	if( params.rcvstate == IR_STATE_STOP ){

		decodeData(&irresults);
		resive = GPIO_PIN_SET;

	}
}

void resume ()
{
	params.rcvstate = IR_STATE_IDLE;
	params.rawlen = 0;

}


uint8_t decodeData(volatile ir_decode_results *record ){


	if ( !( (params.rawbuf[0]) <= 50 && (params.rawbuf[0]) >= 44  )){
		resume();
		return 0 ;
	}

	identify(&value,1,7);
	identify(&adress,7,12);

	Compress[0] = value;
	Compress[1] = adress;

	detectar_tecla();

	resume();

	return 1;

}


uint8_t identify( uint8_t *save, uint8_t pos, uint8_t fin){

	for(count = pos; count <= fin; count ++){

		if ( params.rawbuf[count] >= 20 && params.rawbuf[count] <= 28 ){
			*save = (*save << 1) | 1;
		}
		else if( params.rawbuf[count] >= 8 && params.rawbuf[count] <= 16 ){
			*save = (*save << 1) | 0;
		}
	}

	return 1;
}

void leer_num(){


		if(Compress[0] == 0x68 && Compress[1] == 0xcb && !numero_selec_flag){

			numero_temp              = numero_seleccionado;
			numero_seleccionado      = 0;
			numero_selec_flag        = GPIO_PIN_SET;

		}
		else if(Compress[0] == 0x38 && Compress[1] == 0xcb)
		{
			numero_seleccionado /= 10;
		}
		else if(tecla_actual < 10){

			numero_selec_flag     = GPIO_PIN_RESET;
			numero_seleccionado   *= 10;
			numero_seleccionado   += tecla_actual;

		}
}

void detectar_tecla(){
	 switch (Compress[1]) {
	    case 0xCB:
	        switch (Compress[0]) {
	            case 0x00:
	                tecla_actual = 1;
	            break;
	            case 0x40:
	            	tecla_actual = 2;
	            break;
	            case 0x20:
	            	tecla_actual = 3;
	            break;
	            case 0x60:
	            	tecla_actual = 4;
	            break;
	            case 0x10:
	            	tecla_actual = 5;
	            break;
	            case 0x50:
	            	tecla_actual = 6;
	            break;
	            case 0x30:
	            	tecla_actual = 7;
	            break;
	            case 0x70:
	            	tecla_actual = 8;
	            break;
	            case 0x08:
	            	tecla_actual = 9;
	            break;
	            case 0x48:
	            	tecla_actual = 0;
	            break;
	            default:
	            	tecla_actual = 10;
	            	break;
	        }
	        break;
	     default:
	       tecla_actual = 10;
	       break;
	 }
}

