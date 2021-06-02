#ifndef PID_ch_h
#define PID_ch_h

#include "stm32f1xx_hal.h"

float v1,aux;       //velocidad medid
float r1;                     //Referencia
volatile float u,u_1;     //Acción de Control

float Ts;    //Periodo de muestreo

typedef enum state{
		  on_off ,
		  off_off,
		  off_on ,
		  on_on  }
State_sensor;

typedef enum direccion_num{
			stop,
			derecha,
			izquierda
}Direction;

typedef struct control{

	float feedback;
	float ref;
	float u, u_1;
	float kp,ti,td;
	float q0,q1,q2;
	float e,e_1,e_2;
	float k,tau,theta;
	float Ts;

}Control;

typedef struct motor{

	uint8_t num_sensor;
	State_sensor actual;
	State_sensor anterior;
	Direction move;
	uint8_t revoluciones;
	int distancia;
	uint16_t porcent;
	GPIO_TypeDef * GPIOx;
	uint16_t GPIO_Pin_1;
	uint16_t GPIO_Pin_2;
	uint16_t tim_chanel;
	Control pid;
	uint8_t in_pos;

}Motor;


// Motores

Motor M1;
Motor M2;


//Parámetros del PID
float kp,ti,td;
float q0,q1,q2;

volatile float e,e_1,e_2;

float k,tau,theta;   //Parámetros del Modelo del sistema


// funciones

void initPID_ch(void);
void PID(void);
void actualizar_par(void);
void params_choose( uint16_t limits);

void initPID_ch_m( Motor * ref );
void PID_m( Motor * ref );
void Actualizar_par_m( Motor * ref );
void Params_choose_m( Motor * ref );

#endif
