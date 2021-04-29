#ifndef PID_ch_h
#define PID_ch_h

#include "stm32f1xx_hal.h"

float v1,aux;       //velocidad medid
float r1;                     //Referencia
volatile float u,u_1;     //Acción de Control

uint8_t Ts;                  //Periodo de muestreo

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

#endif
