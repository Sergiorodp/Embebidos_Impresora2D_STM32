#include "PID_ch.h"
#include <stdio.h>
#include <string.h>


void initPID_ch(){

	u=0.0,u_1=0.0;

	r1=0.0;

	e=0.0;
	e_1=0.0;
	e_2=0.0;

	Ts = 1;

	k= 0 , tau= 0, theta= 150/16 + Ts/2 ;

	//*************************************************************************//
	//*****************   SINTONIA POR ZIEGLER y NICHOLS    *******************//
	//*************************************************************************//

	kp=(1.2*tau)/(k*theta);
	ti=2.0*theta;
	td=0.5*theta;

	q0=kp*(1+Ts/(2.0*ti)+td/Ts);
	q1=-kp*(1-Ts/(2.0*ti)+(2.0*td)/Ts);
	q2=(kp*td)/Ts;

}

void PID(){

	e=(r1-v1); // calcular error

	u = u_1 + q0*e + q1*e_1 + q2*e_2; //Ley del controlador PID discreto

	if (u >= 50.0)        //Saturo la accion de control 'uT' en un tope maximo y minimo
	u = 50.0;
	if (u <= 0.0 || r1==0)
	u = 0.0;

	e_2=e_1;
	e_1=e;
	u_1=u;

}

void actualizar_par(){

	kp=(1.2*tau)/(k*theta);
	ti=2.0*theta;
	td=0.5*theta;

	q0=kp*(1+Ts/(2.0*ti)+td/Ts);
	q1=-kp*(1-Ts/(2.0*ti)+(2.0*td)/Ts);
	q2=(kp*td)/Ts;

}

void params_choose( uint16_t limits){
	if(limits < 600){
		tau = (r1 * 0.7);
		theta= 180/16 + Ts/2;
	}
	else if(limits >= 600 && limits <= 1500){
		tau = 1500/16;
		theta= 230/16 + Ts/2;
	}else{
		tau = 3000/16;
		theta= 150/16 + Ts/2;
	}
}

