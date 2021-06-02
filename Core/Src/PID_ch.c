#include "PID_ch.h"
#include <stdio.h>
#include <string.h>


void initPID_ch(){

	u=0.0,u_1=0.0;

	r1=0.0;

	e=0.0;
	e_1=0.0;
	e_2=0.0;

	Ts = 10;

	k= 0 , tau= 0, theta= 5 + Ts/2 ;

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

void initPID_ch_m( Motor * ref ){

	ref->pid.u=0.0;
	ref->pid.u_1=0.0;

	ref->pid.ref=555.0;

	ref->pid.feedback = 555.0;

	ref->pid.e=0.0;
	ref->pid.e_1=0.0;
	ref->pid.e_2=0.0;

	ref->pid.Ts = 1;

	ref->pid.k= 1;
	ref->pid.tau= 1;
	ref->pid.theta= 5 + ref->pid.Ts/2 ;

	//*************************************************************************//
	//*****************   SINTONIA POR ZIEGLER y NICHOLS    *******************//
	//*************************************************************************//

	ref->pid.kp =   ((1.2 * ref->pid.tau) / (ref->pid.k * ref->pid.theta));
	ref->pid.ti =   2.0 * ref->pid.theta;
	ref->pid.td =   0.5 * ref->pid.theta;

	ref->pid.q0 =    ref->pid.kp * ( 1 + ref->pid.Ts / ( 2.0 * ref->pid.ti ) + ref->pid.td/ref->pid.Ts);
	ref->pid.q1 =   - (ref->pid.kp*( 1 - ref->pid.Ts / ( 2.0 * ref->pid.ti ) + ( 2.0 * ref->pid.td)/ref->pid.Ts));
	ref->pid.q2 =   (ref->pid.kp * ref->pid.td)/ref->pid.Ts;

}

void PID_m( Motor* ref ){

	ref->pid.e = (ref->pid.ref - ref->pid.feedback); // calcular error

	if( (ref->pid.e < 1.4) && (ref->pid.e > (-1.4)) ){

		ref->pid.u = 0;
		ref->pid.e = 0;
		ref->in_pos = 1;

	}else{

		ref->in_pos = 0;

		ref->pid.u = ref->pid.u_1 + ref->pid.q0 * ref->pid.e + ref->pid.q1*ref->pid.e_1 +
				ref->pid.q2*ref->pid.e_2; //Ley del controlador PID discreto

		if (ref->pid.u >= 23) //Saturo la accion de control 'uT' en un tope maximo y minimo
			ref->pid.u = 23;
		if (ref->pid.u <= -23)
			ref->pid.u = -23;

	}

	ref->pid.e_2 = ref->pid.e_1;
	ref->pid.e_1 = ref->pid.e;
	ref->pid.u_1 = ref->pid.u;

}

void PID(){

	e = (r1 - v1); // calcular error

	if( (e < 0.2) && (e > (-0.2)) ){

		u = 0;
		e = 0;

	}else{

		u = u_1 + q0*e + q1*e_1 + q2*e_2; //Ley del controlador PID discreto

		if (u >= 25)        //Saturo la accion de control 'uT' en un tope maximo y minimo
		u = 25;
		if (u <= -25)
		u = -25;

	}

	e_2=e_1;
	e_1=e;
	u_1=u;

}

void actualizar_par_m( Motor * ref){

	ref->pid.kp =   (1.2 * ref->pid.tau) / (ref->pid.k * ref->pid.theta);
	ref->pid.ti =   2.0 * ref->pid.theta;
	ref->pid.td =   0.5 * ref->pid.theta;

	ref->pid.q0 =    ref->pid.kp * ( 1 + ref->pid.Ts / ( 2.0 * ref->pid.ti ) + ref->pid.td/ref->pid.Ts);
	ref->pid.q1 =   -ref->pid.kp*( 1 - ref->pid.Ts / ( 2.0 * ref->pid.ti ) + ( 2.0 * ref->pid.td)/ref->pid.Ts);
	ref->pid.q2 =   (ref->pid.kp * ref->pid.td)/ref->pid.Ts;

}


void actualizar_par(){

	kp =   (1.2*tau)/(k*theta);
	ti =   2.0*theta;
	td =   0.5*theta;

	q0 =    kp*(1+Ts/(2.0*ti)+td/Ts);
	q1 =   -kp*(1-Ts/(2.0*ti)+(2.0*td)/Ts);
	q2 =   (kp*td)/Ts;

}

void Params_choose_m( Motor * ref ){

	ref->pid.e = 0.0;
	ref->pid.e_1 = 0.0;
	ref->pid.e_2 = 0.0;

	ref->pid.u = 0.0 , ref->pid.u_1 = 0.0;

	ref->pid.tau = (ref->pid.ref * 3.8);
	//ref->pid.theta= (0.01 + (ref->pid.ref/1000) ) + ref->pid.Ts/2;
	ref->pid.theta= (0.01 + (ref->pid.ref/1000) ) + ref->pid.Ts/10;

	actualizar_par_m( ref );
}


void params_choose( uint16_t limits){

	e=0.0;
	e_1=0.0;
	e_2=0.0;

	u=0.0,u_1=0.0;

	tau = (r1 * 0.95);
	theta= (0.01 + (r1/2000) ) + Ts/2;

	actualizar_par();
}

