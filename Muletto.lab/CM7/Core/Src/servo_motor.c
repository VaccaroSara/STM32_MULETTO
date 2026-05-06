#include "servo_motor.h"


/*
void servo_motor(float angolo)
{
	float tic;
	tic = 0.02;
	  float ccr;
	  float conv_angolo;

	  if(angolo < MIN_ANGOLO)

		   angolo = MIN_ANGOLO;

	  else if (angolo > MAX_ANGOLO)

		   angolo = MAX_ANGOLO;

	  conv_angolo = angolo + DRITTO;

	  ccr=(((0.01111*conv_angolo)+0.5)/tic);
	  TIM1->CCR1=ccr;
}

*/

// VECCHIA FUNZIONE SERVO (MIUZEI MS24)
void servo_motor(float deg_angle){
	if(deg_angle < MIN_ANGOLO)
			deg_angle = MIN_ANGOLO;

	else if (deg_angle > MAX_ANGOLO)
		deg_angle = MAX_ANGOLO;

	float conv_angolo = deg_angle + DRITTO; // conv_angolo - 43° < conv_angolo < conv_angolo + 43°
	//printf("%f\r\n", conv_angolo); //Stampa angolo per DEBUG

	float t_on = 1815-(conv_angolo-57)*7.3255814; // 1815 = T_on per conv_angolo = 57°, in microsec; 7.32... = quanti microsec deve variare il T_on ad ogni grado.

	float duty=t_on/20000; // T_ON/T, dove T=20000 è il periodo ("UpdateEvent") dell'onda convertito in microsecondi: (10^6){(Timer_clk)/([Prescaler+1][Period+1]))}^-1
	float ccr = (duty*(TIM1->ARR));
	//TIM1->CCR2=ccr;
}
