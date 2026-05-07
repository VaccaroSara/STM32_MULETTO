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

//  FUNZIONE SERVO (MIUZEI MS24) PROVA GEMINI
void servo_motor(float deg_angle)
{
    // 1. Limita l'angolo richiesto ai limiti meccanici (Manteniamo il vostro)
    if(deg_angle < MIN_ANGOLO)
        deg_angle = MIN_ANGOLO;
    else if (deg_angle > MAX_ANGOLO)
        deg_angle = MAX_ANGOLO;

    // 2. Calcolo angolo reale
    float conv_angolo = deg_angle + DRITTO;

    // 3. Calcolo microsecondi (La vostra formula calibrata perfetta)
    // 1815 = T_on per conv_angolo = 57°
    // 7.32 = variazione T_on per ogni grado
    float t_on = 1815.0 - (conv_angolo - 57.0) * 7.3255814;

    // 4. PARACADUTE DI SICUREZZA (500us - 2500us)
    // Se per qualche assurdo motivo t_on esce dai limiti, lo blocchiamo qui
    if (t_on < 500.0) {
        t_on = 500.0;
    } else if (t_on > 2500.0) {
        t_on = 2500.0;
    }

    // 5. Invio diretto al Timer
    TIM1->CCR2 = (uint32_t)t_on;
}

//FORMULA STORICA
	/*if(deg_angle < MIN_ANGOLO)
			deg_angle = MIN_ANGOLO;

	else if (deg_angle > MAX_ANGOLO)
		deg_angle = MAX_ANGOLO;

	float conv_angolo = deg_angle + DRITTO; // conv_angolo - 43° < conv_angolo < conv_angolo + 43°
	//printf("%f\r\n", conv_angolo); //Stampa angolo per DEBUG

	float t_on = 1815-(conv_angolo-57)*7.3255814; // 1815 = T_on per conv_angolo = 57°, in microsec; 7.32... = quanti microsec deve variare il T_on ad ogni grado.

	float duty=t_on/20000; // T_ON/T, dove T=20000 è il periodo ("UpdateEvent") dell'onda convertito in microsecondi: (10^6){(Timer_clk)/([Prescaler+1][Period+1]))}^-1
	float ccr = (duty*(TIM1->ARR));
	TIM1->CCR2=ccr;*/
	// 1. LIMITATORE DI SICUREZZA (Evita di spaccare lo sterzo)
	    /*if (deg_angle > 20) {
	    	deg_angle = 20;
	    } else if (deg_angle < -20) {
	    	deg_angle = -20;
	    }

	    // 2. LA FORMULA MAGICA (Traduzione da gradi a microsecondi)
	    // Se angolo = 0   --> microsecondi = 1500 (Centro perfetto)
	    // Se angolo = 20  --> microsecondi = 1500 + (20 * 25) = 2000 (Tutto a destra)
	    // Se angolo = -20 --> microsecondi = 1500 + (-20 * 25) = 1000 (Tutto a sinistra)
	    int microsecondi = 1500 + (deg_angle * 25);

	    // 3. MANDIAMO IL SEGNALE AL NUOVO CANALE 2 (PE11)
	    TIM1->CCR2 = microsecondi;*/

