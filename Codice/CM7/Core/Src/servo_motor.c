#include "servo_motor.h"

void servo_motor(float deg_angle)
{
    //Limiti sicurezza
    if(deg_angle < MIN_ANGOLO)
        deg_angle = MIN_ANGOLO;
    else if (deg_angle > MAX_ANGOLO)
        deg_angle = MAX_ANGOLO;

    //Calcolo angolo reale
    float conv_angolo = deg_angle + DRITTO;

    //Calcolo in microsecondi
    float t_on = 1815.0 - (conv_angolo - 57.0) * 7.3255814;

    //Controllo di sicurezza(500us - 2500us)
    if (t_on < 500.0) {
        t_on = 500.0;
    } else if (t_on > 2500.0) {
        t_on = 2500.0;
    }

    //Invio diretto al timer
    TIM1->CCR2 = (uint32_t)t_on;
}

