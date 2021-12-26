/*
 * algo_suivi_bords.c
 *
 *  Created on: 25 déc. 2021
 *      Author: nmira
 */

#include "pid.h"

#define ROBOT_ACC_MAX 0.04 // m/s²
#define ROBOT_DEC_MAX 0.25 // m/s²

void calculConsigneTrapezoidaleVitesse(float deltaT, float vitesse_cible, float vitesse_courante)
{
    // Calcul de la consigne de vitesse fonction du trapeze
//    if(vitesse_cible > vitesse_courante)
//    {
//        if((vitesse_cible - vitesse_courante) < ROBOT_ACC_MAX)
//            vitesse_courante = vitesse_cible;
//        else
//            vitesse_courante += ROBOT_ACC_MAX;
//    } else
//    if(vitesse_cible < vitesse_courante)
//    {
//        if((vitesse_courante - vitesse_cible) < ROBOT_DEC_MAX)
//            vitesse_courante = vitesse_cible;
//        else
//            vitesse_courante -= ROBOT_DEC_MAX;
//    }
//
    float difference;

    difference = vitesse_cible - vitesse_courante;

    if(difference > 0)
    	if(difference > ROBOT_ACC_MAX/deltaT)
    		vitesse_courante += ROBOT_ACC_MAX/deltaT;
    	else
    		vitesse_courante = vitesse_cible;
    else if(difference < 0)
    	if(difference < ROBOT_DEC_MAX/deltaT)
    		vitesse_courante -= ROBOT_DEC_MAX/deltaT;
    	else
    		vitesse_courante = vitesse_cible;
}

typedef enum { automate_auto_depart, automate_auto_stop, automate_auto_decouverte, automate_auto_fini } eEtatsAutomateAutomatique;

eEtatsAutomateAutomatique etat_automate_automatique;

void algo_decouverte()
{

	etat_automate_automatique = automate_auto_depart;

	do {

		  switch(etat_automate_automatique)
		  {
		  case automate_auto_depart :
			  // Acceleration et passage sous l'arche une fois
			  // Lorsque le robot passe sous l'arche alors passage à l'état automate_auto_decouverte
			  break;
		  case automate_auto_decouverte :
			  // Suivi de la piste et enregistrement de la position, du sens des virages
			  // Si passage sous l'arche, alors passage à l'état automate_auto_stop
			  break;
		  case automate_auto_stop :
			  // Decelleration jusqu'à l'arrêt total
			  // Quand la vitesse reelle est à 0, alors passage à l'état automate_auto_fini
			  break;
		  default :
			  etat_automate_automatique = automate_auto_depart;
		  }

	} while (etat_automate_automatique != automate_auto_fini);

}

void algo_init()
{

}
