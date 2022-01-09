/*
 * algo_suivi_bords.c
 *
 *  Created on: 25 déc. 2021
 *      Author: nmira
 */

#include <stdlib.h>
#include "pid.h"
#include "parametres_configuration.h"
#include "tfminiplus.h"
#include "telemetrie.h"
#include "radio_commandes.h"
#include "imu.h"
#include "algo_suivi_bords.h"

extern st_param_conf gParametresConfiguration;
typedef enum { automate_principal_radio, automate_principal_autonome, automate_principal_shell } eEtatsAutomatePrincipal;

void calculConsigneTrapezoidaleVitesse(st_context_robot *a_pRobot, float deltaT)
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

    difference = a_pRobot->vitesse_cible - a_pRobot->vitesse_cmd_courante;

    if(difference > 0)
    {
    	if(difference > gParametresConfiguration.acceleration_max/deltaT)
    		a_pRobot->vitesse_cmd_courante += gParametresConfiguration.acceleration_max/deltaT;
    	else
    		a_pRobot->vitesse_cmd_courante = a_pRobot->vitesse_cible;
    }
    else if(difference < 0)
    {
    	if(difference < gParametresConfiguration.deceleration_max/deltaT)
    		a_pRobot->vitesse_cmd_courante -= gParametresConfiguration.deceleration_max/deltaT;
    	else
    		a_pRobot->vitesse_cmd_courante = a_pRobot->vitesse_cible;
    }
}

//typedef enum { automate_auto_depart, automate_auto_stop, automate_auto_decouverte, automate_auto_fini } eEtatsAutomateAutomatique;

eEtatsAutomateAutomatique etat_automate_automatique;

void algo_decouverte(st_context_robot *a_pRobot, float a_deltaT)
{
	int32_t lidarDroitDistance, lidarDroitStrength, lidarDroitTemperature;
	int32_t lidarGaucheDistance, lidarGaucheStrength, lidarGaucheTemperature;
	int32_t lidarAvantDistance, lidarAvantStrength, lidarAvantTemperature;
	int32_t lidarHautDistance, lidarHautStrength, lidarHautTemperature;
	float robot_vitesse;
	float robot_distance;
	int32_t erreur_direction;
	float throttle, direction;
	st_tele_element *pTeleElement;
	int erreur;

	// Acquisition des distances
	tfminiplus_getLastAcquisition(MINILIDAR_DROIT, &lidarDroitDistance, &lidarDroitStrength, &lidarDroitTemperature);
	tfminiplus_getLastAcquisition(MINILIDAR_GAUCHE, &lidarGaucheDistance, &lidarGaucheStrength, &lidarGaucheTemperature);
	tfminiplus_getLastAcquisition(MINILIDAR_AVANT, &lidarAvantDistance, &lidarAvantStrength, &lidarAvantTemperature);
	tfminiplus_getLastAcquisition(MINILIDAR_HAUT, &lidarHautDistance, &lidarHautStrength, &lidarHautTemperature);

	// Acquisition de la vitesse instantannée du robot
	vehicule_speed_aimant_get(&robot_vitesse);
	vehicule_distance_aimant_get(&robot_distance);

	a_pRobot->elapsedTime += a_deltaT;
	a_pRobot->travelledDistance += robot_distance;

	// Automate
	switch(etat_automate_automatique)
	{
	case automate_auto_depart :
	  // Demarrage jursuqu'à la vitesse de croisière
		a_pRobot->vitesse_cible = gParametresConfiguration.vitesse_max_decouverte;

		calculConsigneTrapezoidaleVitesse(a_pRobot, a_deltaT);

	  // Lorsque le robot passe sous l'arche alors passage à l'état automate_auto_decouverte
//		if((lidarHautDistance != -2) && ((lidarHautDistance>30) || (lidarHautDistance<150)))
//			// Detection de l'arche
//			etat_automate_automatique = automate_auto_decouverte;
		if(a_pRobot->vitesse_cmd_courante == a_pRobot->vitesse_cible)
			etat_automate_automatique = automate_auto_decouverte;
		break;
	case automate_auto_decouverte :
	  // Suivi de la piste et enregistrement de la position, du sens des virages

	  // Si passage sous l'arche, alors passage à l'état automate_auto_stop
		// Pour commencer on met une limite de distance (7m) pour pas que le robot file partout
		if( ((lidarHautDistance != -2) && ((lidarHautDistance>30) || (lidarHautDistance<150))) ||
			(a_pRobot->travelledDistance > 700) )
		{
			// Detection de l'arche
			a_pRobot->vitesse_cible = 0.0;
			etat_automate_automatique = automate_auto_stop;
		}
	  break;
	case automate_auto_stop :
	  // Decelleration jusqu'à l'arrêt total
	  // Quand la vitesse reelle est à 0, alors passage à l'état automate_auto_fini
		if(abs(robot_vitesse) < 0.1) // 0.1 m/s
			etat_automate_automatique = automate_auto_fini;
	  break;
	default :
	  etat_automate_automatique = automate_auto_depart;
	}

	// Calculs des PID
	throttle = pid_output(&(a_pRobot->pidVitesse), a_pRobot->vitesse_cmd_courante - robot_vitesse);

	if((lidarDroitDistance != -2) && (lidarGaucheDistance != -2))
		erreur_direction = lidarDroitDistance - lidarGaucheDistance;
	else
	{
		// Les lidars mesures les distances des bords à 45°.
		// La piste fait 1.5m de largeur
		// La distance mesurée lorsque le robot est au milieu est de  75/cos(45°) soit 106 cm
		if((lidarDroitDistance == -2) && (lidarGaucheDistance == -2))
			// On est mal !!!!
			erreur_direction = 0;
		else if(lidarDroitDistance == -2)
			erreur_direction = 106 - lidarGaucheDistance;
		else if(lidarGaucheDistance == -2)
			erreur_direction = lidarDroitDistance - 106;
	}

	direction = pid_output(&(a_pRobot->pidDirection), (float)erreur_direction);

	// Envoi des commandes
	vehicule_dir_set(direction);
	vehicule_throttle_set(throttle);

	// Télémétrie
	pTeleElement = telemetrie_pt_enreg_suivant(&erreur);
	if(erreur == 0)
	{
		pTeleElement->consigne_direction = direction;
		pTeleElement->consigne_vitesse = throttle;
		pTeleElement->mesure_vitesse = robot_vitesse;
		pTeleElement->mesure_distance = robot_distance;
		pTeleElement->lidar_droit = lidarDroitDistance;
		pTeleElement->lidar_gauche = lidarGaucheDistance;
		pTeleElement->lidar_avant = lidarAvantDistance;
		pTeleElement->lidar_haut = lidarHautDistance;
		pTeleElement->heading = gyro_get_heading();
		pTeleElement->gyro_dps = gyro_get_dps();
		pTeleElement->etat_automate_principal = automate_principal_autonome;
		pTeleElement->etat_automate_auto = etat_automate_automatique;
	}

}

void algo_init(st_context_robot *a_pRobot)
{
	etat_automate_automatique = automate_auto_depart;

	// Initialisation de la télémétrie
	telemetrie_init();
	gyro_reset_heading();

	// Initialisation des éléments de base du robot
	a_pRobot->travelledDistance = 0;
	a_pRobot->vitesse_cible = 0.0;
	a_pRobot->vitesse_cmd_courante = 0.0;
	a_pRobot->elapsedTime = 0.0;

	// Initialisation des PID
	pid_init(&(a_pRobot->pidVitesse), gParametresConfiguration.pid_vitesse_kp, gParametresConfiguration.pid_vitesse_ki, gParametresConfiguration.pid_vitesse_kd, 0.8);
	pid_init(&(a_pRobot->pidDirection), gParametresConfiguration.pid_direction_kp, gParametresConfiguration.pid_direction_ki, gParametresConfiguration.pid_direction_kd, 0.8);

}
