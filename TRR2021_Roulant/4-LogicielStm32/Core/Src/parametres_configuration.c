/*
 * parametres_configuration.c
 *
 *  Created on: 26 déc. 2021
 *      Author: nmira
 */
#include "stdio.h"
#include "parametres_configuration.h"

st_param_conf gParametresConfiguration;

void paramConf_restaure()
{
	// Relecture dans la flash
	gParametresConfiguration.acceleration_max = 12.0; // mesuré le 24 Jan 2022 : 24.34 m/s²
	gParametresConfiguration.deceleration_max = 12.0;
	gParametresConfiguration.vitesse_max_ligne_droite = 5.0;
	gParametresConfiguration.vitesse_max_virage = 3.0;
	gParametresConfiguration.vitesse_max_decouverte = 5.0;
	gParametresConfiguration.pid_vitesse_kp = 1.0;
	gParametresConfiguration.pid_vitesse_ki = 0.0;
	gParametresConfiguration.pid_vitesse_kd = 5.0;
	gParametresConfiguration.pid_direction_kp = 0.5;
	gParametresConfiguration.pid_direction_ki = 0.0;
	gParametresConfiguration.pid_direction_kd = 10.0;

}

void paramConf_sauvegarde()
{
	// Ecritude dans la flash
}

void paramConf_lecture()
{
	// Affichage des parametres courants
	printf("%f %f %f %f %f %f %f %f %f %f %f\r\n",
			gParametresConfiguration.acceleration_max,
			gParametresConfiguration.deceleration_max,
			gParametresConfiguration.vitesse_max_ligne_droite,
			gParametresConfiguration.vitesse_max_virage,
			gParametresConfiguration.vitesse_max_decouverte,
			gParametresConfiguration.pid_vitesse_kp,
			gParametresConfiguration.pid_vitesse_ki,
			gParametresConfiguration.pid_vitesse_kd,
			gParametresConfiguration.pid_direction_kp,
			gParametresConfiguration.pid_direction_ki,
			gParametresConfiguration.pid_direction_kd );
}

void paramConf_modification(float a_acceleration_max, float a_deceleration_max,
		float a_vitesse_max_ligne_droite, float a_vitesse_max_virage, float a_vitesse_max_decouverte,
		float a_pid_vitesse_kp, float a_pid_vitesse_ki, float a_pid_vitesse_kd,
		float a_pid_direction_kp, float a_pid_direction_ki, float a_pid_direction_kd)
{
	// Modification des parametres courants
	gParametresConfiguration.acceleration_max = a_acceleration_max;
	gParametresConfiguration.deceleration_max = a_deceleration_max;
	gParametresConfiguration.vitesse_max_ligne_droite = a_vitesse_max_ligne_droite;
	gParametresConfiguration.vitesse_max_virage = a_vitesse_max_virage;
	gParametresConfiguration.vitesse_max_decouverte = a_vitesse_max_decouverte;
	gParametresConfiguration.pid_vitesse_kp = a_pid_vitesse_kp;
	gParametresConfiguration.pid_vitesse_ki = a_pid_vitesse_ki;
	gParametresConfiguration.pid_vitesse_kd = a_pid_vitesse_kd;
	gParametresConfiguration.pid_direction_kp = a_pid_direction_kp;
	gParametresConfiguration.pid_direction_ki = a_pid_direction_ki;
	gParametresConfiguration.pid_direction_kd = a_pid_direction_kd;

}
