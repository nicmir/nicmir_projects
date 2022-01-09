/*
 * parametres_configuration.h
 *
 *  Created on: 26 déc. 2021
 *      Author: nmira
 */

#ifndef INC_PARAMETRES_CONFIGURATION_H_
#define INC_PARAMETRES_CONFIGURATION_H_

typedef struct {
	float acceleration_max;
	float deceleration_max;

	float vitesse_max_ligne_droite;
	float vitesse_max_virage;
	float vitesse_max_decouverte;

	float pid_vitesse_kp;
	float pid_vitesse_ki;
	float pid_vitesse_kd;

	float pid_direction_kp;
	float pid_direction_ki;
	float pid_direction_kd;

} st_param_conf;

void paramConf_restaure();
void paramConf_sauvegarde();
void paramConf_lecture();
void paramConf_modification(float a_acceleration_max, float a_deceleration_max,
		float a_vitesse_max_ligne_droite, float a_vitesse_max_virage, float a_vitesse_max_decouverte,
		float a_pid_vitesse_kp, float a_pid_vitesse_ki, float a_pid_vitesse_kd,
		float a_pid_direction_kp, float a_pid_direction_ki, float a_pid_direction_kd);

#endif /* INC_PARAMETRES_CONFIGURATION_H_ */
