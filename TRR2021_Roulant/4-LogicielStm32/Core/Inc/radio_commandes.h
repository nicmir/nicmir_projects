/*
 * radio_commandes.h
 *
 *  Created on: Oct 31, 2021
 *      Author: nmira
 */

#ifndef INC_RADIO_COMMANDES_H_
#define INC_RADIO_COMMANDES_H_

/*
 * radio_commandes.c
 *
 *  Created on: Oct 31, 2021
 *      Author: nmira
 */

typedef struct {
	uint32_t last_time;
	uint32_t period;
	uint32_t duty_cycle;
} st_pwm_infos;

extern st_pwm_infos radio_dir;
extern st_pwm_infos radio_throttle;
extern st_pwm_infos radio_spare;

extern st_pwm_infos vehicule_speedsensor;

void init_radio_commandes();
// Par convention, l'angle est négatif quand on tourne à gauche.
int radio_dir_get(float *a_pDir);
// Par convention, la vitesse est négative quand on recule.
int radio_throttle_get(float *a_pThrottle);
// !!!! A travailler !!!!
int vehicule_speed_get(float *a_pSpeed);
int vehicule_speed_aimant_get(float *a_pSpeed);
int vehicule_distance_aimant_get(float *a_pDistance);
int vehicule_distance_aimant_reset();
// Par convention, l'angle est négatif quand on tourne à gauche.
int vehicule_dir_set(float a_dir);
// Par convention, la vitesse est négative quand on recule.
int vehicule_throttle_set(float a_throttle);
// Est ce que le pilote utilise la télécommande ? 0 : non / 1 : oui
int radio_isThereCommand();

#endif /* INC_RADIO_COMMANDES_H_ */
