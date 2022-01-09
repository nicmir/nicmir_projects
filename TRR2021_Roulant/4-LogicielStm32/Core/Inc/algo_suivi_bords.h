/*
 * algo_suivi_bords.h
 *
 *  Created on: 25 déc. 2021
 *      Author: nmira
 */

#ifndef INC_ALGO_SUIVI_BORDS_H_
#define INC_ALGO_SUIVI_BORDS_H_

typedef enum { automate_auto_depart, automate_auto_stop, automate_auto_decouverte, automate_auto_fini } eEtatsAutomateAutomatique;

typedef struct {
	pid_context_t pidVitesse;
	pid_context_t pidDirection;

	int32_t travelledDistance;  // en cm
	uint32_t elapsedTime;       // en ms

	float vitesse_cible;        // en m/s
	float vitesse_cmd_courante; // en m/s

} st_context_robot;

void algo_init(st_context_robot *a_pRobot);
void algo_decouverte(st_context_robot *a_pRobot, float a_deltaT);

#endif /* INC_ALGO_SUIVI_BORDS_H_ */
