/*
 * radio_commandes.c
 *
 *  Created on: Oct 31, 2021
 *      Author: nmira
 */

#include "main.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim12;

typedef struct {
	uint32_t last_time;
	uint32_t period;
	uint32_t duty_cycle;
	uint32_t duty_cycle_default;
} st_pwm_infos;

// La différence entre l'axe du véhicule et l'axe de la direction. L'angle des roues.
// Ici l'angle quand on met la commande à 2000 us.
// Approximation à vérifier : on considère la commande linéaire (angle = commande * K).
#define VEHICULE_DIR_MAX   26.0 // ° - Mesuré sur la TT01
#define VEHICULE_SPEED_MAX 10.0 // m/s

static st_pwm_infos radio_dir;
static st_pwm_infos radio_throttle;
static st_pwm_infos radio_spare;

static st_pwm_infos vehicule_speedsensor;

static int32_t nb_impulsions_aimants = 0;

static float magnet_count = 4.0;
static float gear_ratio = 2.64;
static float wheel_perimeter = 0.204;
// Speed
static float actual_speed_ms;
static int32_t vitesse_mesuree = -1;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) // Callback for PWM input catpure
{
	if(htim==&htim3) // DIR from RX
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			radio_dir.period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			radio_dir.last_time = HAL_GetTick(); // timestamp last pulse
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			radio_dir.duty_cycle = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		}
	}
	else if(htim==&htim2) // THR from RX
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			radio_throttle.period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			radio_throttle.last_time = HAL_GetTick(); // timestamp last pulse
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			radio_throttle.duty_cycle = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		}
	}
	else if(htim==&htim4) // Spare - SpeedSensorAimant from vehicule
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			nb_impulsions_aimants++;
			vitesse_mesuree = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		}
	}
//	else if(htim==&htim12) // SpeedSensor from vehicule
//	{
//		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//		{
//			vehicule_speedsensor.period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//			vehicule_speedsensor.last_time = HAL_GetTick(); // timestamp last pulse
//		}
//		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//		{
//			vehicule_speedsensor.duty_cycle = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
//		}
//	}
}

// Par convention, l'angle est négatif quand on tourne à gauche.
int radio_dir_get(float *a_pDir)
{
	// Si la commande radio n'a pas été rafraichie depuis plus d'une seconde, on remonte une panne
	if( (HAL_GetTick() - radio_dir.last_time) > 1000)
		return -1;
	else
	{
		// A terme, vérifier que le duty_cycle est bien limité à 1000 - 2000 us.
		if(radio_dir.duty_cycle<1000) radio_dir.duty_cycle = 1000;
		if(radio_dir.duty_cycle>2000) radio_dir.duty_cycle = 2000;

		*a_pDir = ((float)(radio_dir.duty_cycle) - (float)(radio_dir.duty_cycle_default))*VEHICULE_DIR_MAX/500.0;
	}
	return 0;
}

// Par convention, la vitesse est négative quand on recule.
int radio_throttle_get(float *a_pThrottle)
{
	// Si la commande radio n'a pas été rafraichie depuis plus d'une seconde, on remonte une panne
	if((HAL_GetTick() - radio_throttle.last_time) > 1000)
		return -1;
	else
	{
		if(radio_throttle.duty_cycle<1000) radio_throttle.duty_cycle = 1000;
		if(radio_throttle.duty_cycle>2000) radio_throttle.duty_cycle = 2000;
		// A terme, vérifier que le duty_cycle est bien limité à 1000 - 2000 us.
		*a_pThrottle = ((float)(radio_throttle.duty_cycle) - (float)(radio_throttle.duty_cycle_default))*VEHICULE_SPEED_MAX/500.0;
	}
	return 0;
}

int radio_isThereCommand()
{
	int retour;
	float direction, throttle;

	// La fonction sert à savoir si on arrête le pilotage automatique, alors par défaut, on considère que le robot reçoit une commande de la télécommande
	retour = 1;

	// Mesure de vitesse et de direction
	radio_dir_get(&direction);
	radio_throttle_get(&throttle);
	if((abs(direction) < 3) && (abs(throttle) < 0.5))
		retour = 0;

	return retour;
}

// !!!! A travailler !!!!
int vehicule_speed_get(float *a_pSpeed)
{
	// Si la commande radio n'a pas été rafraichie depuis plus d'une seconde, on remonte une panne
	if((HAL_GetTick() - vehicule_speedsensor.last_time) > 1000)
		return -1;
	else
	{
		if(vehicule_speedsensor.period<2000) vehicule_speedsensor.period = 2000;
		// A terme, vérifier que le duty_cycle est bien limité à 1000 - 2000 us.
		//*a_pSpeed = (float)(vehicule_speedsensor.period)*VEHICULE_SPEED_MAX/500.0;
		*a_pSpeed = VEHICULE_SPEED_MAX*(2000.0/(float)(vehicule_speedsensor.period));
	}
	return 0;
}

// !!!! A travailler !!!!
int vehicule_speed_aimant_get(float *a_pSpeed)
{
	// Speed computation
	*a_pSpeed =  100000.0/(magnet_count*(float)(vitesse_mesuree+1)) / gear_ratio * wheel_perimeter;

	return 0;
}

// !!!! A travailler !!!!
int vehicule_distance_aimant_get(float *a_pDistance)
{
	// Distance
	*a_pDistance =  ( (float)(nb_impulsions_aimants) / magnet_count ) / gear_ratio * wheel_perimeter; // m

	return 0;
}

// !!!! A travailler !!!!
int vehicule_distance_aimant_reset()
{
	nb_impulsions_aimants = 0;

	return 0;
}

// Par convention, l'angle est négatif quand on tourne à gauche.
int vehicule_dir_set(float a_dir)
{
	if((a_dir>VEHICULE_DIR_MAX) || (a_dir<-VEHICULE_DIR_MAX))
		return -1;
	else
	{
		// A terme, vérifier que le duty_cycle est bien limité à 1000 - 2000 us.
		htim1.Instance->CCR2 = 1500 + (int32_t)((a_dir/VEHICULE_DIR_MAX)*500.0);
	}

	return 0;
}

// Par convention, la vitesse est négative quand on recule.
int vehicule_throttle_set(float a_throttle)
{
	if((a_throttle>VEHICULE_SPEED_MAX) || (a_throttle<-VEHICULE_SPEED_MAX))
		return -1;
	else
		// A terme, vérifier que le duty_cycle est bien limité à 1000 - 2000 us.
		htim1.Instance->CCR1 = 1500 + (int32_t)((a_throttle/VEHICULE_SPEED_MAX)*500.0);

	return 0;
}

void init_radio_commandes()
{
	// Initialisation de toutes les structures
	radio_dir.last_time = 0;
	radio_dir.period = 0;
	radio_dir.duty_cycle = 0;

	radio_throttle.last_time = 0;
	radio_throttle.period = 0;
	radio_throttle.duty_cycle = 0;

	radio_spare.last_time = 0;
	radio_spare.period = 0;
	radio_spare.duty_cycle = 0;

	vehicule_speedsensor.last_time = 0;
	vehicule_speedsensor.period = 0;
	vehicule_speedsensor.duty_cycle = 0;

	nb_impulsions_aimants = 0;

	// Démarrage des Timers d'acquisition
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);

	// Détermination des valeurs médianes sur la télécommande.
	// Il s'avère que lorsque l'on relache la télécommande le point milieu de la gachette de vitesse
	// et de la direction.
	// On attend une seconde puis on mesure les valeurs par défaut
	HAL_Delay(1000);
	radio_dir.duty_cycle_default = radio_dir.duty_cycle;
	radio_throttle.duty_cycle_default = radio_throttle.duty_cycle;
	// TODO : c'est moins pire, mais ce n'est pas encore génial. Il faudrai peut être prévoir une courbe en expo comme sur les télécommandes de planeur

	// Vitesse et direction à 0
	vehicule_dir_set(0.0);
	vehicule_throttle_set(0.0);
	HAL_Delay(10);
	vehicule_speedsensor.duty_cycle_default = vehicule_speedsensor.duty_cycle;

	// Démarrage du Timer de commande
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

}


