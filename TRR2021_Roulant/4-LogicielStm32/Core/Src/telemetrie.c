/*
 * telemetrue.c
 *
 *  Created on: 30 déc. 2021
 *      Author: nmira
 */

#include "main.h"
#include "telemetrie.h"

#define SRAM1_BUFFER __attribute__((section(".mem_telemetrie")))
st_telemetrie gTelemetrie SRAM1_BUFFER;

// Cette fonction renvoi le pointeur sur l'élément dans le tableau qui est en cours d'écriture
st_tele_element *telemetrie_pt_enreg_en_cours()
{

	return &(gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours]);
}

// Cette fonction passe à l'élément suivant et renvoi le pointeur sur l'élément dans le tableau qui est en cours d'écriture
// Cette fonction renvoi -1 si le tableau est plein
st_tele_element *telemetrie_pt_enreg_suivant(int *a_pErreur)
{
	uint16_t futur_index_enreg;

	*a_pErreur = 0;

	futur_index_enreg = gTelemetrie.index_enreg_en_cours + 1;

	// Pour passer à l'élément suivant, il faut que l'index + 1 soit :

	// - le pointeur soit dans la taille du tableau sinon passage à 0.
	if(futur_index_enreg > TELE_TAILLE_BUFFER)
		futur_index_enreg = 0;

	// - différent du pointeur de lecture (dans ce cas, le tableau est plein)
	if(futur_index_enreg == gTelemetrie.index_lecture_en_cours)
		*a_pErreur = -1;
	else
		gTelemetrie.index_enreg_en_cours = futur_index_enreg;

	// Initialisation des valeurs
	gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours].consigne_direction = 0.0;
	gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours].consigne_vitesse = 0.0;
	gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours].etat_automate_auto = 0;
	gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours].etat_automate_principal = 0;
	gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours].gyro_dps = 0.0;
	gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours].heading = 0.0;
	gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours].lidar_avant = 0;
	gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours].lidar_droit = 0;
	gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours].lidar_gauche = 0;
	gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours].lidar_haut = 0;
	gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours].mesure_distance = 0.0;
	gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours].mesure_vitesse = 0.0;

	return &(gTelemetrie.enregistrements[gTelemetrie.index_enreg_en_cours]);
}

st_tele_element *telemetrie_pt_lecture_en_cours()
{
	return &gTelemetrie.enregistrements[gTelemetrie.index_lecture_en_cours];

}

// Cette fonction donne le pointeur suivant sur l'élément à lire, sauf si le tableau est vide
st_tele_element *telemetrie_pt_lecture_suivant(int *a_pErreur)
{
	uint16_t futur_index_lecture;

	*a_pErreur = 0;

	futur_index_lecture = gTelemetrie.index_lecture_en_cours + 1;

	// Pour passer à l'élément suivant, il faut que l'index + 1 soit :

	// - le pointeur soit dans la taille du tableau sinon passage à 0.
	if(futur_index_lecture > TELE_TAILLE_BUFFER)
		futur_index_lecture = 0;

	// - différent du pointeur de lecture (dans ce cas, le tableau est plein)
	if(futur_index_lecture == gTelemetrie.index_enreg_en_cours)
		*a_pErreur = -1;
	else
		gTelemetrie.index_lecture_en_cours = futur_index_lecture;

	return &gTelemetrie.enregistrements[gTelemetrie.index_lecture_en_cours];
}

// La stratégie ici est de remplir le tableau jusqu'à ce qu'il soit plein.
// Il n'y a pas d'écrasement des données les plus anciennes.
void telemetrie_init()
{
	gTelemetrie.index_enreg_en_cours = 0;
	gTelemetrie.index_lecture_en_cours = TELE_TAILLE_BUFFER - 1;
}
