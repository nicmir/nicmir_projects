/*
 * telemetrie.h
 *
 *  Created on: 30 déc. 2021
 *      Author: nmira
 */

#ifndef INC_TELEMETRIE_H_
#define INC_TELEMETRIE_H_

typedef struct {
	// 4 mots de 32 bits
	float consigne_vitesse;
	float consigne_direction;
	float mesure_vitesse;
	float mesure_distance;

	// 4 mots de 32 bits
	int32_t lidar_droit;
	int32_t lidar_gauche;
	int32_t lidar_avant;
	int32_t lidar_haut;

	// 4 mots de 32 bits
	uint8_t etat_automate_principal;
	uint8_t etat_automate_auto;
	uint8_t spare0_8bits;
	uint8_t spare1_8bits;

	float heading;
	float gyro_dps;
	int32_t spare3_32bits;
} st_tele_element;

// Nous dédions la SRAM1 de 240 ko à la télémétrie.
// Chaque élément de télémétrie fait 12 mots de 32 bits.
// Nous pouvons donc stocker 5120 éléments.
#define TELE_TAILLE_BUFFER 5000
typedef struct {
	st_tele_element enregistrements[TELE_TAILLE_BUFFER];

	uint16_t index_enreg_en_cours;
	uint16_t index_lecture_en_cours;
} st_telemetrie;

st_tele_element *telemetrie_pt_lecture_en_cours();
st_tele_element *telemetrie_pt_lecture_suivant(int *a_pErreur);
st_tele_element *telemetrie_pt_enreg_en_cours();
st_tele_element *telemetrie_pt_enreg_suivant(int *a_pErreur);
void telemetrie_init();

#endif /* INC_TELEMETRIE_H_ */
