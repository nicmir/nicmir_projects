/*
 * TF Mini Lidar.c
 *
 *  Created on: 18/07/2018
 *      Author: Nicolas
 */

#include "main.h"
#include "tfminiplus.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

// Timeout de 1000 ms
#define TIMEOUT 1000

uint32_t buffer_uart5[10];

// D�finition d'une structure qui liste tous les �l�ments d'un capteur.
// Ceci permet d'instancier plusieurs capteurs dans un m�me logiciel facilement
typedef struct
{
	// Pointeur vers l'uart utilis�
	UART_HandleTypeDef *pHuart;

	// La taille max d'une trame, dans tout le protocole, est de 9 octets
	uint8_t serialBuffer[32];
	// Derni�re distance en cm
	int32_t distance;
	// Intensit� du rayon re�u
	int32_t strength;
	// Temperature du sensor en centigrade
	int32_t temperature;

	// Format : 00.version.revision.edition
	int32_t version;

	// Frequence d'acquisition
	int32_t framerate;
	// Frequence de la liaison serie
	int32_t baudrate;
	// Type de format de donn�es
	eLidarOutputFormat outputFormat;

	// Semaphore pour attendre la r�ponse du lidar � une commande
	int32_t semaphore;

	uint32_t nb_irq;

} stMiniLidar;

// Instanciation des capteurs
stMiniLidar miniLidarDroit;
stMiniLidar miniLidarGauche;
stMiniLidar miniLidarAvant;
stMiniLidar miniLidarHaut;

// Dans la routine d'IRQ du DMA, on positionne le num�ro du capteur rattach� au DMA
// Remarque g�n�rale, plutot que de typer numCapteur en int, on pourrait mettre un enum
void tfminiplusIrq(LIDAR_ID a_numCapteur)
{
	int distance, strength, temp;
	uint32_t checksum, checksum_ref;
	stMiniLidar *pLidar;

	if(a_numCapteur == MINILIDAR_DROIT)
		pLidar = &miniLidarDroit;
	else if(a_numCapteur == MINILIDAR_GAUCHE)
		pLidar = &miniLidarGauche;
	else if(a_numCapteur == MINILIDAR_HAUT)
		pLidar = &miniLidarHaut;
	else if(a_numCapteur == MINILIDAR_AVANT)
		pLidar = &miniLidarAvant;
	else
		pLidar = 0;
//	if(huart == miniLidarDroit.pHuart)
//		pLidar = &miniLidarDroit;
//	else if(huart == miniLidarGauche.pHuart)
//		pLidar = &miniLidarGauche;
//	else if(huart == miniLidarHaut.pHuart)
//		pLidar = &miniLidarHaut;
//	else if(huart == miniLidarAvant.pHuart)
//		pLidar = &miniLidarAvant;
//	else
//		pLidar = 0;

	if(pLidar != 0)
	{
		// On v�rifie l'ent�te
		if(pLidar->serialBuffer[0] == 0x59)
		{
			// C'est une trame de donn�e

			// On v�rifie que le deuxi�me octet est correct
			// Ce driver ne g�re que le format standard mais pas le format Pixhawk
			if(pLidar->serialBuffer[1] == 0x59)
			{
				// On v�rifie le checksum
				checksum = 0;
				for (int i=0; i<8;i++) checksum += pLidar->serialBuffer[i];
				checksum &= 0xFF;
				checksum_ref = pLidar->serialBuffer[8];
				if(checksum == checksum_ref)
				{
					// La trame est correcte, on traite les donn�es
					// On constitue les valeurs r�elles
					distance = pLidar->serialBuffer[2] + (pLidar->serialBuffer[3] << 8);
					strength = pLidar->serialBuffer[4] + (pLidar->serialBuffer[5] << 8);
					temp =     pLidar->serialBuffer[6] + (pLidar->serialBuffer[7] << 8);
//					// Si la force du signal de retour est suffisante, la donn�e de distance est valable
//					if((strength>100) && (strength!=65535))
//					{
//						pLidar->distance = distance;
//						pLidar->strength = strength;
//					}
//					// Sinon, on laisse les valeurs de distance et d'intensit� pr�c�dentes
//					pLidar->temperature = temp;
					pLidar->distance = distance;
					pLidar->strength = strength;
					pLidar->temperature = temp;
					pLidar->nb_irq += 1;
				}
			}
		} else if (pLidar->serialBuffer[0] == 0x5A)
		{
			// C'est une trame de r�ponse � une commande
			if((pLidar->serialBuffer[1] == 0x07) &&
			   (pLidar->serialBuffer[2] == 0x01))
			{
				// On v�rifie le checksum
				checksum = 0;
				for (int i=0; i<6;i++)
					checksum += pLidar->serialBuffer[i];
				checksum &= 0xFF;
				checksum_ref = pLidar->serialBuffer[6];
				if(checksum == checksum_ref)
				{
					// La trame est correcte, on traite les donn�es
					// On r�cup�re les num�ros de version
					// Format 00.V3.V2.V1
					pLidar->version = pLidar->serialBuffer[3] + (pLidar->serialBuffer[4] << 8) + (pLidar->serialBuffer[5] << 16);
					pLidar->semaphore++;
				}
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x02) &&
					(pLidar->serialBuffer[3] == 0x00) &&
					(pLidar->serialBuffer[4] == 0x60))
			{
				// Le capteur va reseter
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x02) &&
					(pLidar->serialBuffer[3] == 0x01) &&
					(pLidar->serialBuffer[4] == 0x61))
			{
				// Le capteur refuse de reseter
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x06) &&
					(pLidar->serialBuffer[2] == 0x03))
			{
				// On v�rifie le checksum
				checksum = 0;
				for (int i=0; i<5;i++)
					checksum += pLidar->serialBuffer[i];
				checksum &= 0xFF;
				checksum_ref = pLidar->serialBuffer[5];
				if(checksum == checksum_ref)
				{
					// La trame est correcte, on traite les donn�es
					// On r�cup�re le Frame Rate
					pLidar->framerate = pLidar->serialBuffer[3] + (pLidar->serialBuffer[4] << 8);

					pLidar->semaphore++;
				}
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x05) &&
					(pLidar->serialBuffer[3] == 0x01) &&
					(pLidar->serialBuffer[4] == 0x65))
			{
				// On r�cup�re le Format des donn�es
				pLidar->outputFormat = standard_cm;
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x05) &&
					(pLidar->serialBuffer[3] == 0x02) &&
					(pLidar->serialBuffer[4] == 0x66))
			{
				// On r�cup�re le Format des donn�es
				pLidar->outputFormat = pixhawk;
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x05) &&
					(pLidar->serialBuffer[3] == 0x03) &&
					(pLidar->serialBuffer[4] == 0x67))
			{
				// On r�cup�re le Format des donn�es
				pLidar->outputFormat = standard_mm;
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x08) &&
					(pLidar->serialBuffer[2] == 0x06))
			{
				// On v�rifie le checksum
				checksum = 0;
				for (int i=0; i<7;i++)
					checksum += pLidar->serialBuffer[i];
				checksum &= 0xFF;
				checksum_ref = pLidar->serialBuffer[7];
				if(checksum == checksum_ref)
				{
					// La trame est correcte, on traite les donn�es
					// On r�cup�re le Baud Rate
					pLidar->baudrate = pLidar->serialBuffer[3] + (pLidar->serialBuffer[4] << 8) +
							(pLidar->serialBuffer[5] << 16) + (pLidar->serialBuffer[6] << 24);
					pLidar->semaphore++;
				}
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x07) &&
					(pLidar->serialBuffer[3] == 0x00) &&
					(pLidar->serialBuffer[4] == 0x66))
			{
				// Arr�te la g�n�ration automatique des distances
				// Les distances ne sont fournies que sur demande
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x07) &&
					(pLidar->serialBuffer[3] == 0x01) &&
					(pLidar->serialBuffer[4] == 0x67))
			{
				// D�marre la g�n�ration automatique des distances
				// Les distances sont fournies r�guli�rement. La fr�quence est fournie par le Frame Rate
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x10) &&
					(pLidar->serialBuffer[3] == 0x00) &&
					(pLidar->serialBuffer[4] == 0x6E))
			{
				// La demande de restauration des param�tres d'usine est accept�e
				pLidar->baudrate = 115200;
				pLidar->framerate = 100;
				pLidar->outputFormat = standard_cm;
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x10) &&
					(pLidar->serialBuffer[3] == 0x01) &&
					(pLidar->serialBuffer[4] == 0x6F))
			{
				// La demande de restauration des param�tres d'usine est refus�e
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x22) &&
					(pLidar->serialBuffer[3] == 0x00) &&
					(pLidar->serialBuffer[4] == 0x6F))
			{
				// La demande de sauvegarde des parametres courant est accept�e
				pLidar->semaphore++;
			}
			else if((pLidar->serialBuffer[1] == 0x05) &&
					(pLidar->serialBuffer[2] == 0x22) &&
					(pLidar->serialBuffer[3] == 0x01) &&
					(pLidar->serialBuffer[4] == 0x70))
			{
				// La demande de sauvegarde des parametres courant est refus�e
				pLidar->semaphore++;
			}
		}
		// Sinon, c'est un format inconnu. On j�te la trame.

		// On r�arme le DMA
		HAL_UART_Receive_DMA(pLidar->pHuart, pLidar->serialBuffer, 9);

	}

}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	int distance, strength, temp;
//	uint32_t checksum, checksum_ref;
//	stMiniLidar *pLidar;
//
////	if(a_numCapteur == MINILIDAR_DROIT)
////		pLidar = &miniLidarDroit;
////	else if(a_numCapteur == MINILIDAR_GAUCHE)
////		pLidar = &miniLidarGauche;
////	else if(a_numCapteur == MINILIDAR_HAUT)
////		pLidar = &miniLidarHaut;
////	else if(a_numCapteur == MINILIDAR_AVANT)
////		pLidar = &miniLidarAvant;
////	else
////		pLidar = 0;
//	if(huart == miniLidarDroit.pHuart)
//		pLidar = &miniLidarDroit;
//	else if(huart == miniLidarGauche.pHuart)
//		pLidar = &miniLidarGauche;
//	else if(huart == miniLidarHaut.pHuart)
//		pLidar = &miniLidarHaut;
//	else if(huart == miniLidarAvant.pHuart)
//		pLidar = &miniLidarAvant;
//	else
//		pLidar = 0;
//
//	if(pLidar != 0)
//	{
//		// On v�rifie l'ent�te
//		if(pLidar->serialBuffer[0] == 0x59)
//		{
//			// C'est une trame de donn�e
//
//			// On v�rifie que le deuxi�me octet est correct
//			// Ce driver ne g�re que le format standard mais pas le format Pixhawk
//			if(pLidar->serialBuffer[1] == 0x59)
//			{
//				// On v�rifie le checksum
//				checksum = 0;
//				for (int i=0; i<8;i++) checksum += pLidar->serialBuffer[i];
//				checksum &= 0xFF;
//				checksum_ref = pLidar->serialBuffer[8];
//				if(checksum == checksum_ref)
//				{
//					// La trame est correcte, on traite les donn�es
//					// On constitue les valeurs r�elles
//					distance = pLidar->serialBuffer[2] + (pLidar->serialBuffer[3] << 8);
//					strength = pLidar->serialBuffer[4] + (pLidar->serialBuffer[5] << 8);
//					temp =     pLidar->serialBuffer[6] + (pLidar->serialBuffer[7] << 8);
////					// Si la force du signal de retour est suffisante, la donn�e de distance est valable
////					if((strength>100) && (strength!=65535))
////					{
////						pLidar->distance = distance;
////						pLidar->strength = strength;
////					}
////					// Sinon, on laisse les valeurs de distance et d'intensit� pr�c�dentes
////					pLidar->temperature = temp;
//					pLidar->distance = distance;
//					pLidar->strength = strength;
//					pLidar->temperature = temp;
//				}
//			}
//		} else if (pLidar->serialBuffer[0] == 0x5A)
//		{
//			// C'est une trame de r�ponse � une commande
//			if((pLidar->serialBuffer[1] == 0x07) &&
//			   (pLidar->serialBuffer[2] == 0x01))
//			{
//				// On v�rifie le checksum
//				checksum = 0;
//				for (int i=0; i<6;i++)
//					checksum += pLidar->serialBuffer[i];
//				checksum &= 0xFF;
//				checksum_ref = pLidar->serialBuffer[6];
//				if(checksum == checksum_ref)
//				{
//					// La trame est correcte, on traite les donn�es
//					// On r�cup�re les num�ros de version
//					// Format 00.V3.V2.V1
//					pLidar->version = pLidar->serialBuffer[3] + (pLidar->serialBuffer[4] << 8) + (pLidar->serialBuffer[5] << 16);
//					pLidar->semaphore++;
//				}
//			}
//			else if((pLidar->serialBuffer[1] == 0x05) &&
//					(pLidar->serialBuffer[2] == 0x02) &&
//					(pLidar->serialBuffer[3] == 0x00) &&
//					(pLidar->serialBuffer[4] == 0x60))
//			{
//				// Le capteur va reseter
//				pLidar->semaphore++;
//			}
//			else if((pLidar->serialBuffer[1] == 0x05) &&
//					(pLidar->serialBuffer[2] == 0x02) &&
//					(pLidar->serialBuffer[3] == 0x01) &&
//					(pLidar->serialBuffer[4] == 0x61))
//			{
//				// Le capteur refuse de reseter
//				pLidar->semaphore++;
//			}
//			else if((pLidar->serialBuffer[1] == 0x06) &&
//					(pLidar->serialBuffer[2] == 0x03))
//			{
//				// On v�rifie le checksum
//				checksum = 0;
//				for (int i=0; i<5;i++)
//					checksum += pLidar->serialBuffer[i];
//				checksum &= 0xFF;
//				checksum_ref = pLidar->serialBuffer[5];
//				if(checksum == checksum_ref)
//				{
//					// La trame est correcte, on traite les donn�es
//					// On r�cup�re le Frame Rate
//					pLidar->framerate = pLidar->serialBuffer[3] + (pLidar->serialBuffer[4] << 8);
//
//					pLidar->semaphore++;
//				}
//			}
//			else if((pLidar->serialBuffer[1] == 0x05) &&
//					(pLidar->serialBuffer[2] == 0x05) &&
//					(pLidar->serialBuffer[3] == 0x01) &&
//					(pLidar->serialBuffer[4] == 0x65))
//			{
//				// On r�cup�re le Format des donn�es
//				pLidar->outputFormat = standard_cm;
//				pLidar->semaphore++;
//			}
//			else if((pLidar->serialBuffer[1] == 0x05) &&
//					(pLidar->serialBuffer[2] == 0x05) &&
//					(pLidar->serialBuffer[3] == 0x02) &&
//					(pLidar->serialBuffer[4] == 0x66))
//			{
//				// On r�cup�re le Format des donn�es
//				pLidar->outputFormat = pixhawk;
//				pLidar->semaphore++;
//			}
//			else if((pLidar->serialBuffer[1] == 0x05) &&
//					(pLidar->serialBuffer[2] == 0x05) &&
//					(pLidar->serialBuffer[3] == 0x03) &&
//					(pLidar->serialBuffer[4] == 0x67))
//			{
//				// On r�cup�re le Format des donn�es
//				pLidar->outputFormat = standard_mm;
//				pLidar->semaphore++;
//			}
//			else if((pLidar->serialBuffer[1] == 0x08) &&
//					(pLidar->serialBuffer[2] == 0x06))
//			{
//				// On v�rifie le checksum
//				checksum = 0;
//				for (int i=0; i<7;i++)
//					checksum += pLidar->serialBuffer[i];
//				checksum &= 0xFF;
//				checksum_ref = pLidar->serialBuffer[7];
//				if(checksum == checksum_ref)
//				{
//					// La trame est correcte, on traite les donn�es
//					// On r�cup�re le Baud Rate
//					pLidar->baudrate = pLidar->serialBuffer[3] + (pLidar->serialBuffer[4] << 8) +
//							(pLidar->serialBuffer[5] << 16) + (pLidar->serialBuffer[6] << 24);
//					pLidar->semaphore++;
//				}
//			}
//			else if((pLidar->serialBuffer[1] == 0x05) &&
//					(pLidar->serialBuffer[2] == 0x07) &&
//					(pLidar->serialBuffer[3] == 0x00) &&
//					(pLidar->serialBuffer[4] == 0x66))
//			{
//				// Arr�te la g�n�ration automatique des distances
//				// Les distances ne sont fournies que sur demande
//				pLidar->semaphore++;
//			}
//			else if((pLidar->serialBuffer[1] == 0x05) &&
//					(pLidar->serialBuffer[2] == 0x07) &&
//					(pLidar->serialBuffer[3] == 0x01) &&
//					(pLidar->serialBuffer[4] == 0x67))
//			{
//				// D�marre la g�n�ration automatique des distances
//				// Les distances sont fournies r�guli�rement. La fr�quence est fournie par le Frame Rate
//				pLidar->semaphore++;
//			}
//			else if((pLidar->serialBuffer[1] == 0x05) &&
//					(pLidar->serialBuffer[2] == 0x10) &&
//					(pLidar->serialBuffer[3] == 0x00) &&
//					(pLidar->serialBuffer[4] == 0x6E))
//			{
//				// La demande de restauration des param�tres d'usine est accept�e
//				pLidar->baudrate = 115200;
//				pLidar->framerate = 100;
//				pLidar->outputFormat = standard_cm;
//				pLidar->semaphore++;
//			}
//			else if((pLidar->serialBuffer[1] == 0x05) &&
//					(pLidar->serialBuffer[2] == 0x10) &&
//					(pLidar->serialBuffer[3] == 0x01) &&
//					(pLidar->serialBuffer[4] == 0x6F))
//			{
//				// La demande de restauration des param�tres d'usine est refus�e
//				pLidar->semaphore++;
//			}
//			else if((pLidar->serialBuffer[1] == 0x05) &&
//					(pLidar->serialBuffer[2] == 0x22) &&
//					(pLidar->serialBuffer[3] == 0x00) &&
//					(pLidar->serialBuffer[4] == 0x6F))
//			{
//				// La demande de sauvegarde des parametres courant est accept�e
//				pLidar->semaphore++;
//			}
//			else if((pLidar->serialBuffer[1] == 0x05) &&
//					(pLidar->serialBuffer[2] == 0x22) &&
//					(pLidar->serialBuffer[3] == 0x01) &&
//					(pLidar->serialBuffer[4] == 0x70))
//			{
//				// La demande de sauvegarde des parametres courant est refus�e
//				pLidar->semaphore++;
//			}
//		}
//		// Sinon, c'est un format inconnu. On j�te la trame.
//
//		// On r�arme le DMA
//		HAL_UART_Receive_DMA(pLidar->pHuart, pLidar->serialBuffer, 9);
//
//	}
//}

int tfminiplus_getLastAcquisition(LIDAR_ID a_numCapteur, int32_t *a_pDistance, int32_t *a_pStrength, int32_t *a_pTemperature)
{
	int erreur;
	stMiniLidar *pLidar;

	if(a_numCapteur == MINILIDAR_DROIT)
		pLidar = &miniLidarDroit;
	else if(a_numCapteur == MINILIDAR_GAUCHE)
		pLidar = &miniLidarGauche;
	else if(a_numCapteur == MINILIDAR_HAUT)
		pLidar = &miniLidarHaut;
	else if(a_numCapteur == MINILIDAR_AVANT)
		pLidar = &miniLidarAvant;
	else
		pLidar = 0;

	if(pLidar != 0)
	{
		// On renvoie la distance mesur�e par le premier capteur
		if(pLidar->distance == -2)
		{
			*a_pDistance = -2;
			*a_pStrength = 0;
			*a_pTemperature = 0;
		}
		else if((pLidar->strength >= 100) && (pLidar->strength != 65535))
		{
			*a_pDistance = pLidar->distance;
			*a_pStrength = pLidar->strength;
			*a_pTemperature = pLidar->temperature;
		}
		else
		{
			*a_pDistance = -1;
		}
		erreur = 0;

		// R�initialisation du strength pour d�tecter lorsque le lidar arr�te d'envoyer des valeurs valides
		// En gros, le lidar envoie des captures toutes les 10 ms.
		// La valeur du strength du rayon de retour permet de savoir si la mesure est valide.
		// On peut avoir strength 250 250 10 10 10 10 10 10
		// Si le logiciel applicatif prend la mesure apr�s le deuxi�me 250, et qu'il prend la deuxi�me mesure apr�s le 4ieme 10,
		// alors le driver pourrait renvoyer la derni�re valeur valide, mais elle est tr�s ancienne.
		pLidar->distance = -2;
	}
	else
		// Numero de capteur inconnu
		erreur = -1;

	return erreur;
}

int tfminiplus_setOutputFormat(LIDAR_ID a_numCapteur, eLidarOutputFormat *a_pFormat)
{
	int erreur;
	int attente;
	uint8_t commande[8];
	stMiniLidar *pLidar;

	if(a_numCapteur == MINILIDAR_DROIT)
		pLidar = &miniLidarDroit;
	else if(a_numCapteur == MINILIDAR_GAUCHE)
		pLidar = &miniLidarGauche;
	else if(a_numCapteur == MINILIDAR_HAUT)
		pLidar = &miniLidarHaut;
	else if(a_numCapteur == MINILIDAR_AVANT)
		pLidar = &miniLidarAvant;
	else
		pLidar = 0;

	if(pLidar != 0)
	{
		if(*a_pFormat == standard_cm)
		{
			// Configure le format de sortie
			commande[0] = 0x5A; commande[1] = 0x05;
			commande[2] = 0x05; commande[3] = 0x01;
			commande[4] = 0x65;
		}
		if(*a_pFormat == pixhawk)
		{
			// Configure le format de sortie
			commande[0] = 0x5A; commande[1] = 0x05;
			commande[2] = 0x05; commande[3] = 0x02;
			commande[4] = 0x66;
		}
		if(*a_pFormat == standard_mm)
		{
			// Configure le format de sortie
			commande[0] = 0x5A; commande[1] = 0x05;
			commande[2] = 0x05; commande[3] = 0x06;
			commande[4] = 0x6A;
		}
		HAL_UART_Transmit(pLidar->pHuart, commande, 5, 100000);

		// Attente de la r�ponse
		pLidar->semaphore = 0;
		attente = 0;
		do {
			attente++;
			HAL_Delay(1);
		} while((pLidar->semaphore==0) && (attente<TIMEOUT));

		if(attente < TIMEOUT)
		{
			// Verification que le nouveau format est celui demand�
			if(*a_pFormat == pLidar->outputFormat)
				erreur = 0;
			else
			{
				*a_pFormat = pLidar->outputFormat;
				erreur = -1;
			}
		}
		else
			// Timeout
			erreur = -1;
	}
	else
		// Numero de capteur inconnu
		erreur = -1;

	return erreur;
}

int tfminiplus_getFramerate(LIDAR_ID a_numCapteur, int32_t *a_pFramerate)
{
	int erreur;

	if(a_numCapteur == MINILIDAR_DROIT)
	{
		*a_pFramerate = miniLidarDroit.framerate;
		erreur = 0;
	}
	else if(a_numCapteur == MINILIDAR_GAUCHE)
	{
		*a_pFramerate = miniLidarGauche.framerate;
		erreur = 0;
	}
	else if(a_numCapteur == MINILIDAR_HAUT)
	{
		*a_pFramerate = miniLidarHaut.framerate;
		erreur = 0;
	}
	else if(a_numCapteur == MINILIDAR_AVANT)
	{
		*a_pFramerate = miniLidarHaut.framerate;
		erreur = 0;
	}
	else
		// Numero de capteur inconnu
		erreur = -1;

	return erreur;
}

int tfminiplus_getBaudrate(LIDAR_ID a_numCapteur, int32_t *a_pBaudrate)
{
	int erreur;

	if(a_numCapteur == MINILIDAR_DROIT)
	{
		*a_pBaudrate = miniLidarDroit.baudrate;
		erreur = 0;
	}
	else if(a_numCapteur == MINILIDAR_GAUCHE)
	{
		*a_pBaudrate = miniLidarGauche.baudrate;
		erreur = 0;
	}
	else if(a_numCapteur == MINILIDAR_HAUT)
	{
		*a_pBaudrate = miniLidarHaut.baudrate;
		erreur = 0;
	}
	else if(a_numCapteur == MINILIDAR_AVANT)
	{
		*a_pBaudrate = miniLidarHaut.baudrate;
		erreur = 0;
	}
	else
		// Numero de capteur inconnu
		erreur = -1;

	return erreur;
}

int tfminiplus_getVersion(LIDAR_ID a_numCapteur, int32_t *a_pVersion)
{
	int erreur;
	int attente;
	uint8_t commande[8];
	stMiniLidar *pLidar;

	if(a_numCapteur == MINILIDAR_DROIT)
		pLidar = &miniLidarDroit;
	else if(a_numCapteur == MINILIDAR_GAUCHE)
		pLidar = &miniLidarGauche;
	else if(a_numCapteur == MINILIDAR_HAUT)
		pLidar = &miniLidarHaut;
	else if(a_numCapteur == MINILIDAR_AVANT)
		pLidar = &miniLidarAvant;
	else
		pLidar = 0;

	if(pLidar != 0)
	{
		// Demande de la version au capteur
		commande[0] = 0x5A; commande[1] = 0x04; commande[2] = 0x01; commande[3] = 0x5F;
		HAL_UART_Transmit(pLidar->pHuart, commande, 4, 100000);

		// Attente de la r�ponse
		pLidar->semaphore = 0;
		attente = 0;
		do {
			attente++;
			HAL_Delay(1);
		} while((pLidar->semaphore==0) && (attente<TIMEOUT));

		if(attente < TIMEOUT)
		{
			// Fourniture de la version
			*a_pVersion = pLidar->version;

			erreur = 0;
		}
		else
			erreur = -1;
	}
	else
		// Numero de capteur inconnu
		erreur = -1;

	return erreur;
}

int tfminiplus_init()
{
	HAL_StatusTypeDef retour;

	// Initialisation des parametres
	miniLidarDroit.pHuart = &huart5;
	miniLidarDroit.distance = -2;
	miniLidarDroit.strength = 0;
	miniLidarDroit.temperature = 0;
	miniLidarDroit.baudrate = 115200;
	miniLidarDroit.framerate = 100;
	miniLidarDroit.semaphore = 0;
	miniLidarDroit.nb_irq = 0;

	miniLidarGauche.pHuart = &huart7;
	miniLidarGauche.distance = -2;
	miniLidarGauche.strength = 0;
	miniLidarGauche.temperature = 0;
	miniLidarGauche.baudrate = 115200;
	miniLidarGauche.framerate = 100;
	miniLidarGauche.semaphore = 0;
	miniLidarGauche.nb_irq = 0;

	miniLidarHaut.pHuart = &huart8;
	miniLidarHaut.distance = -2;
	miniLidarHaut.strength = 0;
	miniLidarHaut.temperature = 0;
	miniLidarHaut.baudrate = 115200;
	miniLidarHaut.framerate = 100;
	miniLidarHaut.semaphore = 0;
	miniLidarHaut.nb_irq = 0;

	miniLidarAvant.pHuart = &huart4;
	miniLidarAvant.distance = -2;
	miniLidarAvant.strength = 0;
	miniLidarAvant.temperature = 0;
	miniLidarAvant.baudrate = 115200;
	miniLidarAvant.framerate = 100;
	miniLidarAvant.semaphore = 0;
	miniLidarAvant.nb_irq = 0;

	// Sequence de mise sous tension pour limiter l'appel de courant
	HAL_GPIO_WritePin(lid1_pwr_en_GPIO_Port, lid1_pwr_en_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lid2_pwr_en_GPIO_Port, lid2_pwr_en_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lid3_pwr_en_GPIO_Port, lid3_pwr_en_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lid4_pwr_en_GPIO_Port, lid4_pwr_en_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lid5_pwr_en_GPIO_Port, lid5_pwr_en_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(lid6_pwr_en_GPIO_Port, lid6_pwr_en_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);

	HAL_GPIO_WritePin(lid1_pwr_en_GPIO_Port, lid1_pwr_en_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	retour = HAL_UART_Receive_DMA(miniLidarAvant.pHuart, miniLidarAvant.serialBuffer, 26);
	HAL_Delay(900);

	HAL_GPIO_WritePin(lid2_pwr_en_GPIO_Port, lid2_pwr_en_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	retour = HAL_UART_Receive_DMA(miniLidarGauche.pHuart, miniLidarGauche.serialBuffer, 10);
	HAL_Delay(900);

	HAL_GPIO_WritePin(lid3_pwr_en_GPIO_Port, lid3_pwr_en_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);

	HAL_GPIO_WritePin(lid4_pwr_en_GPIO_Port, lid4_pwr_en_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);

	HAL_GPIO_WritePin(lid5_pwr_en_GPIO_Port, lid5_pwr_en_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	retour = HAL_UART_Receive_DMA(miniLidarDroit.pHuart, miniLidarDroit.serialBuffer, 10);
	HAL_Delay(900);

	HAL_GPIO_WritePin(lid6_pwr_en_GPIO_Port, lid6_pwr_en_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	retour = HAL_UART_Receive_DMA(miniLidarHaut.pHuart, miniLidarHaut.serialBuffer, 26);
	HAL_Delay(900);

	// D�but d'�coute
//	retour = HAL_UART_Receive_DMA(miniLidarGauche.pHuart, miniLidarGauche.serialBuffer, 9);
//	retour += HAL_UART_Receive_DMA(miniLidarDroit.pHuart, miniLidarDroit.serialBuffer, 9);
//	retour += HAL_UART_Receive_DMA(miniLidarHaut.pHuart, miniLidarHaut.serialBuffer, 9);
//	retour += HAL_UART_Receive_DMA(miniLidarAvant.pHuart, miniLidarAvant.serialBuffer, 9);

//	retour = HAL_UART_Receive(miniLidarDroit.pHuart, miniLidarDroit.serialBuffer, 9, HAL_MAX_DELAY);
//	retour = 0;

	return retour;
}


