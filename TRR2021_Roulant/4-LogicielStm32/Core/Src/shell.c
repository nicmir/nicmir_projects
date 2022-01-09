#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "math.h"
#include "radio_commandes.h"
#include "tfminiplus.h"
#include "imu.h"
#include "parametres_configuration.h"
#include "telemetrie.h"

#define PROMPT "TRR2022Roulant> "

#define SHELL_MAX_HISTORIQUE 5
typedef struct {
	char commandes[SHELL_MAX_HISTORIQUE][50];
	int  commande_courante;
} shell_historique;

shell_historique sh_historique;

typedef enum {
	sh_normal,
	sh_fleche_en_cours,
	sh_fleche_confirmee,
	sh_code_fleche
} shell_etat;

//void affiche_float(float a_nombre)
//{
//    int entier;
//    int decimal;
//
//    entier = 0;
//    decimal = 0;
//
//    if(a_nombre < 0)
//    {
//        printf("-");
//        a_nombre = -1 * a_nombre;
//    }
//    entier = (int)(a_nombre);
//    if(entier > a_nombre)
//        decimal = (int)((a_nombre-(entier-1))*1000);
//    else
//        decimal = (int)((a_nombre-entier)*1000);
//
//    printf("%d,%03d", entier, decimal);
//}
//
//int saisie_float(float *a_nombre)
//{
//    int nb_car_valide;
//    float nombre;
//    int decimal, isNegative;
//    char caractere;
//
//    nombre = 0;
//    decimal = 0;
//    nb_car_valide = 0;
//    isNegative = 0;
//
//    do {
//        caractere = getchar();
//        if((caractere >= '0') && (caractere <= '9'))
//        {
//            if(decimal == 0)
//                nombre = nombre * 10 + (caractere - '0');
//            else
//            {
//                nombre = nombre + (float)((float)(caractere - '0') / (float)decimal);
//                decimal *= 10;
//            }
//            nb_car_valide++;
//        }
//        if((caractere == '.') || (caractere == ','))
//        {
//            decimal = 10;
//            nb_car_valide++;
//        }
//        if(caractere == '-')
//        	// On memorise le signe que l'on appliquera � la fin.
//        	// Car le signe est au d�but, et au d�but nombre = 0.
//        	// Si nous appliquions le signe maintenant, nous ferions -1 * 0 = 0 et nous perdrions le signe
//        	isNegative = 1;
//
//    } while (caractere != '\r');
//
//    if(isNegative == 1)	nombre *= -1;
//
//    *a_nombre = nombre;
//
//    return nb_car_valide;
//}

int saisie_nombre(int *a_nombre)
{
    int nb_car_valide;
    int nombre;
    char caractere;
    int signe;

    nombre = 0;
    nb_car_valide = 0;
    signe = 1;

    do {
        caractere = getchar();
        if((caractere == '-') && (nb_car_valide==0))
            signe = -1;
        if((caractere >= '0') && (caractere <= '9'))
        {
            nombre = nombre * 10 + (caractere - '0');
            nb_car_valide++;
        }
    } while (caractere != '\r');

    *a_nombre = nombre * signe;

    return nb_car_valide;
}

char *saisie_commande()
{
	int nb_car_valide, position, i;
    char caractere;
	shell_etat etat;
	char *pCommandeCourante;
	unsigned int pointeur_historique;

    nb_car_valide = 0;
    position = 0;
	etat = sh_normal;

    // Caractere fin de chaine
	pointeur_historique = sh_historique.commande_courante;
	pCommandeCourante = sh_historique.commandes[sh_historique.commande_courante];
	pCommandeCourante[position] = 0x0;

    do {
        caractere = getchar();

		if(etat == sh_normal)
		{
			if(caractere == 0x1b)
				etat = sh_fleche_en_cours;
		}
		else if(etat == sh_fleche_en_cours)
		{
			if(caractere == 0x5b)
				etat = sh_fleche_confirmee;
			else
				// Ce n'est pas le code pour une fleche
				// On jete le caractere 0x1b
				etat = sh_normal;
		}
		else if(etat == sh_fleche_confirmee)
		{
			if(caractere == 0x44)
			{
				// Fleche vers la gauche
				position -=1;
				etat = sh_code_fleche;
			}
			else if(caractere == 0x43)
			{
				// Fleche vers la droite
				position +=1;
				etat = sh_code_fleche;
			}
			else if(caractere == 0x41)
			{
				// Fleche vers le haut
				if(pointeur_historique == 0)
					pointeur_historique = SHELL_MAX_HISTORIQUE-1;
				else
					pointeur_historique = pointeur_historique-1;
				pCommandeCourante = sh_historique.commandes[pointeur_historique];
                printf("\r\n                                                  ");
				printf("\r%s%s", PROMPT, pCommandeCourante);
				position = strlen(pCommandeCourante);
				nb_car_valide = position;
				strcpy(sh_historique.commandes[sh_historique.commande_courante], pCommandeCourante);
				etat = sh_code_fleche;
			}
			else
				// Inconnu ou non gere
				etat = sh_normal;
		}
		else if(etat == sh_code_fleche)
		{
			if(caractere == 0x1b)
				etat = sh_fleche_en_cours;
			else
				etat = sh_normal;
		}

		if(etat == sh_normal)
		{
			if(caractere == 0x08)
	        {
		        // DEL
	            // En consid�rant que la position n'est pas � la fin de la chaine de caractere
	            // D�calage de "position-1" � la "fin de la chaine" d'une case
	            for(i=position-1; i<nb_car_valide; i++)
	                pCommandeCourante[i] = pCommandeCourante[i+1];

	            position -= 1;
	            nb_car_valide -= 1;

                // Rafraichissement de l'affichage
                printf("\r                                                  ");
                printf("\r%s%s", PROMPT, pCommandeCourante);
	        }
	        else if(caractere == 0x7F)
	       {
	            // SUPPR
	            // En consid�rant que la position n'est pas � la fin de la chaine de caractere
	            // D�calage de "position" � la "fin de la chaine" d'une case
	            for(i=position; i<nb_car_valide; i++)
	                pCommandeCourante[i] = pCommandeCourante[i+1];

	            nb_car_valide -= 1;

	            // Rafraichissement de l'affichage
                printf("\r                                                  ");
                printf("\r%s%s", PROMPT, pCommandeCourante);
	        }
	        else
	        {
	            // Pas de caractere special
	            // En consid�rant que la position n'est pas � la fin de la chaine de caractere
	            // D�calage de "position" � la "fin de la chaine" d'une case
	            for(i=nb_car_valide; i>=position; i--)
	                pCommandeCourante[i+1] = pCommandeCourante[i];

	            pCommandeCourante[position++] = caractere;
	            if(caractere==32)
	            	printf("x");
	            nb_car_valide++;
	        }
		}
        // Over writing pour afficher la chaine mise � jour
        //printf("nb_car_valide = %d, position = %d\n", nb_car_valide, position);

    } while (caractere != '\n');

    if(nb_car_valide >= 2)
    {
        pCommandeCourante[nb_car_valide-2] = 0x0;
    }
    else
    {
        pCommandeCourante[0] = 0x0;
    }

	sh_historique.commande_courante = (sh_historique.commande_courante + 1)%SHELL_MAX_HISTORIQUE;

    return pCommandeCourante;
}

typedef  void (*pFunction)(void);
pFunction SysMemBootJump;

void bootLoaderInit()
{

printf("Entering Boot Loader..\r\n");

*((unsigned long *)0x2000FFF0) = 0xDEADBEEF; // 64KB STM32F103
NVIC_SystemReset();

}

void shell()
{
    char commande[50];
    char *tab_args[50];
    int num_args, i;
    float radio_throttle, radio_dir;
    float distance, speed, speed_aimant;
    int32_t lidar_distance_gauche, lidar_distance_droite, lidar_distance_avant, lidar_distance_haut, lidar_rssi, lidar_temperature;
    int quitter;
	int nb_lectures;
	st_tele_element *pTeleElement;
	int erreur;

    sh_historique.commande_courante = 0;
    for(i=0; i<SHELL_MAX_HISTORIQUE; i++)
        sh_historique.commandes[i][0] = '\0';

    printf("\r\n");
    quitter = 0;

    do {
        printf("%s", PROMPT);

        // Saisie de la commande
        saisie_commande();
        if(sh_historique.commande_courante == 0)
        	strcpy(commande, sh_historique.commandes[SHELL_MAX_HISTORIQUE-1]);
        else
        	strcpy(commande, sh_historique.commandes[sh_historique.commande_courante-1]);

        printf("\r\n");

        printf("%s\r\n", commande);

        // Interpretation de la commande
        num_args = 0;
        tab_args[num_args] = strtok(commande, " ");
        while(tab_args[num_args] != NULL)
        {
            tab_args[++num_args] = strtok(NULL, " ");
        }

        // Shell
        if(strcmp(tab_args[0], "hw_led") == 0)
        {
            // Led
            if((num_args == 3) && (strcmp(tab_args[1], "on")==0) && (strcmp(tab_args[2], "led1")==0))
                    HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
            else if((num_args == 3) && (strcmp(tab_args[1], "on")==0) && (strcmp(tab_args[2], "led2")==0))
                    HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
            else if((num_args == 3) && (strcmp(tab_args[1], "on")==0) && (strcmp(tab_args[2], "led3")==0))
                    HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_RESET);
            else if((num_args == 3) && (strcmp(tab_args[1], "off")==0) && (strcmp(tab_args[2], "led1")==0))
                    HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
            else if((num_args == 3) && (strcmp(tab_args[1], "off")==0) && (strcmp(tab_args[2], "led2")==0))
                    HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
            else if((num_args == 3) && (strcmp(tab_args[1], "off")==0) && (strcmp(tab_args[2], "led3")==0))
                    HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET);
            else
                printf("Parametres incorrects. \r\nSyntaxe : hw_led <on/off> <led1/led2/led3>\r\n");
        } else
		if(strcmp(tab_args[0], "hw_buttons") == 0)
		{
			// Afiche l'�tat des boutons
			if(num_args==1)
			{
				if(HAL_GPIO_ReadPin(bouton1_GPIO_Port, bouton1_Pin) ==  GPIO_PIN_SET)
					printf("Bouton1 opened.\r\n");
				else
					printf("Bouton1 closed.\r\n");
				if(HAL_GPIO_ReadPin(bouton2_GPIO_Port, bouton2_Pin) ==  GPIO_PIN_SET)
					printf("Bouton2 opened.\r\n");
				else
					printf("Bouton2 closed.\r\n");
				if(HAL_GPIO_ReadPin(bouton3_GPIO_Port, bouton3_Pin) ==  GPIO_PIN_SET)
					printf("Bouton3 opened.\r\n");
				else
					printf("Bouton3 closed.\r\n");
				if(HAL_GPIO_ReadPin(boutonExt1_GPIO_Port, boutonExt1_Pin) ==  GPIO_PIN_SET)
					printf("BoutonExt1 opened.\r\n");
				else
					printf("BoutonExt1 closed.\r\n");
				printf("\r\n");
			}
			else
				printf("Parametres incorrects. \r\nSyntaxe : hw_buttons\r\n");
		} else
//        if(strcmp(tab_args[0], "params_restore") == 0)
//        {
//            // Eeprom
//            if(num_args==1)
//            {
//            	paramsRestore();
//                printf("\r\n");
//            }
//            else
//                printf("Parametres incorrects. \r\nSyntaxe : params_read\r\n");
//        } else
//        if(strcmp(tab_args[0], "params_show") == 0)
//        {
//            // Parametres de configuration
//            if(num_args==1)
//            {
//                paramsShow();
//                printf("\r\n");
//            }
//            else
//                printf("Parametres incorrects. \r\nSyntaxe : params_show\r\n");
//        } else
//        if(strcmp(tab_args[0], "params_modify") == 0)
//        {
//            // Parametres de configuration
//            if(num_args==1)
//            {
//            	paramsModify();
//            }
//            else
//                printf("Parametres incorrects. \r\nSyntaxe : params_modify\r\n");
//        } else
		if(strcmp(tab_args[0], "radio_get") == 0)
		{
			if(num_args==1)
			{
				radio_dir_get(&radio_dir);
				radio_throttle_get(&radio_throttle);
				// Affiche les commandes reçues de la radio
				printf("Direction <-45 .. 45>, Vitesse <-10..10>\r\n");
				radio_dir = 0.0;
				radio_throttle = 0.0;
				printf("%f° , %fm/s\r\n", radio_dir, radio_throttle);
			}
			else if(num_args==2)
			{
				int nb_lectures = atoi(tab_args[1]);
				// Affiche les commandes reçues de la radio
				printf("Direction <-45 .. 45>, Vitesse <-10..10>\r\n");
				radio_dir = 0.0;
				radio_throttle = 0.0;
				for(i=0;i<nb_lectures;i++)
				{
					radio_dir_get(&radio_dir);
					radio_throttle_get(&radio_throttle);
					printf("%f° , %fm/s\r\n", radio_dir, radio_throttle);

					// Rafraichissement de la radio toutes les 16 ms env.
					HAL_Delay(16);
				}
			}
			else
				printf("Parametres incorrects. \r\nSyntaxe : radio_get <nb_lectures=1>\r\n");
		} else
		if(strcmp(tab_args[0], "vehicule_throttle_set") == 0)
		{
			if(num_args==2)
			{
				float valeur = atof(tab_args[1]);

				if((valeur >= -10.0) && (valeur <= 10.0))
					vehicule_throttle_set(valeur);
			}
			else
				printf("Parametres incorrects. \r\nSyntaxe : vehicule_throttle_set <-10.0 ... 10.0>\r\n");
		} else
		if(strcmp(tab_args[0], "vehicule_dir_set") == 0)
		{
			if(num_args==2)
			{
				float valeur = atof(tab_args[1]);
				printf("%f\r\n", valeur);

				if((valeur >= -26.0) && (valeur <= 26.0))
					vehicule_dir_set(valeur);
			}
			else
				printf("Parametres incorrects. \r\nSyntaxe : vehicule_dir_set <-26.0 ... 26.0>\r\n");
		} else
		if(strcmp(tab_args[0], "vehicule_speed_get") == 0)
		{
			if(num_args==1)
			{

				printf("Iteration; vitesse_cmd; distance; speed; speed_aimant\r\n");
				nb_lectures = 0;
				vehicule_distance_aimant_reset();
				do {
					nb_lectures++;

					vehicule_distance_aimant_get(&distance);
					vehicule_speed_get(&speed);
					vehicule_speed_aimant_get(&speed_aimant);

					// Pilotage par la télécommande
					radio_dir_get(&radio_dir);
					radio_throttle_get(&radio_throttle);
					vehicule_dir_set(radio_dir);
					vehicule_throttle_set(radio_throttle);

					printf("%d; %f; %f; %f; %f\r\n", nb_lectures, radio_throttle, distance, speed, speed_aimant);

					HAL_Delay(10);
					// Acquisition sur 10 s
				} while(nb_lectures<1000);

				// Remise à 0 de la vitesse
				vehicule_throttle_set(0.0);

			}
			else
				printf("Parametres incorrects. \r\nSyntaxe : vehicule_speed_get \r\n");
		} else
		if(strcmp(tab_args[0], "lidar_get") == 0)
		{
			if(num_args==1)
			{
				nb_lectures = 0;

				do {
					tfminiplus_getLastAcquisition(MINILIDAR_GAUCHE, &lidar_distance_gauche, &lidar_rssi, &lidar_temperature);
					tfminiplus_getLastAcquisition(MINILIDAR_DROIT, &lidar_distance_droite, &lidar_rssi, &lidar_temperature);
					tfminiplus_getLastAcquisition(MINILIDAR_AVANT, &lidar_distance_avant, &lidar_rssi, &lidar_temperature);
					tfminiplus_getLastAcquisition(MINILIDAR_HAUT, &lidar_distance_haut, &lidar_rssi, &lidar_temperature);

					printf("Gauche : %ld cm, Avant : %ld cm, Haut : %ld cm, Droit : %ld cm\r\n",
							lidar_distance_gauche, lidar_distance_avant, lidar_distance_haut, lidar_distance_droite);

					HAL_Delay(1000);
					nb_lectures++;
				}while(nb_lectures<120);

			}
			else
				printf("Parametres incorrects. \r\nSyntaxe : lidar_get \r\n");
		} else
//		if(strcmp(tab_args[0], "gyro_registers_get") == 0)
//		{
//			if(num_args==1)
//			{
//				gyroRegisters();
//			}
//			else
//				printf("Parametres incorrects. \r\nSyntaxe : gyro_registers_get\r\n");
//		} else
//		if(strcmp(tab_args[0], "gyro_variance") == 0)
//		{
//			if(num_args==1)
//			{
//				float variance;
//
//				gyroVariance (&variance);
//
//				printf("Variance : ");
//				affiche_float(variance);
//				printf(".\r\n");
//			}
//			else
//				printf("Parametres incorrects. \r\nSyntaxe : gyro_variance\r\n");
//		} else
		if(strcmp(tab_args[0], "paramconf_restaure") == 0)
		{
			if(num_args==1)
			{
				paramConf_restaure();
			} else
				printf("Parametres incorrects. \r\nSyntaxe : paramconf_restaure\r\n");
		} else
		if(strcmp(tab_args[0], "paramconf_sauvegarde") == 0)
		{
			if(num_args==1)
			{
				paramConf_sauvegarde();
			} else
				printf("Parametres incorrects. \r\nSyntaxe : paramconf_sauvegarde\r\n");
		} else
		if(strcmp(tab_args[0], "paramconf_lecture") == 0)
		{
			if(num_args==1)
			{
				paramConf_lecture();
			} else
				printf("Parametres incorrects. \r\nSyntaxe : paramconf_lecture\r\n");
		} else
		if(strcmp(tab_args[0], "paramconf_modification") == 0)
		{
			if(num_args==12)
			{
				paramConf_modification(
						atof(tab_args[1]), atof(tab_args[2]), atof(tab_args[3]),
						atof(tab_args[4]), atof(tab_args[5]), atof(tab_args[6]),
						atof(tab_args[7]), atof(tab_args[8]), atof(tab_args[9]),
						atof(tab_args[10]), atof(tab_args[11]));
			} else
				printf("Parametres incorrects. \r\nSyntaxe : paramconf_modification <N parametres>\r\n");
		} else
		if(strcmp(tab_args[0], "telemetrie") == 0)
		{
			if(num_args==1)
			{
				pTeleElement = telemetrie_pt_lecture_en_cours();
				do {
					printf("T;%f;%f;%f;%f;%f;%f;%d;%d;%d;%d;%d;%d\r\n",
							pTeleElement->consigne_vitesse, pTeleElement->consigne_direction,
							pTeleElement->mesure_vitesse, pTeleElement->heading,
							pTeleElement->gyro_dps, pTeleElement->mesure_distance,
							(int)pTeleElement->lidar_droit, (int)pTeleElement->lidar_gauche,
							(int)pTeleElement->lidar_avant, (int)pTeleElement->lidar_haut,
							pTeleElement->etat_automate_principal, pTeleElement->etat_automate_auto);

					pTeleElement = telemetrie_pt_lecture_suivant(&erreur);
				} while(erreur == 0);
			} else
				printf("Parametres incorrects. \r\nSyntaxe : telemetrie\r\n");
		} else
		if(strcmp(tab_args[0], "gyro_heading_get") == 0)
		{
			if(num_args==1)
			{
				int nb_lectures;
				printf("Iteration; heading; dps\r\n");
				nb_lectures = 0;
				do {
					nb_lectures++;

					printf("%d; %f; %f\r\n", nb_lectures, gyro_get_heading(), gyro_get_dps());

					HAL_Delay(10);
					// Acquisition sur 10 s
				} while(nb_lectures<1000);
			}
			else
				printf("Parametres incorrects. \r\nSyntaxe : gyro_heading_get \r\n");
		} else
        if(strcmp(tab_args[0], "reset") == 0)
        {
            // Reset
            NVIC_SystemReset();
        } else
		if(strcmp(tab_args[0], "quit") == 0)
		{
			// Quitter le shell
			quitter = 1;
		} else
        if(strcmp(tab_args[0], "help") == 0)
        {
            // Help
            printf("Liste des commandes :\r\n");
            printf("- hw_led <on/off> <led0/led1/led2/led3>\r\n");
            printf("         permet d'allumer ou d'eteindre l'une des 4 leds.\r\n");
            printf("- hw_buttons\r\n");
            printf("         permet d'afficher l'etat des boutons.\r\n");
//			  printf("- params_restore\r\n");
//            printf("         permet de charger les parametres depuis la Flash Interne.\r\n");
//            printf("- params_show\r\n");
//            printf("         permet d'afficher les parametres courants\r\n");
//            printf("- params_modify\r\n");
//            printf("         permet de modifier les parametres courants\r\n");
            printf("- radio_get <nb_lectures=1>\r\n");
            printf("         permet de lire les commandes provenant de la radio.\r\n");
            printf("- vehicule_throttle_set <-10.0 ... 10.0>\r\n");
            printf("         permet de commander la vitesse du vehicule.\r\n");
            printf("- vehicule_dir_set <-26.0 ... 26.0>\r\n");
            printf("         permet de commander la direction du vehicule.\r\n");
            printf("- vehicule_speed_get\r\n");
            printf("         permet de lire les capteurs de vitesses pour la calibration du capteur BEAST.\r\n");
            printf("- gyro_heading\r\n");
            printf("         permet de lire le cap en boucle.\r\n");
            printf("- reset\r\n");
            printf("         permet de reseter le robot.\r\n");
            printf("- quit\r\n");
            printf("         permet de sortir du shell.\r\n");
            printf("- version\r\n");
            printf("         permet d'obtenir la version logicielle.\r\n");
        } else
        {
            printf("Commande inconnue !\r\n");
        }

    } while(quitter == 0);
}
