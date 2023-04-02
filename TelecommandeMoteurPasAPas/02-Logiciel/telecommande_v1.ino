/*
 * 1 manivelle avec encodeur en quadrature, et 1 stepper moteur.
 */

#include <Encoder.h>

// === Définition Encodeur
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knob(21, 20);
//   avoid using pins with LEDs attached
// Pin 2 et Pin 3 interruptible

// === Définition SubD9 vers controlleur Moteur
int PUL=2; // define Pulse pin
int DIR=3; // define Direction pin
int ENA=4; // define Enable pin

// === Définition des boutons
// Les IO ont une pull up interne. Les signaux sont donc actifs niveau bas
int SW_GA=8;
int SW_DR=18;

// Si ni MODE_Patrick, ni MODE_Christophe alors c'est le MODE_Libre
int MODE_Patrick=14;
int MODE_Christophe=15;

// Si ni VIT_Basse, ni VIT_Haute alors c'est VIT_Moyenne
int VIT_Basse=9;
int VIT_Haute=10;

void setup() {
  // Initialisation de la liason série
  Serial.begin(9600);
  Serial.println("1 manivelle pilote 1 moteur :");

  // Initialisation des pull up
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);

  pinMode (SW_GA, INPUT_PULLUP);
  pinMode (SW_DR, INPUT_PULLUP);

  pinMode (MODE_Patrick, INPUT_PULLUP);
  pinMode (MODE_Christophe, INPUT_PULLUP);

  pinMode (VIT_Basse, INPUT_PULLUP);
  pinMode (VIT_Haute, INPUT_PULLUP);

  // Initialisation de l'encodeur
  knob.write(0);
}

long cmd_manivelle  = 0;
int  prev_mode = 2;

void loop() {
  long newCmd_manivelle;
  long deplacement_moteur;

  // Mesure de la commande de vitesse
  int cmd_vit = digitalRead(VIT_Basse) + 2 * digitalRead(VIT_Haute);
  Serial.print("Vit : ");
  Serial.print(cmd_vit);
  Serial.print(". ");
  int vitesse = 0;

  if( cmd_vit == 3 )
  {
    // Vitesse moyenne
    vitesse = 3; // Période d'un créneau de 6 ms
  } else if( cmd_vit == 2 )
  {
    // Vitesse Haute
    vitesse = 1; // Période d'un créneau de 2 ms
  } else 
  {
    // Par défaut Vitesse Lente
    vitesse = 6; // Période d'un créneau de 12 ms
  } 

  // Dans quel mode sommes nous ?
  int mode = digitalRead(MODE_Patrick) + 2 * digitalRead(MODE_Christophe);
  Serial.print("Mode : ");
  Serial.print(mode);
  Serial.print(". ");
  if( mode == 2 )
  {
    // Nous sommes en Mode Patrick

    // Le moteur est piloté par le rotary encodeur
    newCmd_manivelle = knob.read();
    // Si precedemment nous n'étions pas en mode Patrick, réinitialisation de l'encodeur. 
    // Un petit malin a pu jouer avec l'encodeur pendant que nous étions en Mode Christophe, et le bras risque de partir dans tous les sens. 
    if( prev_mode != mode )    
      cmd_manivelle = newCmd_manivelle;
    
    if (newCmd_manivelle != cmd_manivelle) {
      // Debug    
      Serial.print("Commande de la manivelle = ");
      Serial.print(newCmd_manivelle);
      Serial.print(". ");
  
      // Le moteur doit se déplacer de newCmd_manivelle - cmd_manivelle
      deplacement_moteur = newCmd_manivelle - cmd_manivelle;
  
      // Si deplacement_moteur est positif, on fait tourner le moteur dans un sens
      // Si deplacement_moteur est négatif, on fait tourner le moteur dans l'autre sens
      if(deplacement_moteur >= 0)
      {
        digitalWrite(DIR,LOW);
      }        
      else    
      {
        digitalWrite(DIR,HIGH);
      }        
  
      // Activation du moteur
      digitalWrite(ENA,HIGH);
      // Debug    
      Serial.print("abs(deplacement_moteur) = ");
      Serial.print(abs(deplacement_moteur));
      Serial.println();
      // Avance du moteur
      for(int i=0; i< 10*abs(deplacement_moteur); i++)
      {
        digitalWrite(PUL,HIGH);
        delay(vitesse);
        digitalWrite(PUL,LOW);
        delay(vitesse);
      }    
      // Désactivation du moteur
      digitalWrite(ENA,LOW);
  
      // Sauvegarde de la commande courante
      cmd_manivelle = newCmd_manivelle;
    } else
    {
      // Désactivation du moteur
      digitalWrite(ENA,LOW);
    }
  } else if ( mode == 1 ) 
  {
    // Nous sommes en Mode Christophe
    // Le moteur est piloté par les boutons Gauche et Droite
    // Dans quel sens allons nous ?
    int sens = digitalRead(SW_GA) + 2 * digitalRead(SW_DR);
    Serial.print("Bouton : ");
    Serial.print(sens);
    Serial.print(". ");

    if( sens == 1 )
    {
      // Sens
      digitalWrite(DIR,LOW);
      // Activation du moteur
      digitalWrite(ENA,HIGH);
      // Avance du moteur
      for(int i=0; i< 10; i++)
      {
        digitalWrite(PUL,HIGH);
        delay(vitesse);
        digitalWrite(PUL,LOW);
        delay(vitesse);
      }    
      // Désactivation du moteur
      digitalWrite(ENA,LOW);
      
    } else if( sens == 2 )
    {
      // Sens
      digitalWrite(DIR,HIGH);
      // Activation du moteur
      digitalWrite(ENA,HIGH);
      // Avance du moteur
      for(int i=0; i< 10; i++)
      {
        digitalWrite(PUL,HIGH);
        delay(vitesse);
        digitalWrite(PUL,LOW);
        delay(vitesse);
      }    
      // Désactivation du moteur
      digitalWrite(ENA,LOW);
    } else
    {
      // Sinon, le moteur ne bouge pas
      // Désactivation du moteur
      digitalWrite(ENA,LOW);
    }
    
  } else if ( mode == 3 )
  {
    // Nous sommes en mode libre. Le moteur tourne à la main
    // Activation du moteur
    //digitalWrite(ENA,HIGH);

  } else
  {
    // La valeur de mode est bizarre, on stoppe le moteur
    Serial.println("Mode incoonu");
  }
  Serial.println(".");

  // On mémorise le mode courant pour la prochaine fois
  prev_mode = mode;

  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knob.write(0);
  }
}
