/**
 * @file gps-rtk.ino
 * @brief Code de lecture de position GPS haute précision avec le module u-blox ZED-F9P.
 * 
 * Ce programme lit les données GNSS (latitude, longitude, altitude, précision) via I2C depuis un module
 * compatible u-blox ZED-F9P, avec communication série vers un module XBee pour le traitement RTCM.
 * Il traite les messages RTCM reçus et vérifie si tous les types nécessaires sont reçus pour une solution RTK fixe.
 *
 * @details
 * - Utilise la bibliothèque SparkFun u-blox Arduino GNSS
 * - Interagit avec des trames RTCM pour la correction différentielle GNSS
 * - Affiche les données sur le moniteur série à 115200 bauds
 * - Vérifie et affiche le type de solution GNSS (Fix, RTK Float, RTK Fixed)
 *
 */

/*
  Get the high precision geodetic solution for latitude and longitude
  By: Nathan Seidle
  Modified by: Steven Rowland and Paul Clark
  SparkFun Electronics
  Date: April 17th, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to inspect the accuracy of the high-precision
  positional solution. Please see below for information about the units.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

/// @brief Timer local pour limiter la fréquence d’interrogation I2C
#include <Wire.h> //Needed for I2C to GPS

///< @brief Bibliothèque SparkFun pour modules GNSS u-blox V1.8.11
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
///< @brief Instance de l’objet GNSS u-blox
SFE_UBLOX_GPS myGPS;

/// @brief Timer local pour limiter la fréquence d’interrogation I2C
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

// Ajustez au besoin (port série pour XBee, débit en bauds)
/// @brief Interface série utilisée pour le module XBee
#define XBEE_SERIAL   Serial2
/// @brief Vitesse de transmission série pour le XBee (en bauds)
#define XBEE_BAUD     115200
/// @brief Intervalle d’affichage des données (en millisecondes)
#define PRINT_PERIOD  1000   // En millisecondes

// Définir l'état du parser RTCM

/**
 * @enum ParserState
 * @brief États du parseur RTCM pour détecter les trames complètes
 */
enum ParserState {
  WAIT_FOR_D3,     ///< Attente du préambule (0xD3)
  READ_LEN_HI,     ///< Lecture du MSB de la longueur
  READ_LEN_LO,     ///< Lecture du LSB de la longueur
  READ_PAYLOAD     ///< Lecture du reste de la trame
};

// Définir la taille maximale d’un message RTCM
/// @brief Taille maximale d’une trame RTCM
#define RTCM_MAX_MSG 1024
/// @brief Longueur maximale d’un message RTCM (sécurité)
#define RTCM_MAX_LEN 1024

/////////////nouveux params////////////////

/// @brief État courant du parseur RTCM
static ParserState rtcmState = WAIT_FOR_D3;

/// @brief Buffer de stockage temporaire d’une trame RTCM
static uint8_t rtcmBuffer[RTCM_MAX_MSG];

/// @brief Nombre d’octets restants à lire pour compléter la trame
static uint16_t bytesToRead = 0;

/// @brief Index courant dans le buffer RTCM
static uint16_t idx = 0;


// Suivi des trames RTCM reçues
/// @brief Tableau de suivi des types RTCM reçus (indexés par type)
bool rtcmSeen[1300] = {false};

/// @brief Horodatage de la dernière réinitialisation du suivi RTCM
unsigned long lastRTCMReset = 0;

// Types critiques requis
/// @brief Liste des types RTCM requis pour une correction GNSS complète
const uint16_t requiredRTCM[] = {1006, 1074, 1087};

/// @brief Indique quels types requis ont été reçus
bool requiredReceived[sizeof(requiredRTCM)/sizeof(uint16_t)] = {false};

/// @brief Indique si tous les types requis ont été reçus
bool allRequiredReceived = false;

/// @brief Indique si le type RTCM 1005 a été vu
bool seen1005 = false;

/// @brief Indique si le type RTCM 1074 a été vu
bool seen1074 = false;

/// @brief Indique si le type RTCM 1084 a été vu
bool seen1084 = false;

/// @brief Indique si le type RTCM 1094 a été vu
bool seen1094 = false;

/// @brief Indique si le type RTCM 1230 a été vu
bool seen1230 = false;

/////////////nouveux params////////////////
/**
 * @brief Initialise les interfaces de communication et configure le module GNSS u-blox.
 *
 * Cette fonction est exécutée une seule fois au démarrage du microcontrôleur.
 * Elle initialise :
 * - La communication série pour debug
 * - Le bus I2C pour le capteur GNSS
 * - Le port série utilisé pour le module XBee
 * 
 * Elle établit la connexion avec le module GNSS u-blox via I2C,
 * configure le type de messages (UBX uniquement) et règle la fréquence de mise à jour à 20 Hz.
 *
 * En cas d’échec de détection du module, la boucle reste bloquée avec un message d’erreur.
 *
 * @return void
 */
void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal

  Wire.begin();
  // while (!myGPS.begin(Wire)); 

  XBEE_SERIAL.begin(XBEE_BAUD, SERIAL_8N1);  

  // myGPS.enableDebugging(Serial);

  while (myGPS.begin(Wire) == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.setNavigationFrequency(20); //Set output to 20 times a second

  byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
  Serial.print("Current update rate: ");
  Serial.println(rate);

  //myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}




void loop()
{
  // 1) Lire UN seul octet du XBee, s'il est dispo :
  if (XBEE_SERIAL.available() > 0)
  {
    uint8_t b = XBEE_SERIAL.read();
    // Serial.print("gestion xbee");
    // Serial.print(b);
    // // Envoyer l'octet au module GNSS :
    // // (selon votre version de la lib : pushRawData(buffer, taille))
    myGPS.pushRawData(&b, 1);
    // parseRTCMFrame();
  }

  // 2) Faire un "update" du GNSS 
  // (Dans les nouvelles versions : myGPS.checkUblox(Wire), checkCallbacks())
  // Mais dans votre version, probablement :
  myGPS.checkUblox();
 

  // 3) Périodiquement (par ex. toutes les 500 ms) afficher des infos
  unsigned long now = millis();
  if (millis() - lastTime >= 1000)
  {
    lastTime = millis();

// First, let's collect the position data
    int32_t latitude = myGPS.getHighResLatitude();
    int8_t latitudeHp = myGPS.getHighResLatitudeHp();
    int32_t longitude = myGPS.getHighResLongitude();
    int8_t longitudeHp = myGPS.getHighResLongitudeHp();
    int32_t ellipsoid = myGPS.getElipsoid();
    int8_t ellipsoidHp = myGPS.getElipsoidHp();
    int32_t msl = myGPS.getMeanSeaLevel();
    int8_t mslHp = myGPS.getMeanSeaLevelHp();
    uint32_t accuracy = myGPS.getHorizontalAccuracy();

    // Defines storage for the lat and lon units integer and fractional parts
    int32_t integer_latitude_bateau; // Integer part of the latitude in degrees
    int32_t lat_frac; // Fractional part of the latitude
    int32_t integer_longitude_bateau; // Integer part of the longitude in degrees
    int32_t lon_frac; // Fractional part of the longitude
    double g_latitude_bateau; // concatenated latitude
    double g_longitude_bateau; // concatenated longitude

    // Calculate the latitude and longitude integer and fractional parts
    integer_latitude_bateau = latitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
    int32_t un_lat_frac = latitude - (integer_latitude_bateau  * 10000000); // Calculate the fractional part of the latitude
    lat_frac = (un_lat_frac * 100) + latitudeHp; // Now add the high resolution component
    if (lat_frac < 0) // If the fractional part is negative, remove the minus sign
    {
      lat_frac = 0 - lat_frac;
    }
    if (integer_latitude_bateau < 0){
      lat_frac = -lat_frac;
    }
    g_latitude_bateau = (double)integer_latitude_bateau + ((double)lat_frac / 1e9);

    integer_longitude_bateau = longitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
    lon_frac = longitude - (integer_longitude_bateau * 10000000); // Calculate the fractional part of the longitude
    lon_frac = (lon_frac * 100) + longitudeHp; // Now add the high resolution component
    if (lon_frac < 0) // If the fractional part is negative, remove the minus sign
    {
      lon_frac = 0 - lon_frac;
    }
    if (integer_longitude_bateau < 0){
      lon_frac = -lon_frac;
    }
    g_longitude_bateau = (double)integer_longitude_bateau + ((double)lon_frac / 1e9);
    
    // Print the lat and lon
    Serial.print("Lat (deg): ");
    Serial.print(integer_latitude_bateau); // Print the integer part of the latitude
    Serial.print(".");
    Serial.print(lat_frac); // Print the fractional part of the latitude
    Serial.print(", test lat (deg): ");
    Serial.print(g_latitude_bateau, 9); // Print the integer part of the latitude
    Serial.print(", Lon (deg): ");
    Serial.print(integer_longitude_bateau); // Print the integer part of the latitude
    Serial.print(".");
    Serial.println(abs(lon_frac)); // Print the fractional part of the latitude
    Serial.print(", test long (deg): ");
    Serial.print(g_longitude_bateau, 9); // Print the integer part of the latitude

    // Now define float storage for the heights and accuracy
    float f_ellipsoid;
    float f_msl;
    float f_accuracy;

    // Calculate the height above ellipsoid in mm * 10^-1
    f_ellipsoid = (ellipsoid * 10) + ellipsoidHp;
    // Now convert to m
    f_ellipsoid = f_ellipsoid / 10000.0; // Convert from mm * 10^-1 to m

    // Calculate the height above mean sea level in mm * 10^-1
    f_msl = (msl * 10) + mslHp;
    // Now convert to m
    int32_t g_altitude_bateau = f_msl / 10000.0; // Convert from mm * 10^-1 to m

    // Convert the horizontal accuracy (mm * 10^-1) to a float
    f_accuracy = accuracy;
    // Now convert to m
    f_accuracy = f_accuracy / 10000.0; // Convert from mm * 10^-1 to m

    // Finally, do the printing
    Serial.print("Ellipsoid (m): ");
    Serial.print(f_ellipsoid, 4); // Print the ellipsoid with 4 decimal places

    Serial.print(", Mean Sea Level(m): ");
    Serial.print(g_altitude_bateau, 4); // Print the mean sea level with 4 decimal places

    Serial.print(", Accuracy (m): ");
    Serial.println(f_accuracy, 4); // Print the accuracy with 4 decimal places

        uint8_t fixType = myGPS.getFixType();
    uint8_t carrierSolution = myGPS.getCarrierSolutionType();

    Serial.print("Fix Type: ");
    switch (fixType)
    {
      case 0: Serial.print("No Fix"); break;
      case 1: Serial.print("Dead Reckoning"); break;
      case 2: Serial.print("2D Fix"); break;
      case 3: Serial.print("3D Fix"); break;
      case 4: Serial.print("GNSS + Dead Reckoning"); break;
      case 5: Serial.print("Time Only"); break;
      default: Serial.print("Unknown"); break;
    }

    Serial.print(", RTK Status: ");
    switch (carrierSolution)
    {
      case 0: Serial.println("No RTK"); break;
      case 1: Serial.println("RTK Float"); break;
      case 2: Serial.println("RTK Fixed"); break;
      default: Serial.println("Unknown"); break;
    }
  }

  // => Fin de loop(), on y retourne immédiatement.  
  // Si d’autres octets XBee sont arrivés, on les lira au prochain tour,
  // tout en relançant checkUblox() à chaque fois.
}


// void parseRTCMFrame()
// {
//   static ParserState rtcmState = WAIT_FOR_D3;
//   static uint8_t rtcmBuffer[RTCM_MAX_MSG];
//   static uint16_t bytesToRead = 0;
//   static uint16_t idx = 0;

//   while (XBEE_SERIAL.available()) {
//     uint8_t b = XBEE_SERIAL.read();

//     switch(rtcmState) {
//       case WAIT_FOR_D3:
//         if (b == 0xD3) {
//           Serial.println("début trame");
//           idx = 0;
//           rtcmBuffer[idx++] = b;
//           rtcmState = READ_LEN_HI;
//         }
//         break;

//       case READ_LEN_HI:
//         Serial.println("trame en cours 1");
//         rtcmBuffer[idx++] = b;
//         bytesToRead = (b & 0x3F) << 8;
//         rtcmState = READ_LEN_LO;
//         break;

//       case READ_LEN_LO:
//         Serial.println("trame en cours 2");
//         rtcmBuffer[idx++] = b;
//         bytesToRead |= b;
//         if (bytesToRead > RTCM_MAX_LEN) {
//           rtcmState = WAIT_FOR_D3;
//         } else {
//           bytesToRead += 3; // CRC
//           rtcmState = READ_PAYLOAD;
//         }
//         break;

//       case READ_PAYLOAD:
//         rtcmBuffer[idx++] = b;
//         Serial.println("fin trame");
//         bytesToRead--;
//         if (bytesToRead == 0) {
//           Serial.print("RTCM Frame: ");
//           for (uint16_t i = 0; i < idx; i++) {
//             if (rtcmBuffer[i] < 0x10) Serial.print("0"); // pour avoir toujours 2 chiffres
//             Serial.print(rtcmBuffer[i], HEX);
//             Serial.print(" ");
//           }
//           printRTCMType(rtcmBuffer, idx);
//           Serial.println();
//           // ✅ Une trame complète est reçue, on l'envoie puis on stoppe
//           myGPS.pushRawData(rtcmBuffer, idx);
//           // Réinitialisation pour la prochaine fois
//           rtcmState = WAIT_FOR_D3;
//           idx = 0;
//           bytesToRead = 0;
//           return; // ✅ On quitte la fonction après 1 seule trame
//         }
//         break;
//     }
//   }
// }

/**
 * @brief Extrait et affiche le type d’un message RTCM à partir d’un buffer.
 *
 * Cette fonction extrait le type du message RTCM contenu dans le buffer binaire,
 * l'affiche sur le port série, et met à jour les tableaux de suivi `rtcmSeen` et `requiredReceived`.
 * Si tous les types requis sont reçus, elle affiche une confirmation.
 *
 * @param buf Pointeur vers le buffer contenant le message RTCM.
 * @param len Longueur du buffer.
 * @return void
 */

void printRTCMType(const uint8_t* buf, size_t len) {
  if (len < 6) return;
  uint16_t type = ((buf[3] & 0xFC) << 4) | ((buf[4] & 0xF0) >> 4);
  Serial.print("RTCM Type: ");
  Serial.println(type);

  if (type < 1300) rtcmSeen[type] = true;

  for (size_t i = 0; i < sizeof(requiredRTCM)/sizeof(uint16_t); i++) {
    if (type == requiredRTCM[i]) {
      requiredReceived[i] = true;
    }
  }

  allRequiredReceived = true;
  for (size_t i = 0; i < sizeof(requiredRTCM)/sizeof(uint16_t); i++) {
    if (!requiredReceived[i]) {
      allRequiredReceived = false;
      break;
    }
  }

  if (allRequiredReceived) {
    Serial.println("✅ Tous les types RTCM requis ont été reçus.");
  }
}

/**
 * @brief Réinitialise le suivi des messages RTCM reçus.
 *
 * Cette fonction remet à zéro :
 * - le tableau `rtcmSeen[]` qui trace les types RTCM observés,
 * - les flags `requiredReceived[]` pour les types requis,
 * - l’indicateur global `allRequiredReceived`.
 *
 * Elle enregistre également un nouvel horodatage de réinitialisation.
 *
 * @return void
 */
void resetRTCMSeen() {
  for (int i = 0; i < 1300; i++) rtcmSeen[i] = false;
  for (size_t i = 0; i < sizeof(requiredRTCM)/sizeof(uint16_t); i++) requiredReceived[i] = false;
  allRequiredReceived = false;
  lastRTCMReset = millis();
}

/**
 * @brief Affiche l’état actuel des messages RTCM observés.
 *
 * Cette fonction affiche :
 * - tous les types RTCM reçus depuis la dernière réinitialisation (`rtcmSeen`),
 * - l’état de réception des types requis (`requiredReceived`),
 * avec des indicateurs ✅ ou ❌.
 *
 * Elle est utile pour le diagnostic de la réception des corrections GNSS.
 *
 * @return void
 */
void printRTCMStatus() {
  Serial.println("[RTCM Types vus récemment]");
  for (int i = 0; i < 1300; i++) {
    if (rtcmSeen[i]) {
      Serial.print(i);
      Serial.print(" ");
    }
  }
  Serial.println();

  Serial.print("Reçus requis: ");
  for (size_t i = 0; i < sizeof(requiredRTCM)/sizeof(uint16_t); i++) {
    Serial.print(requiredRTCM[i]);
    Serial.print(requiredReceived[i] ? ":✅ " : ":❌ ");
  }
  Serial.println();
}

/**
 * @brief Parse les trames RTCM reçues via la liaison série XBee.
 *
 * Cette fonction lit les octets reçus depuis `XBEE_SERIAL`, les assemble dans un buffer RTCM,
 * et détecte automatiquement le début (`0xD3`), la longueur et la fin de chaque trame.
 *
 * Une fois une trame complète détectée :
 * - Elle extrait le type RTCM à partir des octets de l’en-tête.
 * - Elle appelle `updateRTCMTypeReceived()` pour mettre à jour les types requis.
 * - Si la trame est considérée comme utile, elle est transmise au module GNSS via `myGPS.pushRawData()`.
 *
 * Toutes les 5 secondes, la fonction affiche les types RTCM reçus et réinitialise les flags via `printRTCMStatus()` et `resetRTCMSeen()`.
 *
 * @note Utilise un automate à 4 états : `WAIT_FOR_D3`, `READ_LEN_HI`, `READ_LEN_LO`, `READ_PAYLOAD`.
 * @return void
 */

void parseRTCMFrame() {
  while (XBEE_SERIAL.available()) {
    uint8_t b = XBEE_SERIAL.read();

    switch(rtcmState) {
      case WAIT_FOR_D3:
        if (b == 0xD3) {
          idx = 0;
          rtcmBuffer[idx++] = b;
          rtcmState = READ_LEN_HI;
        }
        break;

      case READ_LEN_HI:
        rtcmBuffer[idx++] = b;
        bytesToRead = (b & 0x3F) << 8;
        rtcmState = READ_LEN_LO;
        break;

      case READ_LEN_LO:
        rtcmBuffer[idx++] = b;
        bytesToRead |= b;
        if (bytesToRead > RTCM_MAX_MSG - 6) {
          rtcmState = WAIT_FOR_D3;
        } else {
          bytesToRead += 3; // CRC
          rtcmState = READ_PAYLOAD;
        }
        break;

      // case READ_PAYLOAD:
      //   rtcmBuffer[idx++] = b;
      //   bytesToRead--;
      //   if (bytesToRead == 0) {
      //     printRTCMType(rtcmBuffer, idx);
      //     if (allRequiredReceived) {
      //       myGPS.pushRawData(rtcmBuffer, idx);
      //     }
      //     rtcmState = WAIT_FOR_D3;
      //     idx = 0;
      //   }
      //   break;
      case READ_PAYLOAD:
        rtcmBuffer[idx++] = b;
        bytesToRead--;
        if (bytesToRead == 0) {
          // Trame complète reçue

          // 🔍 Extraire le type RTCM (les bits 6 à 17, soit octet 3 & 4)
          uint16_t rtcmType = ((uint16_t)(rtcmBuffer[3] & 0xFC) << 4) | ((rtcmBuffer[4] & 0xF0) >> 4);

          // 🖨️ Affichage optionnel pour debug
          Serial.print("RTCM Type: ");
          Serial.println(rtcmType);

          // ✅ Mettre à jour la checklist des types nécessaires
          updateRTCMTypeReceived(rtcmType); // ↪️ À définir plus haut

          // 🚦Pousser au GPS si trame pertinente
          if (isUsefulRTCMType(rtcmType)) {
            myGPS.pushRawData(rtcmBuffer, idx);
          }

          // 🔄 Réinitialisation de l'état du parseur
          rtcmState = WAIT_FOR_D3;
          idx = 0;
          bytesToRead = 0;
        }
        break;

    }
  }

  if (millis() - lastRTCMReset > 5000) {
    printRTCMStatus();
    resetRTCMSeen();
  }
}



// Affiche le type de message RTCM reçu (ex: 1005, 1077, 1230, ...)
void printRTCMType(const uint8_t* buffer, uint16_t len) {
  if (len < 6 || buffer[0] != 0xD3) return;

  uint16_t header = ((buffer[3] & 0x03) << 8) | buffer[4];
  Serial.print("RTCM Type: ");
  Serial.println(header);
}

bool isUsefulRTCMType(uint16_t type) {
  // Liste des types utiles à garder
  return (type == 1005 || type == 1074 || type == 1084 || type == 1094 || type == 1230);
}

void updateRTCMTypeReceived(uint16_t type) {
  if (type == 1005) seen1005 = true;
  if (type == 1074) seen1074 = true;
  if (type == 1084) seen1084 = true;
  if (type == 1094) seen1094 = true;
  if (type == 1230) seen1230 = true;

  allRequiredReceived = seen1005 && seen1074 && seen1084 && seen1094 && seen1230;
}


// void loop()
// {
//   //Query module only every second. Doing it more often will just cause I2C traffic.
//   //The module only responds when a new position is available
//   if (millis() - lastTime > 1000)
//   {
//     lastTime = millis(); //Update the timer

//     // getHighResLatitude: returns the latitude from HPPOSLLH as an int32_t in degrees * 10^-7
//     // getHighResLatitudeHp: returns the high resolution component of latitude from HPPOSLLH as an int8_t in degrees * 10^-9
//     // getHighResLongitude: returns the longitude from HPPOSLLH as an int32_t in degrees * 10^-7
//     // getHighResLongitudeHp: returns the high resolution component of longitude from HPPOSLLH as an int8_t in degrees * 10^-9
//     // getElipsoid: returns the height above ellipsoid as an int32_t in mm
//     // getElipsoidHp: returns the high resolution component of the height above ellipsoid as an int8_t in mm * 10^-1
//     // getMeanSeaLevel: returns the height above mean sea level as an int32_t in mm
//     // getMeanSeaLevelHp: returns the high resolution component of the height above mean sea level as an int8_t in mm * 10^-1
//     // getHorizontalAccuracy: returns the horizontal accuracy estimate from HPPOSLLH as an uint32_t in mm * 10^-1

//     // If you want to use the high precision latitude and longitude with the full 9 decimal places
//     // you will need to use a 64-bit double - which is not supported on all platforms

//     // To allow this example to run on standard platforms, we cheat by converting lat and lon to integer and fractional degrees

//     // The high resolution altitudes can be converted into standard 32-bit float

//     // First, let's collect the position data
//     int32_t latitude = myGPS.getHighResLatitude();
//     int8_t latitudeHp = myGPS.getHighResLatitudeHp();
//     int32_t longitude = myGPS.getHighResLongitude();
//     int8_t longitudeHp = myGPS.getHighResLongitudeHp();
//     int32_t ellipsoid = myGPS.getElipsoid();
//     int8_t ellipsoidHp = myGPS.getElipsoidHp();
//     int32_t msl = myGPS.getMeanSeaLevel();
//     int8_t mslHp = myGPS.getMeanSeaLevelHp();
//     uint32_t accuracy = myGPS.getHorizontalAccuracy();

//     // Defines storage for the lat and lon units integer and fractional parts
//     int32_t lat_int; // Integer part of the latitude in degrees
//     int32_t lat_frac; // Fractional part of the latitude
//     int32_t lon_int; // Integer part of the longitude in degrees
//     int32_t lon_frac; // Fractional part of the longitude

//     // Calculate the latitude and longitude integer and fractional parts
//     lat_int = latitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
//     lat_frac = latitude - (lat_int * 10000000); // Calculate the fractional part of the latitude
//     lat_frac = (lat_frac * 100) + latitudeHp; // Now add the high resolution component
//     if (lat_frac < 0) // If the fractional part is negative, remove the minus sign
//     {
//       lat_frac = 0 - lat_frac;
//     }
//     lon_int = longitude / 10000000; // Convert latitude from degrees * 10^-7 to Degrees
//     lon_frac = longitude - (lon_int * 10000000); // Calculate the fractional part of the longitude
//     lon_frac = (lon_frac * 100) + longitudeHp; // Now add the high resolution component
//     if (lon_frac < 0) // If the fractional part is negative, remove the minus sign
//     {
//       lon_frac = 0 - lon_frac;
//     }

//     // Print the lat and lon
//     Serial.print("Lat (deg): ");
//     Serial.print(lat_int); // Print the integer part of the latitude
//     Serial.print(".");
//     Serial.print(lat_frac); // Print the fractional part of the latitude
//     Serial.print(", Lon (deg): ");
//     Serial.print(lon_int); // Print the integer part of the latitude
//     Serial.print(".");
//     Serial.println(lon_frac); // Print the fractional part of the latitude

//     // Now define float storage for the heights and accuracy
//     float f_ellipsoid;
//     float f_msl;
//     float f_accuracy;

//     // Calculate the height above ellipsoid in mm * 10^-1
//     f_ellipsoid = (ellipsoid * 10) + ellipsoidHp;
//     // Now convert to m
//     f_ellipsoid = f_ellipsoid / 10000.0; // Convert from mm * 10^-1 to m

//     // Calculate the height above mean sea level in mm * 10^-1
//     f_msl = (msl * 10) + mslHp;
//     // Now convert to m
//     f_msl = f_msl / 10000.0; // Convert from mm * 10^-1 to m

//     // Convert the horizontal accuracy (mm * 10^-1) to a float
//     f_accuracy = accuracy;
//     // Now convert to m
//     f_accuracy = f_accuracy / 10000.0; // Convert from mm * 10^-1 to m

//     // Finally, do the printing
//     Serial.print("Ellipsoid (m): ");
//     Serial.print(f_ellipsoid, 4); // Print the ellipsoid with 4 decimal places

//     Serial.print(", Mean Sea Level(m): ");
//     Serial.print(f_msl, 4); // Print the mean sea level with 4 decimal places

//     Serial.print(", Accuracy (m): ");
//     Serial.println(f_accuracy, 4); // Print the accuracy with 4 decimal places

//         uint8_t fixType = myGPS.getFixType();
//     uint8_t carrierSolution = myGPS.getCarrierSolutionType();

//     Serial.print("Fix Type: ");
//     switch (fixType)
//     {
//       case 0: Serial.print("No Fix"); break;
//       case 1: Serial.print("Dead Reckoning"); break;
//       case 2: Serial.print("2D Fix"); break;
//       case 3: Serial.print("3D Fix"); break;
//       case 4: Serial.print("GNSS + Dead Reckoning"); break;
//       case 5: Serial.print("Time Only"); break;
//       default: Serial.print("Unknown"); break;
//     }

//     Serial.print(", RTK Status: ");
//     switch (carrierSolution)
//     {
//       case 0: Serial.println("No RTK"); break;
//       case 1: Serial.println("RTK Float"); break;
//       case 2: Serial.println("RTK Fixed"); break;
//       default: Serial.println("Unknown"); break;
//     }

//   }
// }