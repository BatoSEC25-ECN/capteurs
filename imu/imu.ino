/**
 * @file imu.ino
 * @brief Lecture du cap (heading) et du roulis (roll) via le capteur CMPS12 en I2C.
 *
 * Ce programme interroge un capteur d’orientation CMPS12 connecté en I2C pour obtenir :
 * - Le cap du bateau (direction vers laquelle il pointe, en degrés par rapport au nord magnétique).
 * - Le roulis (roll), c’est-à-dire l’inclinaison latérale du bateau.
 *
 * Ces informations sont affichées sur le port série pour un usage en navigation embarquée.
 */



// Arduino DUE and CMPS12 compass
// Copyright (C) 2021 https://www.roboticboat.uk
// 65928a79-e9a1-403b-a375-b1fe84b15aa4
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// These Terms shall be governed and construed in accordance with the laws of 
// England and Wales, without regard to its conflict of law provisions.


#include <Wire.h>


// Register Function
// 0        Command register (write) / Software version (read)

// 1        Compass Bearing as a byte, i.e. 0-255 for a full circle
// 2,3      Compass Bearing as a word, i.e. 0-3599 for a full circle, representing 0-359.9 degrees. Register 2 being the high byte

// 4        Pitch angle - signed byte giving angle in degrees from the horizontal plane, Kalman filtered with Gyro
// 5        Roll angle - signed byte giving angle in degrees from the horizontal plane, Kalman filtered with Gyro

// 6,7      Magnetometer X axis raw output, 16 bit signed integer with register 6 being the upper 8 bits
// 8,9      Magnetometer Y axis raw output, 16 bit signed integer with register 8 being the upper 8 bits
// 10,11    Magnetometer Z axis raw output, 16 bit signed integer with register 10 being the upper 8 bits

// 12,13    Accelerometer  X axis raw output, 16 bit signed integer with register 12 being the upper 8 bits
// 14,15    Accelerometer  Y axis raw output, 16 bit signed integer with register 14 being the upper 8 bits
// 16,17    Accelerometer  Z axis raw output, 16 bit signed integer with register 16 being the upper 8 bits

// 18,19    Gyro X axis raw output, 16 bit signed integer with register 18 being the upper 8 bits
// 20,21    Gyro Y axis raw output, 16 bit signed integer with register 20 being the upper 8 bits
// 22,23    Gyro Z axis raw output, 16 bit signed integer with register 22 being the upper 8 bits

//---------------------------------

  //Address of the CMPS12 compass on i2C
  /// @brief Adresse I2C du capteur CMPS12
  #define _i2cAddress 0x60

  /// @brief registre du statut de calibration
  #define CALIBRATION_STATUS_REG 0x1E  // Registre du statut de calibration

  /// @brief Registre de contrôle du capteur (lecture/commande)
  #define CONTROL_Register 0

/// @brief Registre de poids fort de l’angle de cap (2 octets)
  #define BEARING_Register 2 

  /// @brief Registre contenant le tangage (pitch), en degrés signés (-90 à +90)
  #define PITCH_Register 4 

  /// @brief Registre contenant le roulis (roll), en degrés signés (-90 à +90)
  #define ROLL_Register 5


/// @brief Registre de l’axe X du magnétomètre (2 octets)
  #define MAGNET_X_Register  6

/// @brief Registre de l’axe Y du magnétomètre (2 octets)
  #define MAGNET_Y_Register  8

/// @brief Registre de l’axe Z du magnétomètre (2 octets)
  #define MAGNET_Z_Register 10

/// @brief Registre de l’axe X de l’accéléromètre (2 octets)
  #define ACCELERO_X_Register 12
  /// @brief Registre de l’axe Y de l’accéléromètre (2 octets)
  #define ACCELERO_Y_Register 14
  /// @brief Registre de l’axe XZde l’accéléromètre (2 octets)
  #define ACCELERO_Z_Register 16

/// @brief Registre de l’axe X du gyroscope (2 octets)
  #define _Register_GYRO_X 18
  /// @brief Registre de l’axe Y du gyroscope (2 octets)
  #define _Register_GYRO_Y 20
  /// @brief Registre de l’axe Z du gyroscope (2 octets)
  #define _Register_GYRO_Z 22

/// @brief Constante représentant la lecture d’un seul octet
  #define ONE_BYTE 1
/// @brief Constante représentant la lecture de deux octets
  #define TWO_BYTES 2

//---------------------------------

  /// @brief Cap (bearing) brut lu depuis le capteur, en dixièmes de degré (0–3599)
int _bearing;

/// @brief Nombre d’octets reçus lors de la lecture I2C
int nReceived;

/// @brief Valeur fine de l’angle (précision supplémentaire du cap)
byte _fine;

/// @brief Octet de poids fort d’une valeur sur 16 bits
byte _byteHigh;

/// @brief Octet de poids faible d’une valeur sur 16 bits
byte _byteLow;

/// @brief Valeur du tangage (pitch), exprimée en degrés signés (-90 à +90)
char _pitch;

/// @brief Valeur du roulis (roll), exprimée en degrés signés (-90 à +90)
char _roll;

/// @brief Accélération sur l’axe X du bateau, en m/s²
float g_acceleration_x_bateau = 0;

/// @brief Accélération sur l’axe Y du bateau, en m/s²
float g_acceleration_y_bateau = 0;

/// @brief Accélération sur l’axe Z du bateau, en m/s²
float g_acceleration_z_bateau = 0;

/// @brief Échelle de conversion des données brutes en accélération : 1 m/s² = 100 LSB
float _accelScale = 1.0f / 100.f;

/// @brief État de la calibration du capteur : true si elle est réussie
bool calibrationOk = false;

//---------------------------------

/**
 * @brief Initialise les interfaces de communication série et I2C.
 *
 * Cette fonction configure :
 * - Le port série à 9600 bauds pour l'affichage dans le moniteur série.
 * - La communication I2C avec le capteur CMPS12.
 *
 * @return void
 */

void setup() {

  // Initialize the serial port to the User
  // Set this up early in the code, so the User sees all messages
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize i2c network
  Wire.begin();

  calibrationOk = false;

}



/**
 * @brief Boucle principale du programme.
 *
 * Appelle les fonctions de lecture du cap et du roll, puis affiche les valeurs
 * obtenues sur le moniteur série. Répète l’opération toutes les 500 ms.
 *
 * @return void
 */

void loop() {


  if(isCalibrationOk())
  {
    digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(500);                      // wait for a second
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(500);
  }
  
  // read the compass
  
  int g_cap_actuel_bateau = getBearing();

  signed char g_gite_bateau = getPitch();

  signed char roll = getRoll();

  // Read the accelerator
  g_acceleration_x_bateau = getAcceleroX() * _accelScale;
  g_acceleration_y_bateau = getAcceleroY() * _accelScale;
  g_acceleration_z_bateau = getAcceleroZ() * _accelScale;

  // Print data to Serial Monitor window
  
  Serial.print("$CMP, relèvement :");
  Serial.println(g_cap_actuel_bateau);
  /*Serial.print(" , tangage: ");
  Serial.print(g_gite_bateau); 
  Serial.print(" , roulis :");
  Serial.print(roll);
  Serial.print(" degree,");
  
  Serial.print("\t$ACC, x :");
  Serial.print(g_acceleration_x_bateau,4);
  Serial.print(" , y :");
  Serial.print(g_acceleration_y_bateau,4); 
  Serial.print(" , z :");
  Serial.print(g_acceleration_z_bateau,4);
  Serial.print(" m/s^2,");*/

  delay(100);
}

/**
 * @brief Vérifie si la calibration du capteur CMPS12 est terminée avec succès.
 *
 * Cette fonction interroge le registre de statut de calibration du CMPS12 via I2C.
 * Elle considère la calibration comme réussie si le registre retourne la valeur `0xFF`.
 *
 * @note Cette vérification est cruciale avant d’utiliser les données d’orientation du capteur.
 *
 * @return true si la calibration est complète (`0xFF`), false sinon (erreur ou incomplète).
 */

bool isCalibrationOk() {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(CALIBRATION_STATUS_REG);
    Wire.endTransmission();

    Wire.requestFrom(_i2cAddress, 1);
    if (Wire.available()) {
        uint8_t status = Wire.read();
        return (status == 0xFF); // Calibration complète uniquement
    }
    return false; // Erreur de lecture
}

/**
 * @brief Lit le cap (heading) du capteur CMPS12.
 *
 * Cette fonction lit les deux octets représentant l'angle d’orientation
 * du capteur CMPS12. L’angle est renvoyé en dixièmes de degrés.
 *
 * @note Si la lecture échoue (capteur non disponible), retourne -1.
 *
 * @return int Cap en dixièmes de degrés (ex: 1234 = 123.4°), ou -1 en cas d’échec.
 */
int16_t getBearing()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(BEARING_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate full bearing
  _bearing = ((_byteHigh<<8) + _byteLow) / 10;
  
  return _bearing;
}


/**
 * @brief Lit l’angle de tangage (pitch) depuis le capteur CMPS12.
 *
 * Cette fonction communique avec le registre de tangage du CMPS12 via I2C.
 * Elle lit un octet signé représentant l’inclinaison avant/arrière du bateau
 * sur l’axe longitudinal (pitch), exprimée en degrés dans l’intervalle [-90, +90].
 *
 * En cas d’échec de communication ou d'erreur de lecture, la fonction retourne 0.
 *
 * @return byte Valeur du tangage (pitch) en degrés signés, ou 0 en cas d’échec.
 */
byte getPitch()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(PITCH_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}

  // Request 1 byte from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , ONE_BYTE);

  // Something has gone wrong
  if (nReceived != ONE_BYTE) return 0;

  // Read the values
  _pitch = Wire.read();

  return _pitch;
}


/**
 * @brief Lit l’angle de roulis (roll) du capteur CMPS12.
 *
 * Cette fonction lit un octet signé représentant l’angle d’inclinaison latérale.
 * Elle permet d’évaluer le basculement du bateau sur l’axe longitudinal.
 *
 * @note Le capteur retourne une valeur entière entre -90° et +90°.
 * @note En cas d’échec de communication, retourne 0 par défaut.
 *
 * @return int8_t Roulis en degrés (entre -90 et +90)
 */

byte getRoll()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ROLL_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 1 byte from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , ONE_BYTE);

  // Something has gone wrong
  if (nReceived != ONE_BYTE) return 0;

  // Read the values
  _roll = Wire.read();

  return _roll ;
}

/**
 * @brief Lit la valeur brute de l'accélération sur l'axe X depuis le capteur CMPS12.
 *
 * Cette fonction interroge le registre de l'accéléromètre X du CMPS12 via I2C,
 * lit deux octets (MSB + LSB) et les combine pour obtenir une valeur signée 16 bits.
 *
 * La valeur retournée est brute (en LSB) et peut être convertie en m/s²
 * en utilisant l’échelle `_accelScale = 1.0f / 100.f`.
 *
 * @note En cas de problème de communication ou de réception incomplète, retourne 0.
 *
 * @return int16_t Valeur d'accélération brute sur l’axe X, ou 0 si erreur.
 */
int16_t getAcceleroX()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELERO_X_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}


/**
 * @brief Lit la valeur brute de l'accélération sur l'axe Y depuis le capteur CMPS12.
 *
 * Cette fonction interroge le registre de l'accéléromètre Y du CMPS12 via I2C,
 * lit deux octets (MSB + LSB) et les combine pour obtenir une valeur signée 16 bits.
 *
 * La valeur retournée est brute (en LSB) et peut être convertie en m/s²
 * en utilisant l’échelle `_accelScale = 1.0f / 100.f`.
 *
 * @note En cas de problème de communication ou de réception incomplète, retourne 0.
 *
 * @return int16_t Valeur d'accélération brute sur l’axe Y, ou 0 si erreur.
 */
int16_t getAcceleroY()
{ 
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELERO_Y_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

/**
 * @brief Lit la valeur brute de l'accélération sur l'axe Z depuis le capteur CMPS12.
 *
 * Cette fonction interroge le registre de l'accéléromètre Z du CMPS12 via I2C,
 * lit deux octets (MSB + LSB) et les combine pour obtenir une valeur signée 16 bits.
 *
 * La valeur retournée est brute (en LSB) et peut être convertie en m/s²
 * en utilisant l’échelle `_accelScale = 1.0f / 100.f`.
 *
 * @note En cas de problème de communication ou de réception incomplète, retourne 0.
 *
 * @return int16_t Valeur d'accélération brute sur l’axe Z, ou 0 si erreur.
 */
int16_t getAcceleroZ()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELERO_Z_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);

}

/**
 * @brief Lit la valeur brute du champ magnétique sur l’axe X depuis le capteur CMPS12.
 *
 * Cette fonction interroge le registre du magnétomètre X via I2C.
 * Elle lit deux octets (MSB + LSB), les combine pour produire une valeur
 * entière signée 16 bits représentant l’intensité du champ magnétique.
 *
 * @note La valeur retournée est brute (en LSB) et peut être convertie en µT
 * si l’échelle est connue (non fournie par défaut par CMPS12).
 *
 * @note En cas d’échec de communication ou de réception incomplète, la fonction retourne 0.
 *
 * @return int16_t Valeur brute du champ magnétique sur l’axe X, ou 0 si erreur.
 */

int16_t getMagnetX()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNET_X_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

/**
 * @brief Lit la valeur brute du champ magnétique sur l’axe Y depuis le capteur CMPS12.
 *
 * Cette fonction interroge le registre du magnétomètre Y via I2C.
 * Elle lit deux octets (MSB + LSB), les combine pour produire une valeur
 * entière signée 16 bits représentant l’intensité du champ magnétique.
 *
 * @note La valeur retournée est brute (en LSB) et peut être convertie en µT
 * si l’échelle est connue (non fournie par défaut par CMPS12).
 *
 * @note En cas d’échec de communication ou de réception incomplète, la fonction retourne 0.
 *
 * @return int16_t Valeur brute du champ magnétique sur l’axe Y, ou 0 si erreur.
 */

int16_t getMagnetY()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNET_Y_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

/**
 * @brief Lit la valeur brute du champ magnétique sur l’axe Z depuis le capteur CMPS12.
 *
 * Cette fonction interroge le registre du magnétomètre Z via I2C.
 * Elle lit deux octets (MSB + LSB), les combine pour produire une valeur
 * entière signée 16 bits représentant l’intensité du champ magnétique.
 *
 * @note La valeur retournée est brute (en LSB) et peut être convertie en µT
 * si l’échelle est connue (non fournie par défaut par CMPS12).
 *
 * @note En cas d’échec de communication ou de réception incomplète, la fonction retourne 0.
 *
 * @return int16_t Valeur brute du champ magnétique sur l’axe Z, ou 0 si erreur.
 */

int16_t getMagnetZ()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNET_Z_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

/**
 * @brief Change l’adresse I2C du capteur CMPS12.
 *
 * Cette fonction envoie une séquence spécifique au capteur CMPS12
 * via le registre de contrôle pour modifier son adresse I2C.
 * 
 * ⚠️ **Important** :
 * - Seul le capteur dont l'adresse doit être changée doit être présent sur le bus I2C.
 * - La nouvelle adresse doit être une valeur paire (7 bits, se terminant par 0).
 * - Le changement est **persistant** (sauvegardé en EEPROM sur le capteur).
 *
 * @warning Cette opération est irréversible sans outils spécifiques si la nouvelle
 * adresse est incorrecte. Utiliser avec précaution.
 *
 * @param i2cAddress Adresse I2C actuelle du capteur (ex: `0x60`).
 * @param newi2cAddress Nouvelle adresse I2C souhaitée (ex: `0x64`).
 * @return void
 */

void changeAddress(byte i2cAddress, byte newi2cAddress)
{
  // Reset the address on the i2c network
  // Ensure that you have only this module connected on the i2c network
  // The 7 bit i2c address must end with a 0. (even numbers please)
  // For example changeAddress(0x60, 0x64)

  // Address 0x60, 1 long flash, 0 short flashes
  // Address 0x62, 1 long flash, 1 short flashes
  // Address 0x64, 1 long flash, 2 short flashes
  // Address 0x66, 1 long flash, 3 short flashes
  // Address 0x68, 1 long flash, 4 short flashes
  // Address 0x6A, 1 long flash, 5 short flashes
  // Address 0x6C, 1 long flash, 6 short flashes
  // Address 0x6E, 1 long flash, 7 short flashes

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xA0));
  
  // End the transmission
  int nackCatcher = Wire.endTransmission();

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xAA));

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return;}

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xA5));

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return;}

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(newi2cAddress);

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return;}

}

/**
 * @brief Vérifie l’état de calibration du capteur CMPS12 et affiche un message explicatif.
 *
 * Cette fonction lit le registre de statut de calibration du CMPS12 via I2C
 * et affiche une description du résultat sur le port série :
 * - `0xFF` : Calibration complète 
 * - `0x00` : Calibration absente 
 * - Autre : Calibration partielle 
 *
 * Elle attend 10 secondes (`delay(10000)`) après l'affichage pour laisser
 * le temps à l’utilisateur de lire le message.
 *
 * @note Cette fonction est utile pour l’étape de diagnostic ou de test du capteur.
 *
 * @return true si la calibration est complète (`0xFF`), false sinon.
 */

bool checkCalibrationStatus() {

    bool check_calibration_status = false;
    Wire.beginTransmission(_i2cAddress);
    Wire.write(CALIBRATION_STATUS_REG);
    Wire.endTransmission();

    Wire.requestFrom(_i2cAddress, 1);
    if (Wire.available()) {
        uint8_t status = Wire.read();
        Serial.print("Statut de calibration : 0x");
        Serial.println(status, HEX);

        if (status == 0xFF) {
            Serial.println("Calibration réussie !");
            check_calibration_status = true;
        } else if (status == 0x00) {
            Serial.println("Calibration incomplète, veuillez recalibrer.");
        } else {
            Serial.println("Calibration partielle. Faites tourner le capteur dans toutes les directions.");
        }
    } else {
        Serial.println("Impossible de lire le statut de calibration.");
    }

    delay(10000);

    return check_calibration_status;
}