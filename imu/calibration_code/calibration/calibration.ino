/**
 * @file calibration.ino
 * @brief Programme de vérification du statut de calibration du capteur CMPS12 via I2C.
 *
 * Ce programme interroge le registre de statut de calibration du capteur CMPS12
 * et affiche l'état actuel via le port série.
 */

#include <Wire.h>

/// @brief Adresse I2C du capteur CMPS12
#define CMPS12_I2C_ADDR 0x60

/// @brief Adresse du registre indiquant le statut de calibration
#define CALIBRATION_STATUS_REG 0x1E

/**
 * @brief Fonction d'initialisation Arduino.
 *
 * Initialise la communication série et le bus I2C.
 */
void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial.println("Lecture du statut de calibration...");
}

/**
 * @brief Fonction principale exécutée en boucle.
 *
 * Lit le registre de statut de calibration du capteur CMPS12
 * et affiche une description lisible du statut.
 */
void loop() {
    Wire.beginTransmission(CMPS12_I2C_ADDR);
    Wire.write(CALIBRATION_STATUS_REG);
    Wire.endTransmission();

    Wire.requestFrom(CMPS12_I2C_ADDR, 1);
    if (Wire.available()) {
        uint8_t status = Wire.read();
        Serial.print("Statut de calibration : 0x");
        Serial.println(status, HEX);

        if (status == 0xFF) {
            Serial.println("✅ Calibration réussie !");
        } else if (status == 0x00) {
            Serial.println("⚠️ Calibration incomplète, veuillez recalibrer.");
        } else {
            Serial.println("⚠️ Calibration partielle.");
        }
    }

    delay(1000);
}