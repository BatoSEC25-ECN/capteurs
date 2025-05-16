#include <Wire.h>

#define CMPS12_I2C_ADDR 0x60  // Adresse I2C du CMPS12
#define CALIBRATION_STATUS_REG 0x1E  // Registre du statut de calibration

void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial.println("Lecture du statut de calibration...");
}

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
            Serial.println("⚠️ Calibration partielle. Faites tourner le capteur dans toutes les directions.");
        }
    } else {
        Serial.println("❌ Impossible de lire le statut de calibration.");
    }

    delay(5000);
}