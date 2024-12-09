#include <VectorNav.h>

// Initialiser VectorNav-objektet med Serial1
VectorNav imu(Serial1);

void setup() {
    // Start Serial til debugging
    Serial.begin(115200);

    // Start Serial1 til IMU
    Serial1.begin(115200);

    // Test forbindelsen til IMU'en
    if (imu.testConnection()) {
        Serial.println("IMU connected successfully.");
    } else {
        Serial.println("Failed to connect to IMU.");
    }
}

void loop() {
    // Opdater IMU-data
    if (imu.update()) {
        // Udskriv IMU-værdier til Serial
        Serial.print("Pitch: ");
        Serial.print(imu.getPitch());
        Serial.print(", Roll: ");
        Serial.print(imu.getRoll());
        Serial.print(", Yaw: ");
        Serial.print(imu.getYaw());
        Serial.print(", AngX: ");
        Serial.print(imu.getAngx());
        Serial.print(", AngY: ");
        Serial.print(imu.getAngy());
        Serial.print(", AngZ: ");
        Serial.print(imu.getAngz());
        Serial.print(", AccX: ");
        Serial.print(imu.getAccx());
        Serial.print(", AccY: ");
        Serial.print(imu.getAccy());
        Serial.print(", AccZ: ");
        Serial.println(imu.getAccz());
    } else {
        // Hvis ingen data modtages
        Serial.println("No new IMU data.");
    }

    // Vent lidt før næste opdatering
    delay(100);
}
