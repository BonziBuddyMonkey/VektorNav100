#ifndef VECTORNAV_H
#define VECTORNAV_H

#include <Arduino.h>

// Klasse til at håndtere VectorNav IMU-data
class VectorNav {
public:
    // Constructor der tager en reference til den Serial-port, der skal bruges
    VectorNav(HardwareSerial &serialPort) : imuSerial(serialPort) {}

    // Opdaterer pitch, roll og andre værdier ved at læse fra IMU'en
    bool update() {
        imu_sync_detected = false; // Nulstil synkroniseringsstatus

        // Tjek om der er nye data tilgængelige fra IMU'en
        if (imuSerial.available() > 4) {
            checkSyncByte(); // Søger efter synkroniseringsbyte
        }

        // Hvis synkronisering er opdaget, læs resten af dataene
        if (imu_sync_detected) {
            readImuData(); // Læs IMU-data og opdater værdier
            return true; // Returnér true, hvis data blev opdateret
        }
        return false; // Returnér falsk, hvis ingen gyldige data blev modtaget
    }

    // Getter-funktioner til IMU-værdier
    float getPitch() const { return pitch.f; } // Returnér pitch-værdi
    float getRoll() const { return roll.f; } // Returnér roll-værdi
    float getYaw() const { return yaw.f; } // Returnér yaw-værdi
    float getAngx() const { return W_x.f; } // Returnér vinkelhastighed i x
    float getAngy() const { return W_y.f; } // Returnér vinkelhastighed i y
    float getAngz() const { return W_z.f; } // Returnér vinkelhastighed i z
    float getAccx() const { return a_x.f; } // Returnér acceleration i x
    float getAccy() const { return a_y.f; } // Returnér acceleration i y
    float getAccz() const { return a_z.f; } // Returnér acceleration i z

    // Tester forbindelsen til IMU'en
    bool testConnection() {
        imuSerial.write(0xFA); // Send en test-byte
        delay(10); // Vent på respons
        if (imuSerial.available() > 0) { // Hvis respons er tilgængelig
            byte response;
            imuSerial.readBytes(&response, 1); // Læs respons
            return response == 0xFA; // Forventet respons fra IMU
        }
        return false; // Ingen respons, forbindelsen fejlede
    }

private:
    HardwareSerial &imuSerial; // Serial-port til kommunikation med IMU

    // Unioner til at konvertere mellem bytes og float-værdier
    union { float f; byte b[4]; } yaw, pitch, roll; // Orientering
    union { float f; byte b[4]; } W_x, W_y, W_z; // Vinkelhastigheder
    union { float f; byte b[4]; } a_x, a_y, a_z; // Accelerationer
    union { unsigned short s; byte b[2]; } checksum; // Checksum til validering

    // Indikator for om synkroniseringsbyte er fundet
    bool imu_sync_detected = false;
    byte in[100]; // Buffer til indkommende data

    // Tjekker for synkroniseringsbyte (0xFA)
    void checkSyncByte() {
        for (int i = 0; i < 6; i++) { // Maksimalt 6 forsøg på at finde sync-byte
            imuSerial.readBytes(in, 1); // Læs én byte fra IMU
            if (in[0] == 0xFA) { // Hvis sync-byte er fundet
                imu_sync_detected = true; // Marker synkronisering som fundet
                break;
            }
        }
    }

    // Læser data fra IMU'en og opdaterer værdier
    void readImuData() {
        imuSerial.readBytes(in, 41); // Læs 41 bytes (dataformat fra IMU)

        // Ekstraher checksum fra de sidste to bytes
        checksum.b[0] = in[40]; // Første byte af checksum
        checksum.b[1] = in[39]; // Anden byte af checksum

        // Valider data med checksum
        if (calculateImuCrc(in, 39) == checksum.s) { // Tjek om checksum stemmer
            // Hvis valid, konverter bytes til float-værdier
            for (int i = 0; i < 4; i++) { // Læs 4 bytes pr. værdi
                yaw.b[i] = in[3 + i]; // Yaw-data
                pitch.b[i] = in[7 + i]; // Pitch-data
                roll.b[i] = in[11 + i]; // Roll-data
                W_x.b[i] = in[15 + i]; // Vinkelhastighed i x
                W_y.b[i] = in[19 + i]; // Vinkelhastighed i y
                W_z.b[i] = in[23 + i]; // Vinkelhastighed i z
                a_x.b[i] = in[27 + i]; // Acceleration i x
                a_y.b[i] = in[31 + i]; // Acceleration i y
                a_z.b[i] = in[35 + i]; // Acceleration i z
            }
        }
    }

    // Beregner 16-bit CRC til at validere IMU-data
    unsigned short calculateImuCrc(byte data[], unsigned int length) {
        unsigned short crc = 0; // Initialiser CRC-værdi
        for (unsigned int i = 0; i < length; i++) { // Iterer over data
            crc = (byte)(crc >> 8) | (crc << 8); // Skift bits
            crc ^= data[i]; // XOR med data
            crc ^= (byte)(crc & 0xff) >> 4; // Juster crc med højre skift
            crc ^= crc << 12; // Juster crc med venstre skift
            crc ^= (crc & 0x00ff) << 5; // Juster crc med endnu et venstre skift
        }
        return crc; // Returnér beregnet CRC
    }
};

#endif
