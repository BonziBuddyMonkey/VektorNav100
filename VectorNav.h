#ifndef VECTORNAV_H
#define VECTORNAV_H

#include <Arduino.h>

class VectorNav {
public:
    // Constructor that takes a reference to the Serial port to use
    VectorNav(HardwareSerial &serialPort) : imuSerial(serialPort) {}

    // Update the pitch and roll values by reading from the IMU
    bool update() {
        imu_sync_detected = false;

        // Check if new IMU data is available
        if (imuSerial.available() > 4) {
            checkSyncByte();
        }

        // If sync byte is detected, read the rest of the data
        if (imu_sync_detected) {
            readImuData();
            return true;
        }
        return false;
    }

    // Get pitch value
    float getPitch() const {
        return pitch.f;
    }

    // Get roll value
    float getRoll() const {
        return roll.f;
    }

    // Test if connection to IMU is successful
    bool testConnection() {
        imuSerial.write(0xFA);  // Send a test byte
        delay(10);
        if (imuSerial.available() > 0) {
            byte response;
            imuSerial.readBytes(&response, 1);
            return response == 0xFA;  // Assuming the IMU echoes the byte back
        }
        return false;
    }

private:
    HardwareSerial &imuSerial;

    // Union functions for byte to float conversions
    union { float f; byte b[4]; } pitch;
    union { float f; byte b[4]; } roll;
    union { unsigned short s; byte b[2]; } checksum;

    // Parameters
    bool imu_sync_detected = false;
    byte in[100];

    // Check for the sync byte (0xFA)
    void checkSyncByte() {
        for (int i = 0; i < 6; i++) {
            imuSerial.readBytes(in, 1);
            if (in[0] == 0xFA) {
                imu_sync_detected = true;
                break;
            }
        }
    }

    // Read the IMU bytes
    void readImuData() {
        imuSerial.readBytes(in, 41);

        checksum.b[0] = in[40];
        checksum.b[1] = in[39];

        if (calculateImuCrc(in, 39) == checksum.s) {
            for (int i = 0; i < 4; i++) {
                pitch.b[i] = in[7 + i];
                roll.b[i] = in[11 + i];
            }
        }
    }

    // Calculate the 16-bit CRC for the given ASCII or binary message.
    unsigned short calculateImuCrc(byte data[], unsigned int length) {
        unsigned int i;
        unsigned short crc = 0;
        for (i = 0; i < length; i++) {
            crc = (byte)(crc >> 8) | (crc << 8);
            crc ^= data[i];
            crc ^= (byte)(crc & 0xff) >> 4;
            crc ^= crc << 12;
            crc ^= (crc & 0x00ff) << 5;
        }
        return crc;
    }
};

#endif
