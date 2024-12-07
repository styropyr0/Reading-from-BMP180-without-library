# BMP180 Sensor Data Reading Project

Hey there, tech nerds and DIY wizards! This project is your golden ticket to dive deep into the guts of the BMP180 sensor. We're going old-school and bypassing the dedicated library, reading data straight from the registers like true hackers. Ready to unlock the mysteries of temperature and pressure readings? Let's get cracking!

## Overview

This project lets you read and understand data from the BMP180 sensor without using the BMP180 library. We're talking raw, unfiltered data straight from the sensor. Perfect for those who want to get their hands dirty and learn the ins and outs of I2C communication.

## Features

- **Direct Register Reading**: Skip the library and get your data straight from the source.
- **Temperature Calculation**: Turn raw data into actual temperature readings.
- **Pressure Calculation**: Convert raw pressure data and even calculate altitude.
- **Educational**: Ideal for learning how to communicate with sensors and process data.

## Components

- **BMP180 Sensor**: The star of the show, a digital barometric pressure sensor.
- **Arduino Board**: Any board that supports I2C will do.
- **Wiring**: Connect the sensor to the Arduino using the I2C interface.

## Setup and Usage

1. **Wiring**:
   - Connect the SDA and SCL pins of the BMP180 to the corresponding pins on the Arduino.
   - VCC to 3.3V and GND to GND.

2. **Code Breakdown**:

### 1. Include Libraries and Define Variables
```cpp
#include <Wire.h>

uint8_t DAT[22] = { /* Calibration data addresses */ };
short _AC1, _AC2, _AC3, _B1, _B2, _MB, _MC, _MD;
short _AC4, _AC5, _AC6;
long B5, UT, UP, T;
```
- **Wire.h**: Library for I2C communication.
- **DAT array**: Holds calibration data addresses.
- **Calibration Variables**: Stores necessary calibration values.
- **Measurement Variables**: Temporary storage for raw and calculated data.

### 2. Setup Function
```cpp
void setup() {
    Wire.begin();
    Serial.begin(9600);
    delay(100);

    for (int i = 0; i < 22; i++)
        DAT[i] = readData(DAT[i]);

    shiftVal(DAT[0], DAT[1], &_AC1, 0);
    // Repeat for other calibration values...
}
```
- **Wire.begin()**: Starts I2C communication.
- **Serial.begin(9600)**: Sets up serial communication.
- **Calibration Data Reading**: Reads and stores calibration values from the sensor.

### 3. Read Data Function
```cpp
uint16_t readData(int REG_ADDR) {
    Wire.beginTransmission(0x77);
    Wire.write(REG_ADDR);
    Wire.endTransmission(false);
    Wire.requestFrom(0x77, 1);
    return Wire.read();
}
```
- **readData()**: Reads a byte of data from the sensor's register.

### 4. Shift Value Function
```cpp
void shiftVal(uint8_t V1, uint8_t V2, short *store, int isAbs) {
    *store = static_cast<int16_t>((int16_t(V1) << 8) | V2);
    if (isAbs)
        *store = (uint16_t(V1) << 8) | V2;
}
```
- **shiftVal()**: Combines two 8-bit values into a 16-bit value.

### 5. Read Uncompensated Temperature
```cpp
void readUT() {
    Wire.beginTransmission(0x77);
    Wire.write(0xF4);
    Wire.write(0x2E);
    Wire.endTransmission(false);
    delayMicroseconds(4500);

    Wire.beginTransmission(0x77);
    Wire.write(0xF6);
    Wire.endTransmission(false);
    Wire.requestFrom(0x77, 2);
    UT = static_cast<uint16_t>((uint16_t(Wire.read()) << 8) | Wire.read());
}
```
- **readUT()**: Grabs the raw temperature data from the sensor.

### 6. Calculate Temperature
```cpp
void temp() {
    long x1 = (UT - _AC6) * (_AC5 / pow(2, 15));
    long x2 = _MC * pow(2, 11) / (x1 + _MD);
    B5 = x1 + x2;
    T = (B5 + 8) / pow(2, 4);
    Serial.print("Temperature: ");
    Serial.print(float(T) / 10.4);
    Serial.println("C");
}
```
- **temp()**: Converts raw temperature data into actual temperature readings.

### 7. Read Uncompensated Pressure
```cpp
void readUP(short oss) {
    Wire.beginTransmission(0x77);
    Wire.write(0xF4);
    Wire.write(0x34 | (oss << 6));
    Wire.endTransmission(false);
    delay(70);

    UP = 0;
    Wire.beginTransmission(0x77);
    Wire.write(0xF6);
    Wire.endTransmission(false);
    Wire.requestFrom(0x77, 3);
    for (int i = 0; i < 3; i++) {
        UP <<= 8;
        UP |= Wire.read();
    }
    UP >>= (8 - oss);
}
```
- **readUP()**: Grabs the raw pressure data from the sensor.

### 8. Calculate Pressure and Altitude
```cpp
void pressure(short oss) {
    long B6 = B5 - 4000;
    long X1 = (_B2 * (B6 * B6 / pow(2, 12))) / pow(2, 11);
    long X2 = _AC2 * B6 / pow(2, 11);
    long X3 = X1 + X2;
    long B3 = ((((_AC1 * 4) + X3) << oss) + 2) / 4;
    X1 = _AC3 * B6 / pow(2, 13);
    X2 = (_B1 * (B6 * B6 / pow(2, 12))) / pow(2, 16);
    X3 = ((X1 + X2) + 2) / pow(2, 2);
    unsigned long B4 = _AC4 * (unsigned long)(X3 + 32768) / pow(2, 15);
    unsigned long B7 = ((unsigned long)UP - B3) * (50000 >> oss);
    long p = 0;
    if (B7 < 0x80000000)
        p = (B7 * 2) / B4;
    else
        p = (B7 / B4) * 2;
    X1 = pow((p / pow(2, 8)), 2);
    X1 = (X1 * 3038) / pow(2, 16);
    X2 = (-7357 * p) / pow(2, 16);
    p = p + (X1 + X2 + 3791) / pow(2, 4);
    Serial.print("Pressure: ");
    Serial.print(p);
    Serial.print("Pa");
    Serial.print("\tAltitude: ");
    double alt1 = 44330.0 * (1 - pow(((double)p / 101325.0), 1.0 / 5.255));
    Serial.print(alt1);
    Serial.println("m");
}
```
- **pressure()**: Turns raw pressure data into actual pressure readings and calculates altitude.

### 9. Main Loop
```cpp
void loop() {
    readUT();
    readUP(3);
    temp();
    pressure(3);
    delay(1000);
}
```
- **loop()**: Continuously reads temperature and pressure data, calculates the actual values, and prints them every second.

## Author

- **Saurav Sajeev**
- Email: sauravsajeev202@gmail.com

## License

This project is open-source and available under the MIT License.
