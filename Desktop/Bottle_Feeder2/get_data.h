#ifndef GET_DATA_H
#define GET_DATA_H

#include <Arduino.h>
#include <HardwareSerial.h>

//-----------------------------------------
// MAX485
//-----------------------------------------
#define RS485_DE 7
#define RS485_RE 8

HardwareSerial PLC(1);

//-----------------------------------------
// PLC Settings
//-----------------------------------------
#define SLAVE_ID 1

//-----------------------------------------
// CRC16
//-----------------------------------------
uint16_t modbusCRC(uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t pos = 0; pos < len; pos++)
    {
        crc ^= buf[pos];

        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }

    return crc;
}

//-----------------------------------------
// UART Init
//-----------------------------------------
void init_modbus()
{
    pinMode(RS485_DE, OUTPUT);
    pinMode(RS485_RE, OUTPUT);

    digitalWrite(RS485_DE, LOW);
    digitalWrite(RS485_RE, LOW);

    PLC.begin(9600);

    delay(500);
}

//-----------------------------------------
// Read Single Coil
//-----------------------------------------
bool readCoil(uint16_t address, bool &state)
{
    uint16_t zeroAddress = address - 1;

    uint8_t frame[8];

    frame[0] = SLAVE_ID;
    frame[1] = 0x01;
    frame[2] = highByte(zeroAddress);
    frame[3] = lowByte(zeroAddress);
    frame[4] = 0x00;
    frame[5] = 0x01;

    uint16_t crc = modbusCRC(frame, 6);

    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    while (PLC.available())
        PLC.read();

    digitalWrite(RS485_DE, HIGH);
    digitalWrite(RS485_RE, HIGH);

    delay(2);

    PLC.write(frame, 8);
    PLC.flush();

    delay(2);

    digitalWrite(RS485_DE, LOW);
    digitalWrite(RS485_RE, LOW);

    uint8_t resp[10];
    int count = 0;

    unsigned long t = millis();

    while (millis() - t < 300)
    {
        while (PLC.available())
        {
            resp[count++] = PLC.read();

            if (count >= 6)
                break;
        }

        if (count >= 6)
            break;
    }

    Serial.print("TX: ");

    for (int i = 0; i < 8; i++)
    {
        if (frame[i] < 16)
            Serial.print("0");

        Serial.print(frame[i], HEX);
        Serial.print(" ");
    }

    Serial.println();

    Serial.print("RX: ");

    for (int i = 0; i < count; i++)
    {
        if (resp[i] < 16)
            Serial.print("0");

        Serial.print(resp[i], HEX);
        Serial.print(" ");
    }

    Serial.println();

    if (count < 6)
        return false;

    if (resp[1] == 0x81)
    {
        Serial.print("PLC Exception Code: ");
        Serial.println(resp[2]);
        return false;
    }

    state = resp[3] & 0x01;

    return true;
}

//-----------------------------------------
// Read Holding Register
//-----------------------------------------
bool readRegister(uint16_t reg, uint16_t &value)
{
    uint16_t zero = reg - 40001;

    uint8_t frame[8];

    frame[0] = SLAVE_ID;
    frame[1] = 0x03;
    frame[2] = highByte(zero);
    frame[3] = lowByte(zero);
    frame[4] = 0x00;
    frame[5] = 0x01;

    uint16_t crc = modbusCRC(frame, 6);

    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    while (PLC.available())
        PLC.read();

    digitalWrite(RS485_DE, HIGH);
    digitalWrite(RS485_RE, HIGH);

    delay(2);

    PLC.write(frame, 8);
    PLC.flush();

    delay(2);

    digitalWrite(RS485_DE, LOW);
    digitalWrite(RS485_RE, LOW);

    uint8_t resp[20];
    int count = 0;

    unsigned long t = millis();

    while (millis() - t < 300)
    {
        while (PLC.available())
        {
            resp[count++] = PLC.read();

            if (count >= 7)
                break;
        }

        if (count >= 7)
            break;
    }

    if (count < 7)
        return false;

    value = ((uint16_t)resp[3] << 8) | resp[4];

    return true;
}

bool readRegister32(uint16_t reg, uint32_t &value)
{
    uint16_t lowWord, highWord;

    if (!readRegister(reg, lowWord))
        return false;

    if (!readRegister(reg + 1, highWord))
        return false;

    // Delta PLC: D0 = Low word, D1 = High word
    value = ((uint32_t)highWord << 16) | lowWord;

    return true;
}

//-----------------------------------------
// Print Helpers
//-----------------------------------------
void printCoil(uint16_t addr, const char *name)
{
    bool state;

    Serial.print(name);
    Serial.print(" (");
    Serial.print(addr);
    Serial.println(")");

    if (readCoil(addr, state))
    {
        Serial.print("Status: ");
        Serial.println(state ? "ON" : "OFF");
    }
    else
    {
        Serial.println("FAILED");
    }

    Serial.println("----------------------------");
    delay(1500);
}

void printRegister(uint16_t reg, const char *name)
{
    uint32_t value;

    Serial.print(name);
    Serial.print(" (");
    Serial.print(reg);
    Serial.println(")");

    if (readRegister32(reg, value))
    {
        Serial.print("Value: ");
        Serial.println(value);
    }
    else
    {
        Serial.println("FAILED");
    }

    Serial.println("----------------------------");
    delay(1500);
}

//-----------------------------------------
// Main Display
//-----------------------------------------
void load_data()
{
    Serial.println();
    Serial.println("========================================");
    Serial.println(" Bottle Feeder PLC Status");
    Serial.println("========================================");

    // Process States
    printCoil(2049, "M0 Start");
    printCoil(2050, "M1 Conveyor1");
    printCoil(2051, "M2 Rotary 90");
    printCoil(2052, "M3 Pump");
    printCoil(2053, "M4 Delay");
    printCoil(2054, "M5 Second Rotation");
    printCoil(2055, "M6 Cap Push");
    printCoil(2056, "M7 Third Rotation");
    printCoil(2057, "M8 Cap Lock");
    printCoil(2068, "M20 Interlock");
    printCoil(2069, "M21 Interlock");
    printCoil(2070, "M22 Interlock");

    // Timer Status
    printCoil(1537, "T0 12s Pump Delay");
    printCoil(1538, "T1 Filling Timer");
    printCoil(1539, "T2 Rotation Delay");
    printCoil(1540, "T3 Cap Push Timer");
    printCoil(1541, "T4 Cap Lock Timer");
    printCoil(1542, "T5 Conveyor Restart");
    printCoil(1543, "T6 Conveyor 2 Timer");

    // Physical Inputs
    // printCoil(11039, "X16 Start Button");
    // printCoil(11026, "X3 Sensor1");
    // printCoil(11027, "X0 Sensor2");
    // printCoil(11026, "X1 Sensor3");
    // printCoil(11027, "X2 Sensor4");
    // printCoil(11034, "X11 Y-Axis Limit");
    // printCoil(11036, "X13 X-Axis Limit");
    // printCoil(11027, "X6 Z-Axis Limit");

    // Outputs
    // printCoil(241, "Y20 Cap Push");
    // printCoil(10257, "Y21 DC Motor");
    // printCoil(10258, "Y22 Conveyor");
    // printCoil(10259, "Y23 Pump");
    // printCoil(10260, "Y24 Actuator");
    // printCoil(10261, "Y25 Red LED");
    // printCoil(10262, "Y26 Green LED");
    // printCoil(10263, "Y27 Compressor");

    // Bottle Counter
    printRegister(44097, "Bottle Count D0");

    Serial.println("========================================");
}

#endif