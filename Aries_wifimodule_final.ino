#include <Arduino.h>
#include <HardwareSerial.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

// ============================================================
//              ARIES V2 + EM6400NG+
//        RS485 + ESP-01 + MQTT + CSV
// ============================================================


// ============================================================
//                         RS485
// ============================================================

#define RS485_DE       7
#define RS485_RE       8

#define SLAVE_ID       52
#define METER_BAUD     9600

HardwareSerial meter(1);


// ============================================================
//                         ESP-01
// ============================================================

#define ESP_ENABLE_PIN 25
#define ESP_BAUD       115200

HardwareSerial esp(2);


// ============================================================
//                         WIFI
// ============================================================

#define WIFI_SSID      "5500"
#define WIFI_PASS      "55004554"


// ============================================================
//                         MQTT
// ============================================================

#define MQTT_HOST      "broker.hivemq.com"
#define MQTT_PORT      1883

#define CLIENT_ID      "ARIES_V2_EM6400_001"
#define MQTT_TOPIC     "aries/em6400/data"


// ============================================================
//                 EM6400NG+ REGISTERS
// ============================================================

// Current
#define REG_CURRENT_A      2999
#define REG_CURRENT_B      3001
#define REG_CURRENT_C      3003
#define REG_CURRENT_AVG    3009

// Voltage
#define REG_VLL_AVG        3025

// Power Factor
#define REG_PF_TOTAL       3083
#define REG_PF_ALT         3191

// Frequency
#define REG_FREQUENCY      3109

// Energy
#define REG_ENERGY         3203


// ============================================================
//                         STATUS
// ============================================================

bool wifiConnected = false;
bool mqttConnected = false;


// ============================================================
//              FLOAT -> TEXT WITHOUT dtostrf()
// ============================================================
//
// IMPORTANT:
//
// Do NOT use:
//   String(value, 3)
//   snprintf(... "%f" ...)
//
// This function manually converts the value to:
//   123.456
//
// It is used because the VEGA ARIES v2 core is missing
// dtostrf(), which is normally used by Arduino String(float).
//
// ============================================================

void appendFloat3(
    char *out,
    int &pos,
    float value
)
{
    // --------------------------------------------------------
    // NaN
    // --------------------------------------------------------

    if (isnan(value))
    {
        out[pos++] = 'n';
        out[pos++] = 'a';
        out[pos++] = 'n';

        out[pos] = '\0';

        return;
    }


    // --------------------------------------------------------
    // Infinity
    // --------------------------------------------------------

    if (isinf(value))
    {
        if (value < 0)
        {
            out[pos++] = '-';
        }

        out[pos++] = 'i';
        out[pos++] = 'n';
        out[pos++] = 'f';

        out[pos] = '\0';

        return;
    }


    // --------------------------------------------------------
    // Negative
    // --------------------------------------------------------

    if (value < 0)
    {
        out[pos++] = '-';

        value = -value;
    }


    // --------------------------------------------------------
    // Scale to 3 decimal places
    // --------------------------------------------------------

    unsigned long scaled =
        (unsigned long)(
            value * 1000.0f + 0.5f
        );


    unsigned long whole =
        scaled / 1000;


    unsigned long fraction =
        scaled % 1000;


    // --------------------------------------------------------
    // Convert whole number
    // --------------------------------------------------------

    char temp[16];

    int t = 0;


    if (whole == 0)
    {
        temp[t++] = '0';
    }
    else
    {
        while (whole > 0)
        {
            temp[t++] =
                '0' + (whole % 10);

            whole /= 10;
        }
    }


    // --------------------------------------------------------
    // Reverse whole number
    // --------------------------------------------------------

    while (t > 0)
    {
        out[pos++] =
            temp[--t];
    }


    // --------------------------------------------------------
    // Decimal point
    // --------------------------------------------------------

    out[pos++] = '.';


    // --------------------------------------------------------
    // Three decimal digits
    // --------------------------------------------------------

    out[pos++] =
        '0' + ((fraction / 100) % 10);

    out[pos++] =
        '0' + ((fraction / 10) % 10);

    out[pos++] =
        '0' + (fraction % 10);


    out[pos] = '\0';
}


// ============================================================
//                     MODBUS CRC16
// ============================================================

uint16_t modbusCRC(
    uint8_t *buf,
    uint8_t len
)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t pos = 0; pos < len; pos++)
    {
        crc ^= buf[pos];

        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 1)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}


// ============================================================
//                    RS485 RECEIVE
// ============================================================

void rs485Receive()
{
    digitalWrite(
        RS485_DE,
        LOW
    );

    digitalWrite(
        RS485_RE,
        LOW
    );
}


// ============================================================
//                    RS485 TRANSMIT
// ============================================================

void rs485Transmit()
{
    digitalWrite(
        RS485_DE,
        HIGH
    );

    digitalWrite(
        RS485_RE,
        HIGH
    );
}


// ============================================================
//                    RS485 INIT
// ============================================================

void initModbus()
{
    pinMode(
        RS485_DE,
        OUTPUT
    );

    pinMode(
        RS485_RE,
        OUTPUT
    );

    rs485Receive();

    meter.begin(
        METER_BAUD
    );

    Serial.println();
    Serial.println(
        "================================"
    );
    Serial.println(
        "          RS485 INIT"
    );
    Serial.println(
        "================================"
    );

    Serial.print(
        "Baud Rate : "
    );

    Serial.println(
        METER_BAUD
    );

    Serial.print(
        "Slave ID  : "
    );

    Serial.println(
        SLAVE_ID
    );

    Serial.println(
        "RS485 READY"
    );
}


// ============================================================
//                CLEAR METER BUFFER
// ============================================================

void clearMeterBuffer()
{
    while (meter.available())
    {
        meter.read();
    }
}


// ============================================================
//                  READ FLOAT32
// ============================================================

bool readFloatRegister(
    uint16_t address,
    float &value
)
{
    uint8_t request[8];


    // --------------------------------------------------------
    // MODBUS REQUEST
    // --------------------------------------------------------

    request[0] = SLAVE_ID;
    request[1] = 0x03;

    request[2] =
        highByte(address);

    request[3] =
        lowByte(address);

    // Read 2 registers = 4 bytes
    request[4] = 0x00;
    request[5] = 0x02;


    // --------------------------------------------------------
    // CRC
    // --------------------------------------------------------

    uint16_t crc =
        modbusCRC(
            request,
            6
        );

    request[6] =
        lowByte(crc);

    request[7] =
        highByte(crc);


    // --------------------------------------------------------
    // CLEAR OLD DATA
    // --------------------------------------------------------

    clearMeterBuffer();


    // --------------------------------------------------------
    // TRANSMIT
    // --------------------------------------------------------

    rs485Transmit();

    delay(2);

    meter.write(
        request,
        8
    );

    meter.flush();

    delay(2);


    // --------------------------------------------------------
    // RECEIVE
    // --------------------------------------------------------

    rs485Receive();


    uint8_t response[32];

    uint8_t count = 0;

    unsigned long start =
        millis();


    while (
        millis() - start < 1500
    )
    {
        while (meter.available())
        {
            if (
                count <
                sizeof(response)
            )
            {
                response[count++] =
                    meter.read();
            }
            else
            {
                meter.read();
            }
        }

        if (count >= 9)
        {
            break;
        }
    }


    // --------------------------------------------------------
    // TIMEOUT
    // --------------------------------------------------------

    if (count < 9)
    {
        Serial.print(
            "TIMEOUT register "
        );

        Serial.println(
            address
        );

        return false;
    }


    // --------------------------------------------------------
    // SLAVE ID
    // --------------------------------------------------------

    if (
        response[0] != SLAVE_ID
    )
    {
        Serial.print(
            "WRONG SLAVE register "
        );

        Serial.println(
            address
        );

        return false;
    }


    // --------------------------------------------------------
    // FUNCTION
    // --------------------------------------------------------

    if (
        response[1] != 0x03
    )
    {
        Serial.print(
            "WRONG FUNCTION register "
        );

        Serial.println(
            address
        );

        return false;
    }


    // --------------------------------------------------------
    // BYTE COUNT
    // --------------------------------------------------------

    if (
        response[2] != 0x04
    )
    {
        Serial.print(
            "WRONG BYTE COUNT register "
        );

        Serial.println(
            address
        );

        return false;
    }


    // --------------------------------------------------------
    // RAW DATA
    // --------------------------------------------------------

    Serial.print(
        "Reg "
    );

    Serial.print(
        address
    );

    Serial.print(
        " RAW: "
    );


    for (
        uint8_t i = 3;
        i < 7;
        i++
    )
    {
        if (
            response[i] < 0x10
        )
        {
            Serial.print(
                "0"
            );
        }

        Serial.print(
            response[i],
            HEX
        );

        Serial.print(
            " "
        );
    }

    Serial.println();


    // --------------------------------------------------------
    // FLOAT32
    // --------------------------------------------------------

    uint32_t raw =
        ((uint32_t)response[3] << 24) |
        ((uint32_t)response[4] << 16) |
        ((uint32_t)response[5] << 8)  |
        ((uint32_t)response[6]);


    memcpy(
        &value,
        &raw,
        sizeof(float)
    );


    return true;
}


// ============================================================
//                  READ ENERGY 3203
// ============================================================
//
// Register 3203
// 4 Modbus registers
// 8 bytes
// INT64
// Wh
//
// Converted to kWh by dividing by 1000.
//
// ============================================================

bool readEnergyRegister(
    uint16_t address,
    double &energyKWh
)
{
    uint8_t request[8];


    // --------------------------------------------------------
    // REQUEST
    // --------------------------------------------------------

    request[0] = SLAVE_ID;
    request[1] = 0x03;

    request[2] =
        highByte(address);

    request[3] =
        lowByte(address);

    // Read 4 registers
    request[4] = 0x00;
    request[5] = 0x04;


    // --------------------------------------------------------
    // CRC
    // --------------------------------------------------------

    uint16_t crc =
        modbusCRC(
            request,
            6
        );

    request[6] =
        lowByte(crc);

    request[7] =
        highByte(crc);


    clearMeterBuffer();


    // --------------------------------------------------------
    // TRANSMIT
    // --------------------------------------------------------

    rs485Transmit();

    delay(2);

    meter.write(
        request,
        8
    );

    meter.flush();

    delay(2);


    // --------------------------------------------------------
    // RECEIVE
    // --------------------------------------------------------

    rs485Receive();


    uint8_t response[32];

    uint8_t count = 0;

    unsigned long start =
        millis();


    while (
        millis() - start < 2000
    )
    {
        while (meter.available())
        {
            if (
                count <
                sizeof(response)
            )
            {
                response[count++] =
                    meter.read();
            }
            else
            {
                meter.read();
            }
        }

        if (count >= 13)
        {
            break;
        }
    }


    // --------------------------------------------------------
    // CHECK
    // --------------------------------------------------------

    if (count < 13)
    {
        Serial.println(
            "ENERGY TIMEOUT"
        );

        return false;
    }


    if (
        response[0] != SLAVE_ID
    )
    {
        Serial.println(
            "ENERGY WRONG SLAVE"
        );

        return false;
    }


    if (
        response[1] != 0x03
    )
    {
        Serial.println(
            "ENERGY WRONG FUNCTION"
        );

        return false;
    }


    if (
        response[2] != 0x08
    )
    {
        Serial.print(
            "ENERGY WRONG BYTE COUNT: "
        );

        Serial.println(
            response[2]
        );

        return false;
    }


    // --------------------------------------------------------
    // RAW DATA
    // --------------------------------------------------------

    Serial.print(
        "Reg 3203 RAW: "
    );


    for (
        uint8_t i = 3;
        i < 11;
        i++
    )
    {
        if (
            response[i] < 0x10
        )
        {
            Serial.print(
                "0"
            );
        }

        Serial.print(
            response[i],
            HEX
        );

        Serial.print(
            " "
        );
    }

    Serial.println();


    // --------------------------------------------------------
    // INT64
    // --------------------------------------------------------

    uint64_t raw =
        ((uint64_t)response[3] << 56) |
        ((uint64_t)response[4] << 48) |
        ((uint64_t)response[5] << 40) |
        ((uint64_t)response[6] << 32) |
        ((uint64_t)response[7] << 24) |
        ((uint64_t)response[8] << 16) |
        ((uint64_t)response[9] << 8)  |
        ((uint64_t)response[10]);


    // Wh -> kWh

    energyKWh =
        (double)raw / 1000.0;


    return true;
}


// ============================================================
//                     CLEAR ESP BUFFER
// ============================================================

void clearESP()
{
    while (esp.available())
    {
        esp.read();
    }
}


// ============================================================
//                    SEND AT COMMAND
// ============================================================

bool sendAT(
    const char *command,
    const char *expected,
    unsigned long timeout
)
{
    clearESP();


    Serial.print(
        ">> "
    );

    Serial.println(
        command
    );


    esp.print(
        command
    );

    esp.print(
        "\r\n"
    );


    String response = "";


    unsigned long start =
        millis();


    while (
        millis() - start < timeout
    )
    {
        while (esp.available())
        {
            char c =
                esp.read();


            Serial.write(c);


            response += c;


            if (
                response.indexOf(
                    expected
                ) >= 0
            )
            {
                return true;
            }


            if (
                response.indexOf(
                    "ERROR"
                ) >= 0
            )
            {
                return false;
            }


            if (
                response.indexOf(
                    "FAIL"
                ) >= 0
            )
            {
                return false;
            }
        }
    }


    return false;
}


// ============================================================
//                    WIFI CONNECT
// ============================================================

bool connectWiFi()
{
    Serial.println();

    Serial.println(
        "================================"
    );

    Serial.println(
        "          WIFI CONNECT"
    );

    Serial.println(
        "================================"
    );


    // --------------------------------------------------------
    // ESP ENABLE
    // --------------------------------------------------------

    pinMode(
        ESP_ENABLE_PIN,
        OUTPUT
    );

    digitalWrite(
        ESP_ENABLE_PIN,
        HIGH
    );


    delay(1000);


    // --------------------------------------------------------
    // ESP UART
    // --------------------------------------------------------

    esp.begin(
        ESP_BAUD
    );


    delay(1000);


    // --------------------------------------------------------
    // AT TEST
    // --------------------------------------------------------

    if (
        !sendAT(
            "AT",
            "OK",
            3000
        )
    )
    {
        Serial.println(
            "ESP-01 NOT RESPONDING"
        );

        wifiConnected = false;

        return false;
    }


    Serial.println(
        "ESP-01 RESPONDING"
    );


    // --------------------------------------------------------
    // ECHO OFF
    // --------------------------------------------------------

    sendAT(
        "ATE0",
        "OK",
        3000
    );


    // --------------------------------------------------------
    // STATION MODE
    // --------------------------------------------------------

    if (
        !sendAT(
            "AT+CWMODE=1",
            "OK",
            3000
        )
    )
    {
        Serial.println(
            "CWMODE FAILED"
        );

        wifiConnected = false;

        return false;
    }


    // --------------------------------------------------------
    // WIFI JOIN
    // --------------------------------------------------------

    char command[160];


    snprintf(
        command,
        sizeof(command),
        "AT+CWJAP=\"%s\",\"%s\"",
        WIFI_SSID,
        WIFI_PASS
    );


    Serial.println();

    Serial.println(
        "Connecting to WiFi..."
    );


    if (
        !sendAT(
            command,
            "WIFI GOT IP",
            30000
        )
    )
    {
        Serial.println(
            "WIFI FAILED"
        );

        wifiConnected = false;

        return false;
    }


    Serial.println();

    Serial.println(
        "WIFI GOT IP"
    );


    // --------------------------------------------------------
    // GET IP
    // --------------------------------------------------------

    sendAT(
        "AT+CIFSR",
        "OK",
        5000
    );


    wifiConnected = true;


    Serial.println(
        "WIFI SUCCESS"
    );


    return true;
}


// ============================================================
//              WAIT FOR MQTT CONNACK
// ============================================================

bool waitForMQTTConnack(
    unsigned long timeout
)
{
    // MQTT CONNACK:
    //
    // 20 02 00 00
    //
    // 20 = CONNACK
    // 02 = Remaining length
    // 00 = Session present
    // 00 = Connection accepted


    uint8_t state = 0;


    unsigned long start =
        millis();


    while (
        millis() - start < timeout
    )
    {
        while (esp.available())
        {
            uint8_t c =
                esp.read();


            Serial.print(
                "[ESP 0x"
            );


            if (c < 0x10)
            {
                Serial.print(
                    "0"
                );
            }


            Serial.print(
                c,
                HEX
            );


            Serial.println(
                "]"
            );


            // ------------------------------------------------
            // STATE 0
            // ------------------------------------------------

            if (state == 0)
            {
                if (c == 0x20)
                {
                    state = 1;
                }
            }


            // ------------------------------------------------
            // STATE 1
            // ------------------------------------------------

            else if (state == 1)
            {
                if (c == 0x02)
                {
                    state = 2;
                }
                else if (c == 0x20)
                {
                    state = 1;
                }
                else
                {
                    state = 0;
                }
            }


            // ------------------------------------------------
            // STATE 2
            // ------------------------------------------------

            else if (state == 2)
            {
                if (c == 0x00)
                {
                    state = 3;
                }
                else
                {
                    state = 0;
                }
            }


            // ------------------------------------------------
            // STATE 3
            // ------------------------------------------------

            else if (state == 3)
            {
                if (c == 0x00)
                {
                    return true;
                }

                state = 0;
            }
        }
    }


    return false;
}


// ============================================================
//                    MQTT CONNECT
// ============================================================

bool connectMQTT()
{
    Serial.println();

    Serial.println(
        "================================"
    );

    Serial.println(
        "          MQTT CONNECT"
    );

    Serial.println(
        "================================"
    );


    mqttConnected = false;


    // --------------------------------------------------------
    // SINGLE CONNECTION
    // --------------------------------------------------------

    if (
        !sendAT(
            "AT+CIPMUX=0",
            "OK",
            3000
        )
    )
    {
        Serial.println(
            "CIPMUX FAILED"
        );
    }


    // --------------------------------------------------------
    // NORMAL MODE
    // --------------------------------------------------------

    sendAT(
        "AT+CIPMODE=0",
        "OK",
        3000
    );


    // --------------------------------------------------------
    // CLOSE OLD CONNECTION
    // --------------------------------------------------------

    sendAT(
        "AT+CIPCLOSE",
        "OK",
        2000
    );


    delay(500);


    // --------------------------------------------------------
    // TCP CONNECT
    // --------------------------------------------------------

    char command[160];


    snprintf(
        command,
        sizeof(command),
        "AT+CIPSTART=\"TCP\",\"%s\",%d",
        MQTT_HOST,
        MQTT_PORT
    );


    Serial.println();

    Serial.println(
        "Connecting to HiveMQ..."
    );


    if (
        !sendAT(
            command,
            "CONNECT",
            15000
        )
    )
    {
        Serial.println(
            "TCP CONNECTION FAILED"
        );

        return false;
    }


    Serial.println(
        "TCP CONNECTED"
    );


    delay(1000);


    // --------------------------------------------------------
    // MQTT CONNECT PACKET
    // --------------------------------------------------------

    const char *clientID =
        CLIENT_ID;


    uint16_t clientLength =
        strlen(
            clientID
        );


    // MQTT 3.1.1 CONNECT:
    //
    // Variable header = 10 bytes
    // Client ID length = 2 bytes
    // Client ID = clientLength
    //

    uint16_t remainingLength =
        10 +
        2 +
        clientLength;


    uint8_t packet[128];


    int p = 0;


    // --------------------------------------------------------
    // FIXED HEADER
    // --------------------------------------------------------

    packet[p++] =
        0x10;


    packet[p++] =
        (uint8_t)remainingLength;


    // --------------------------------------------------------
    // PROTOCOL NAME
    // --------------------------------------------------------

    packet[p++] = 0x00;
    packet[p++] = 0x04;

    packet[p++] = 'M';
    packet[p++] = 'Q';
    packet[p++] = 'T';
    packet[p++] = 'T';


    // MQTT 3.1.1
    packet[p++] =
        0x04;


    // Clean session
    packet[p++] =
        0x02;


    // Keep alive = 60 seconds
    packet[p++] =
        0x00;

    packet[p++] =
        0x3C;


    // --------------------------------------------------------
    // CLIENT ID LENGTH
    // --------------------------------------------------------

    packet[p++] =
        highByte(
            clientLength
        );

    packet[p++] =
        lowByte(
            clientLength
        );


    // --------------------------------------------------------
    // CLIENT ID
    // --------------------------------------------------------

    memcpy(
        &packet[p],
        clientID,
        clientLength
    );


    p += clientLength;


    // --------------------------------------------------------
    // CIPSEND
    // --------------------------------------------------------

    snprintf(
        command,
        sizeof(command),
        "AT+CIPSEND=%d",
        p
    );


    Serial.print(
        "MQTT packet size: "
    );

    Serial.println(
        p
    );


    if (
        !sendAT(
            command,
            ">",
            5000
        )
    )
    {
        Serial.println(
            "MQTT CIPSEND FAILED"
        );

        return false;
    }


    // --------------------------------------------------------
    // SEND CONNECT
    // --------------------------------------------------------

    Serial.println(
        "Sending MQTT CONNECT..."
    );


    esp.write(
        packet,
        p
    );


    // --------------------------------------------------------
    // WAIT CONNACK
    // --------------------------------------------------------

    Serial.println(
        "Waiting for MQTT CONNACK..."
    );


    if (
        waitForMQTTConnack(
            8000
        )
    )
    {
        Serial.println();

        Serial.println(
            "********************************"
        );

        Serial.println(
            "     MQTT CONNECT SUCCESS"
        );

        Serial.println(
            "********************************"
        );


        mqttConnected = true;


        return true;
    }


    Serial.println();

    Serial.println(
        "MQTT CONNACK TIMEOUT"
    );


    return false;
}


// ============================================================
//                    MQTT PUBLISH
// ============================================================

bool publishMQTT(
    const char *payload
)
{
    Serial.println();

    Serial.println(
        "================================"
    );

    Serial.println(
        "          MQTT PUBLISH"
    );

    Serial.println(
        "================================"
    );


    if (!mqttConnected)
    {
        Serial.println(
            "MQTT NOT CONNECTED"
        );

        return false;
    }


    // --------------------------------------------------------
    // LENGTHS
    // --------------------------------------------------------

    uint16_t topicLength =
        strlen(
            MQTT_TOPIC
        );


    uint16_t payloadLength =
        strlen(
            payload
        );


    uint16_t remainingLength =
        2 +
        topicLength +
        payloadLength;


    // --------------------------------------------------------
    // SINGLE BYTE MQTT REMAINING LENGTH
    // --------------------------------------------------------

    if (
        remainingLength >= 128
    )
    {
        Serial.println(
            "MQTT PAYLOAD TOO LARGE"
        );

        return false;
    }


    // --------------------------------------------------------
    // PACKET
    // --------------------------------------------------------

    uint8_t packet[400];


    int p = 0;


    // PUBLISH QoS 0
    packet[p++] =
        0x30;


    // Remaining length
    packet[p++] =
        (uint8_t)remainingLength;


    // --------------------------------------------------------
    // TOPIC LENGTH
    // --------------------------------------------------------

    packet[p++] =
        highByte(
            topicLength
        );

    packet[p++] =
        lowByte(
            topicLength
        );


    // --------------------------------------------------------
    // TOPIC
    // --------------------------------------------------------

    memcpy(
        &packet[p],
        MQTT_TOPIC,
        topicLength
    );


    p += topicLength;


    // --------------------------------------------------------
    // PAYLOAD
    // --------------------------------------------------------

    memcpy(
        &packet[p],
        payload,
        payloadLength
    );


    p += payloadLength;


    // --------------------------------------------------------
    // DEBUG
    // --------------------------------------------------------

    Serial.print(
        "Topic   : "
    );

    Serial.println(
        MQTT_TOPIC
    );


    Serial.print(
        "Payload : "
    );

    Serial.println(
        payload
    );


    Serial.print(
        "Bytes   : "
    );

    Serial.println(
        p
    );


    // --------------------------------------------------------
    // CIPSEND
    // --------------------------------------------------------

    char command[50];


    snprintf(
        command,
        sizeof(command),
        "AT+CIPSEND=%d",
        p
    );


    Serial.print(
        "Sending: "
    );

    Serial.println(
        command
    );


    if (
        !sendAT(
            command,
            ">",
            5000
        )
    )
    {
        Serial.println(
            "NO SEND PROMPT"
        );

        mqttConnected = false;

        return false;
    }


    // --------------------------------------------------------
    // SEND MQTT PUBLISH
    // --------------------------------------------------------

    Serial.println(
        "Sending MQTT PUBLISH packet..."
    );


    esp.write(
        packet,
        p
    );


    // --------------------------------------------------------
    // WAIT SEND OK
    // --------------------------------------------------------

    String response = "";


    unsigned long start =
        millis();


    while (
        millis() - start < 8000
    )
    {
        while (esp.available())
        {
            char c =
                esp.read();


            Serial.write(
                c
            );


            response += c;


            if (
                response.indexOf(
                    "SEND OK"
                ) >= 0
            )
            {
                Serial.println();

                Serial.println(
                    "********************************"
                );

                Serial.println(
                    "     MQTT PUBLISH SUCCESS"
                );

                Serial.println(
                    "********************************"
                );


                return true;
            }


            if (
                response.indexOf(
                    "ERROR"
                ) >= 0
            )
            {
                Serial.println();

                Serial.println(
                    "MQTT PUBLISH ERROR"
                );


                mqttConnected = false;


                return false;
            }


            if (
                response.indexOf(
                    "CLOSED"
                ) >= 0
            )
            {
                Serial.println();

                Serial.println(
                    "MQTT CONNECTION CLOSED"
                );


                mqttConnected = false;


                return false;
            }
        }
    }


    Serial.println();

    Serial.println(
        "MQTT SEND TIMEOUT"
    );


    mqttConnected = false;


    return false;
}


// ============================================================
//                         SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200
    );


    delay(2000);


    Serial.println();

    Serial.println(
        "=========================================="
    );

    Serial.println(
        "       ARIES V2 + EM6400NG+"
    );

    Serial.println(
        "       RS485 + ESP-01 + MQTT"
    );

    Serial.println(
        "=========================================="
    );


    // --------------------------------------------------------
    // RS485
    // --------------------------------------------------------

    initModbus();


    // --------------------------------------------------------
    // WIFI
    // --------------------------------------------------------

    while (
        !connectWiFi()
    )
    {
        Serial.println();

        Serial.println(
            "Retrying WiFi in 3 seconds..."
        );

        delay(3000);
    }


    // --------------------------------------------------------
    // MQTT
    // --------------------------------------------------------

    while (
        !connectMQTT()
    )
    {
        Serial.println();

        Serial.println(
            "Retrying MQTT in 3 seconds..."
        );

        delay(3000);
    }


    Serial.println();

    Serial.println(
        "=========================================="
    );

    Serial.println(
        "             SYSTEM READY"
    );

    Serial.println(
        "=========================================="
    );
}


// ============================================================
//                         LOOP
// ============================================================

void loop()
{
    // ========================================================
    // VARIABLES
    // ========================================================

    float currentA = 0.0f;
    float currentB = 0.0f;
    float currentC = 0.0f;
    float currentAvg = 0.0f;

    float vll = 0.0f;

    float pf = NAN;
    float pfAlt = NAN;

    float frequency = 0.0f;

    double energy = 0.0;


    bool okA;
    bool okB;
    bool okC;
    bool okAvg;
    bool okVLL;
    bool okPF;
    bool okPFAlt;
    bool okFrequency;
    bool okEnergy;


    // ========================================================
    // FETCH METER DATA
    // ========================================================

    Serial.println();

    Serial.println(
        "=========================================="
    );

    Serial.println(
        "       FETCHING EM6400NG+ DATA"
    );

    Serial.println(
        "=========================================="
    );


    // Current A
    okA =
        readFloatRegister(
            REG_CURRENT_A,
            currentA
        );


    // Current B
    okB =
        readFloatRegister(
            REG_CURRENT_B,
            currentB
        );


    // Current C
    okC =
        readFloatRegister(
            REG_CURRENT_C,
            currentC
        );


    // Current AVG
    okAvg =
        readFloatRegister(
            REG_CURRENT_AVG,
            currentAvg
        );


    // VLL AVG
    okVLL =
        readFloatRegister(
            REG_VLL_AVG,
            vll
        );


    // PF TOTAL
    okPF =
        readFloatRegister(
            REG_PF_TOTAL,
            pf
        );


    // PF ALT
    okPFAlt =
        readFloatRegister(
            REG_PF_ALT,
            pfAlt
        );


    // Frequency
    okFrequency =
        readFloatRegister(
            REG_FREQUENCY,
            frequency
        );


    // Energy
    okEnergy =
        readEnergyRegister(
            REG_ENERGY,
            energy
        );


    // ========================================================
    // DISPLAY VALUES
    // ========================================================

    Serial.println();

    Serial.println(
        "----------- EM6400NG+ -----------"
    );


    Serial.print(
        "Current A  : "
    );

    Serial.print(
        currentA,
        3
    );

    Serial.print(
        " A ["
    );

    Serial.print(
        okA ? "OK" : "FAILED"
    );

    Serial.println(
        "]"
    );


    Serial.print(
        "Current B  : "
    );

    Serial.print(
        currentB,
        3
    );

    Serial.print(
        " A ["
    );

    Serial.print(
        okB ? "OK" : "FAILED"
    );

    Serial.println(
        "]"
    );


    Serial.print(
        "Current C  : "
    );

    Serial.print(
        currentC,
        3
    );

    Serial.print(
        " A ["
    );

    Serial.print(
        okC ? "OK" : "FAILED"
    );

    Serial.println(
        "]"
    );


    Serial.print(
        "Current AVG: "
    );

    Serial.print(
        currentAvg,
        3
    );

    Serial.print(
        " A ["
    );

    Serial.print(
        okAvg ? "OK" : "FAILED"
    );

    Serial.println(
        "]"
    );


    Serial.print(
        "VLL AVG    : "
    );

    Serial.print(
        vll,
        3
    );

    Serial.print(
        " V ["
    );

    Serial.print(
        okVLL ? "OK" : "FAILED"
    );

    Serial.println(
        "]"
    );


    Serial.print(
        "PF TOTAL   : "
    );


    if (isnan(pf))
    {
        Serial.print(
            "nan"
        );
    }
    else
    {
        Serial.print(
            pf,
            3
        );
    }


    Serial.print(
        " ["
    );

    Serial.print(
        okPF ? "OK" : "FAILED"
    );

    Serial.println(
        "]"
    );


    Serial.print(
        "PF ALT     : "
    );


    if (isnan(pfAlt))
    {
        Serial.print(
            "nan"
        );
    }
    else
    {
        Serial.print(
            pfAlt,
            3
        );
    }


    Serial.print(
        " ["
    );

    Serial.print(
        okPFAlt ? "OK" : "FAILED"
    );

    Serial.println(
        "]"
    );


    Serial.print(
        "Frequency  : "
    );

    Serial.print(
        frequency,
        3
    );

    Serial.print(
        " Hz ["
    );

    Serial.print(
        okFrequency ? "OK" : "FAILED"
    );

    Serial.println(
        "]"
    );


    Serial.print(
        "Energy     : "
    );

    Serial.print(
        energy,
        3
    );

    Serial.print(
        " kWh ["
    );

    Serial.print(
        okEnergy ? "OK" : "FAILED"
    );

    Serial.println(
        "]"
    );


    Serial.println(
        "------------------------------------------"
    );


    // ========================================================
    // CHECKPOINT 1
    // ========================================================

    Serial.println(
        "CHECKPOINT 1: METER READ COMPLETE"
    );


    // ========================================================
    // CREATE CSV
    // ========================================================
    //
    // IMPORTANT:
    // No String(float, 3)
    // No snprintf("%f")
    //
    // ========================================================

    Serial.println(
        "CHECKPOINT 2: STARTING CSV CREATION"
    );


    char payload[300];

    int pos = 0;


    // --------------------------------------------------------
    // DEVICE NAME
    // --------------------------------------------------------

    // const char *name =
    //     "ARIES_V2";


    // while (*name)
    // {
    //     payload[pos++] =
    //         *name++;
    // }


    // payload[pos++] =
    //     ',';


    // --------------------------------------------------------
    // CURRENT A
    // --------------------------------------------------------

    appendFloat3(
        payload,
        pos,
        currentA
    );

    payload[pos++] =
        ',';


    // --------------------------------------------------------
    // CURRENT B
    // --------------------------------------------------------

    appendFloat3(
        payload,
        pos,
        currentB
    );

    payload[pos++] =
        ',';


    // --------------------------------------------------------
    // CURRENT C
    // --------------------------------------------------------

    appendFloat3(
        payload,
        pos,
        currentC
    );

    payload[pos++] =
        ',';


    // --------------------------------------------------------
    // CURRENT AVG
    // --------------------------------------------------------

    appendFloat3(
        payload,
        pos,
        currentAvg
    );

    payload[pos++] =
        ',';


    // --------------------------------------------------------
    // VLL AVG
    // --------------------------------------------------------

    appendFloat3(
        payload,
        pos,
        vll
    );

    payload[pos++] =
        ',';


    // --------------------------------------------------------
    // PF TOTAL
    // --------------------------------------------------------

    appendFloat3(
        payload,
        pos,
        pf
    );

    payload[pos++] =
        ',';


    // --------------------------------------------------------
    // PF ALT
    // --------------------------------------------------------

    appendFloat3(
        payload,
        pos,
        pfAlt
    );

    payload[pos++] =
        ',';


    // --------------------------------------------------------
    // FREQUENCY
    // --------------------------------------------------------

    appendFloat3(
        payload,
        pos,
        frequency
    );

    payload[pos++] =
        ',';


    // --------------------------------------------------------
    // ENERGY
    // --------------------------------------------------------
    //
    // Energy is converted to float only here for the CSV.
    // This is sufficient for normal energy-meter values.
    //
    // --------------------------------------------------------

    appendFloat3(
        payload,
        pos,
        (float)energy
    );


    // --------------------------------------------------------
    // TERMINATE STRING
    // --------------------------------------------------------

    payload[pos] =
        '\0';


    // ========================================================
    // CSV OUTPUT
    // ========================================================

    Serial.println();

    Serial.println(
        "=========================================="
    );

    Serial.println(
        "                 CSV DATA"
    );

    Serial.println(
        "=========================================="
    );


    Serial.println(
        payload
    );


    Serial.println(
        "=========================================="
    );


    Serial.println(
        "CHECKPOINT 3: CSV CREATED"
    );


    // ========================================================
    // WIFI CHECK
    // ========================================================

    Serial.println(
        "CHECKPOINT 4: CHECKING WIFI"
    );


    if (!wifiConnected)
    {
        Serial.println(
            "WiFi disconnected. Reconnecting..."
        );


        if (
            !connectWiFi()
        )
        {
            Serial.println(
                "WiFi reconnect FAILED"
            );


            delay(3000);


            return;
        }
    }


    Serial.println(
        "CHECKPOINT 5: WIFI OK"
    );


    // ========================================================
    // MQTT CHECK
    // ========================================================

    if (!mqttConnected)
    {
        Serial.println(
            "MQTT disconnected. Reconnecting..."
        );


        if (
            !connectMQTT()
        )
        {
            Serial.println(
                "MQTT reconnect FAILED"
            );


            delay(3000);


            return;
        }
    }


    Serial.println(
        "CHECKPOINT 6: MQTT OK"
    );


    // ========================================================
    // PUBLISH
    // ========================================================

    Serial.println();

    Serial.println(
        "CHECKPOINT 7: STARTING MQTT PUBLISH"
    );


    bool publishResult =
        publishMQTT(
            payload
        );


    // ========================================================
    // RESULT
    // ========================================================

    if (publishResult)
    {
        Serial.println();

        Serial.println(
            "=========================================="
        );

        Serial.println(
            "       CSV PUBLISHED SUCCESSFULLY"
        );

        Serial.println(
            "=========================================="
        );
    }
    else
    {
        Serial.println();

        Serial.println(
            "=========================================="
        );

        Serial.println(
            "             CSV PUBLISH FAILED"
        );

        Serial.println(
            "=========================================="
        );


        mqttConnected =
            false;
    }


    // ========================================================
    // FINAL CHECKPOINT
    // ========================================================

    Serial.println(
        "CHECKPOINT 8: PUBLISH COMPLETE"
    );


    // ========================================================
    // WAIT
    // ========================================================

    Serial.println();

    Serial.println(
        "Waiting 5 seconds..."
    );


    delay(5000);
}