#ifndef GET_DATA_H
#define GET_DATA_H

#include <Arduino.h>
#include <HardwareSerial.h>

#include <SPI.h>
#include <WiFiNINA.h>

#include <PubSubClient.h>
#include <ArduinoJson.h>


// =====================================================
// MAX485 CONTROL PINS
// =====================================================

#define RS485_DE 7
#define RS485_RE 8


// =====================================================
// ARIES UART1
// =====================================================

HardwareSerial maxsensor(1);


// =====================================================
// EM6400 NG+ SETTINGS
// =====================================================

#define SLAVE_ID 52


// =====================================================
// WIFI SETTINGS
// =====================================================

#define WIFI_SSID       "DHA"
#define WIFI_PASSWORD   "12345678"


// =====================================================
// MQTT SETTINGS
// =====================================================

#define MQTT_SERVER     "broker.hivemq.com"
#define MQTT_PORT       1883

// #define MQTT_USER       "YOUR_MQTT_USERNAME"
// #define MQTT_PASSWORD   "YOUR_MQTT_PASSWORD"

#define MQTT_TOPIC      "aries/em6400/data"


// =====================================================
// WIFI / MQTT OBJECTS
// =====================================================

WiFiClient wifiClient;

PubSubClient mqttClient(wifiClient);


// =====================================================
// FUNCTION DECLARATIONS
// =====================================================

// IMPORTANT:
// connectToWIFI() calls printWifiStatus()
// so declare it before connectToWIFI().

void printWifiStatus();

void connectToWIFI();

void initMQTT();

void connectMQTT();


// =====================================================
// MODBUS CRC16
// =====================================================

uint16_t modbusCRC(uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t pos = 0; pos < len; pos++)
    {
        crc ^= buf[pos];

        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
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


// =====================================================
// INITIALIZE RS485
// =====================================================

void init_modbus()
{
    pinMode(RS485_DE, OUTPUT);
    pinMode(RS485_RE, OUTPUT);

    // Receive mode initially
    digitalWrite(RS485_DE, LOW);
    digitalWrite(RS485_RE, LOW);

    // EM6400 NG+
    // 9600 baud, 8N1
    maxsensor.begin(9600);

    delay(500);

    Serial.println("RS485 initialized");
    Serial.println("Baud Rate : 9600");
    Serial.println("Slave ID  : 52");
}


// =====================================================
// PRINT WIFI STATUS
// =====================================================

void printWifiStatus()
{
    Serial.println();

    Serial.println("----------------------------------------");
    Serial.println("Wi-Fi Status");
    Serial.println("----------------------------------------");

    // SSID
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // IP address
    IPAddress ip = WiFi.localIP();

    Serial.print("IP Address: ");
    Serial.println(ip);

    // RSSI
    long rssi = WiFi.RSSI();

    Serial.print("Signal strength: ");
    Serial.print(rssi);
    Serial.println(" dBm");

    Serial.println("----------------------------------------");
}


// =====================================================
// CONNECT TO WIFI
// =====================================================

void connectToWIFI()
{
    Serial.println();
    Serial.println("========================================");
    Serial.println("             WIFI CONNECTION");
    Serial.println("========================================");

    // -------------------------------------------------
    // CHECK WIFI MODULE
    // -------------------------------------------------

    if (WiFi.status() == WL_NO_MODULE)
    {
        Serial.println("Communication with WiFi module failed!");

        while (true)
        {
            delay(1000);
        }
    }


    // -------------------------------------------------
    // CHECK FIRMWARE
    // -------------------------------------------------

    String fv = WiFi.firmwareVersion();

    Serial.print("WiFi firmware: ");
    Serial.println(fv);

    if (fv < "1.0.0")
    {
        Serial.println("Warning: Please upgrade WiFi firmware");
    }


    // -------------------------------------------------
    // CONNECT TO WIFI
    // -------------------------------------------------

    int status = WiFi.status();

    while (status != WL_CONNECTED)
    {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(WIFI_SSID);

        status = WiFi.begin(
            WIFI_SSID,
            WIFI_PASSWORD
        );

        // Wait 10 seconds
        delay(10000);
    }


    // -------------------------------------------------
    // CONNECTED
    // -------------------------------------------------

    Serial.println();
    Serial.println("Wi-Fi connected!");

    printWifiStatus();
}


// =====================================================
// CONNECT TO MQTT
// =====================================================

void connectMQTT()
{
    // -------------------------------------------------
    // CHECK WIFI FIRST
    // -------------------------------------------------

    if (WiFi.status() != WL_CONNECTED)
    {
        connectToWIFI();

        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("Wi-Fi unavailable.");

            return;
        }
    }


    Serial.println();
    Serial.println("========================================");
    Serial.println("             MQTT CONNECTION");
    Serial.println("========================================");


    // -------------------------------------------------
    // MQTT CONNECTION
    // -------------------------------------------------

    while (!mqttClient.connected())
    {
        Serial.print("Connecting to MQTT broker... ");

        // Create unique client ID
        String clientID = "VEGA-EM6400-";

        clientID += String(millis());


        if (mqttClient.connect(
                clientID.c_str()))
                
        {
            Serial.println("CONNECTED");

            Serial.print("Broker : ");
            Serial.println(MQTT_SERVER);

            Serial.print("Port   : ");
            Serial.println(MQTT_PORT);

            Serial.print("Topic  : ");
            Serial.println(MQTT_TOPIC);
        }
        else
        {
            Serial.print("FAILED");

            Serial.print(" | MQTT state = ");

            Serial.println(
                mqttClient.state()
            );

            delay(3000);
        }
    }
}


// =====================================================
// INITIALIZE MQTT
// =====================================================

void initMQTT()
{
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

    // Increase MQTT packet size (default is 256 bytes)
    mqttClient.setBufferSize(1024);

    connectMQTT();
}


// =====================================================
// READ TWO MODBUS REGISTERS AS FLOAT
//
// address = zero-based Modbus register address
// =====================================================

bool readFloatAddress(
    uint16_t address,
    float &value
)
{
    uint8_t frame[8];


    // -------------------------------------------------
    // SLAVE ID
    // -------------------------------------------------

    frame[0] = SLAVE_ID;


    // -------------------------------------------------
    // FUNCTION CODE 03
    // -------------------------------------------------

    frame[1] = 0x03;


    // -------------------------------------------------
    // REGISTER ADDRESS
    // -------------------------------------------------

    frame[2] = (address >> 8) & 0xFF;

    frame[3] = address & 0xFF;


    // -------------------------------------------------
    // NUMBER OF REGISTERS = 2
    // -------------------------------------------------

    frame[4] = 0x00;

    frame[5] = 0x02;


    // -------------------------------------------------
    // CRC
    // -------------------------------------------------

    uint16_t crc = modbusCRC(
        frame,
        6
    );

    frame[6] = crc & 0xFF;

    frame[7] = (crc >> 8) & 0xFF;


    // -------------------------------------------------
    // CLEAR OLD RX DATA
    // -------------------------------------------------

    while (maxsensor.available())
    {
        maxsensor.read();
    }


    // -------------------------------------------------
    // TRANSMIT MODE
    // -------------------------------------------------

    digitalWrite(
        RS485_DE,
        HIGH
    );

    digitalWrite(
        RS485_RE,
        HIGH
    );

    delay(2);


    // -------------------------------------------------
    // SEND REQUEST
    // -------------------------------------------------

    maxsensor.write(
        frame,
        8
    );

    maxsensor.flush();

    delay(2);


    // -------------------------------------------------
    // RECEIVE MODE
    // -------------------------------------------------

    digitalWrite(
        RS485_DE,
        LOW
    );

    digitalWrite(
        RS485_RE,
        LOW
    );


    // -------------------------------------------------
    // RECEIVE RESPONSE
    // -------------------------------------------------

    uint8_t response[20];

    int count = 0;

    unsigned long startTime = millis();


    while (
        (millis() - startTime) < 1000
    )
    {
        while (maxsensor.available())
        {
            if (count < 20)
            {
                response[count] =
                    maxsensor.read();

                count++;
            }
            else
            {
                maxsensor.read();
            }
        }


        // Expected response:
        //
        // Slave ID   = 1
        // Function   = 1
        // Byte count = 1
        // Data       = 4
        // CRC        = 2
        //
        // Total = 9 bytes

        if (count >= 9)
        {
            break;
        }
    }


    // -------------------------------------------------
    // DEBUG RX
    // -------------------------------------------------

    Serial.print("RX (");

    Serial.print(count);

    Serial.print(" bytes): ");


    for (int i = 0; i < count; i++)
    {
        if (response[i] < 0x10)
        {
            Serial.print("0");
        }

        Serial.print(
            response[i],
            HEX
        );

        Serial.print(" ");
    }

    Serial.println();


    // -------------------------------------------------
    // RESPONSE LENGTH
    // -------------------------------------------------

    if (count < 9)
    {
        Serial.println(
            "Modbus ERROR: Response timeout"
        );

        return false;
    }


    // -------------------------------------------------
    // SLAVE ID
    // -------------------------------------------------

    if (response[0] != SLAVE_ID)
    {
        Serial.println(
            "Modbus ERROR: Wrong slave ID"
        );

        return false;
    }


    // -------------------------------------------------
    // FUNCTION CODE
    // -------------------------------------------------

    if (response[1] != 0x03)
    {
        Serial.print(
            "Modbus ERROR: Function = "
        );

        Serial.println(
            response[1],
            HEX
        );

        return false;
    }


    // -------------------------------------------------
    // BYTE COUNT
    // -------------------------------------------------

    if (response[2] != 0x04)
    {
        Serial.print(
            "Modbus ERROR: Byte count = "
        );

        Serial.println(
            response[2]
        );

        return false;
    }


    // -------------------------------------------------
    // CONVERT TO FLOAT
    // -------------------------------------------------

    uint32_t raw =
        ((uint32_t)response[3] << 24) |
        ((uint32_t)response[4] << 16) |
        ((uint32_t)response[5] << 8) |
        ((uint32_t)response[6]);


    memcpy(
        &value,
        &raw,
        sizeof(value)
    );


    return true;
}


// =====================================================
// READ 40001-STYLE REGISTER
//
// Example:
//
// 43024 -> address 3023
//
// address = register - 40001
// =====================================================

bool readFloat40001(
    uint16_t reg,
    float &value
)
{
    if (reg < 40001)
    {
        return false;
    }


    uint16_t address =
        reg - 40001;


    return readFloatAddress(
        address,
        value
    );
}


// =====================================================
// READ ALL PARAMETERS
// AND PUBLISH MQTT
// =====================================================

void load_data()
{
    // -------------------------------------------------
    // VARIABLES
    // -------------------------------------------------

    float pfAvg = 0.0;

    float vllAvg = 0.0;

    float vlnAvg = 0.0;

    float energy = 0.0;

    float current = 0.0;

    float rPhasePower = 0.0;

    float yPhasePower = 0.0;

    float bPhasePower = 0.0;

    float frequency = 0.0;


    // -------------------------------------------------
    // STATUS FLAGS
    // -------------------------------------------------

    bool pfOK = false;

    bool vllOK = false;

    bool vlnOK = false;

    bool energyOK = false;

    bool currentOK = false;

    bool rPowerOK = false;

    bool yPowerOK = false;

    bool bPowerOK = false;

    bool freqOK = false;


    // =================================================
    // HEADER
    // =================================================

    Serial.println();

    Serial.println(
        "========================================"
    );

    Serial.println(
        "       EM6400 NG+ PARAMETER DATA"
    );

    Serial.println(
        "========================================"
    );


    // =================================================
    // POWER FACTOR
    // Register = 43024
    // =================================================

    pfOK = readFloat40001(
        43024,
        pfAvg
    );

    Serial.print("PF Avg        = ");

    if (pfOK)
    {
        Serial.println(
            pfAvg,
            3
        );
    }
    else
    {
        Serial.println("FAILED");
    }

    delay(100);


    // =================================================
    // VLL AVG
    // Register = 43026
    // =================================================

    vllOK = readFloat40001(
        43026,
        vllAvg
    );

    Serial.print("VLL Avg       = ");

    if (vllOK)
    {
        Serial.println(
            vllAvg,
            3
        );
    }
    else
    {
        Serial.println("FAILED");
    }

    delay(100);


    // =================================================
    // VLN AVG
    // Register = 43028
    // =================================================

    vlnOK = readFloat40001(
        43028,
        vlnAvg
    );

    Serial.print("VLN Avg       = ");

    if (vlnOK)
    {
        Serial.println(
            vlnAvg,
            3
        );
    }
    else
    {
        Serial.println("FAILED");
    }

    delay(100);


    // =================================================
    // ENERGY
    // Direct register = 3024
    // =================================================

    energyOK = readFloatAddress(
        3024,
        energy
    );

    Serial.print("Energy        = ");

    if (energyOK)
    {
        Serial.println(
            energy,
            3
        );
    }
    else
    {
        Serial.println("FAILED");
    }

    delay(100);


    // =================================================
    // CURRENT
    // Direct register = 3026
    // =================================================

    currentOK = readFloatAddress(
        3026,
        current
    );

    Serial.print("Current       = ");

    if (currentOK)
    {
        Serial.println(
            current,
            3
        );
    }
    else
    {
        Serial.println("FAILED");
    }

    delay(100);


    // =================================================
    // R PHASE POWER
    // Register = 43025
    // =================================================

    rPowerOK = readFloat40001(
        43025,
        rPhasePower
    );

    Serial.print("R Phase Power = ");

    if (rPowerOK)
    {
        Serial.println(
            rPhasePower,
            3
        );
    }
    else
    {
        Serial.println("FAILED");
    }

    delay(100);


    // =================================================
    // Y PHASE POWER
    // Direct register = 3025
    // =================================================

    yPowerOK = readFloatAddress(
        3025,
        yPhasePower
    );

    Serial.print("Y Phase Power = ");

    if (yPowerOK)
    {
        Serial.println(
            yPhasePower,
            3
        );
    }
    else
    {
        Serial.println("FAILED");
    }

    delay(100);


    // =================================================
    // B PHASE POWER
    // Register = 43947
    // =================================================

    bPowerOK = readFloat40001(
        43947,
        bPhasePower
    );

    Serial.print("B Phase Power = ");

    if (bPowerOK)
    {
        Serial.println(
            bPhasePower,
            3
        );
    }
    else
    {
        Serial.println("FAILED");
    }

    delay(100);


    // =================================================
    // FREQUENCY
    // Register = 43915
    // =================================================

    freqOK = readFloat40001(
        43915,
        frequency
    );

    Serial.print("Frequency     = ");

    if (freqOK)
    {
        Serial.println(
            frequency,
            3
        );
    }
    else
    {
        Serial.println("FAILED");
    }


    // =================================================
    // FOOTER
    // =================================================

    Serial.println(
        "========================================"
    );


    // =================================================
    // CHECK MQTT
    // =================================================

    if (!mqttClient.connected())
    {
        Serial.println(
            "MQTT disconnected. Reconnecting..."
        );

        connectMQTT();
    }


    if (!mqttClient.connected())
    {
        Serial.println(
            "MQTT unavailable. Data not published."
        );

        return;
    }


    mqttClient.loop();


    // =================================================
    // CREATE JSON
    // =================================================

    JsonDocument doc;


    // -------------------------------------------------
    // DEVICE INFORMATION
    // -------------------------------------------------

    doc["device"] =
        "ARIES IoT v2.0";

    doc["meter"] =
        "EM6400 NG+";

    doc["slave_id"] =
        SLAVE_ID;


    // -------------------------------------------------
    // ELECTRICAL VALUES
    // -------------------------------------------------

    doc["pf_avg"] =
        pfOK ? pfAvg : 0.0;

    doc["vll_avg"] =
        vllOK ? vllAvg : 0.0;

    doc["vln_avg"] =
        vlnOK ? vlnAvg : 0.0;

    doc["energy"] =
        energyOK ? energy : 0.0;

    doc["current"] =
        currentOK ? current : 0.0;

    doc["r_phase_power"] =
        rPowerOK ? rPhasePower : 0.0;

    doc["y_phase_power"] =
        yPowerOK ? yPhasePower : 0.0;

    doc["b_phase_power"] =
        bPowerOK ? bPhasePower : 0.0;

    doc["frequency"] =
        freqOK ? frequency : 0.0;


    // -------------------------------------------------
    // READ STATUS
    // -------------------------------------------------

    doc["pf_ok"] =
        pfOK;

    doc["vll_ok"] =
        vllOK;

    doc["vln_ok"] =
        vlnOK;

    doc["energy_ok"] =
        energyOK;

    doc["current_ok"] =
        currentOK;

    doc["r_power_ok"] =
        rPowerOK;

    doc["y_power_ok"] =
        yPowerOK;

    doc["b_power_ok"] =
        bPowerOK;

    doc["frequency_ok"] =
        freqOK;


    // =================================================
    // SERIALIZE JSON
    // =================================================

    char payload[768];

    serializeJson(
        doc,
        payload,
        sizeof(payload)
    );


    // =================================================
    // PRINT MQTT PAYLOAD
    // =================================================

    Serial.println();

    Serial.println(
        "MQTT Payload:"
    );

    Serial.println(
        payload
    );


    // =================================================
    // PUBLISH MQTT
    // =================================================

    size_t payloadSize = strlen(payload);

    Serial.print("Payload size: ");
    Serial.print(payloadSize);
    Serial.println(" bytes");

    bool ok = mqttClient.publish(MQTT_TOPIC, payload);

    Serial.print("MQTT State: ");
    Serial.println(mqttClient.state());

    if (ok)
    {
        Serial.println("MQTT Publish: SUCCESS");
    }
    else
    {
        Serial.println("MQTT Publish: FAILED");
    }

    Serial.println();
}

#endif