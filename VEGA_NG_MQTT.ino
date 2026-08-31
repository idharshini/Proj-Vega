#include "Get_data.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("      ARIES IoT v2.0");
    Serial.println("      EM6400 NG+");
    Serial.println("================================");

    // Initialize Modbus
    init_modbus();

    // Connect WiFi
    connectToWIFI();

    // Initialize MQTT
    initMQTT();

    // Connect MQTT
    connectMQTT();
}

void loop()
{
    // -----------------------------
    // WiFi check
    // -----------------------------
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi disconnected!");
        connectToWIFI();
    }

    // -----------------------------
    // MQTT check
    // -----------------------------
    if (!mqttClient.connected())
    {
        Serial.println("MQTT disconnected!");
        connectMQTT();
    }

    // Keep MQTT connection alive
    mqttClient.loop();

    // -----------------------------
    // Read meter + publish
    // -----------------------------
    load_data();

    delay(5000);
}