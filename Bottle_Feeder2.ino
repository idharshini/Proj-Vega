#include "Get_data.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("========================================");
    Serial.println(" VEGA ARIES IoT V2");
    Serial.println(" Delta DVP28SV Bottle Feeder");
    Serial.println(" Modbus RTU Reader");
    Serial.println("========================================");

    init_modbus();
}

void loop()
{
    load_data();
    delay(1000);
}