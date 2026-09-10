# Proj-Vega — ARIES IoT v2.0 EM6400 NG+ MQTT Bridge

Arduino sketch for the **ARIES IoT v2.0** board that reads an **EM6400 NG+**
energy meter over Modbus RTU (RS485) and publishes the readings as JSON
over MQTT via WiFi.

## Hardware

- ARIES IoT v2.0 board (WiFiNINA-based WiFi)
- MAX485 RS485 transceiver, wired to:
  - `DE` → pin 7
  - `RE` → pin 8
  - Data UART: hardware serial port 1 (`maxsensor`)
- EM6400 NG+ energy meter, Modbus slave ID `52`, 9600 baud 8N1

## Required libraries

Install via the Arduino Library Manager:

- `WiFiNINA`
- `PubSubClient`
- `ArduinoJson`

## Setup

1. Copy `secrets.h.example` to `secrets.h` and fill in your own WiFi SSID/
   password and MQTT broker settings:
   ```
   cp secrets.h.example secrets.h
   ```
   `secrets.h` is gitignored so your credentials are never committed.
2. Open `VEGA_NG_MQTT.ino` in the Arduino IDE and upload it to the board.
3. Open the Serial Monitor at `115200` baud to watch WiFi/MQTT connection
   status and the meter readings being published.

## MQTT payload

Every 5 seconds the sketch publishes a JSON payload like:

```json
{
  "device": "ARIES IoT v2.0",
  "meter": "EM6400 NG+",
  "slave_id": 52,
  "pf_avg": 0.98,
  "vll_avg": 415.2,
  "vln_avg": 239.8,
  "energy": 1234.5,
  "current": 3.2,
  "r_phase_power": 1.1,
  "y_phase_power": 1.0,
  "b_phase_power": 1.2,
  "frequency": 50.0
}
```

to the topic configured as `MQTT_TOPIC` in `secrets.h`.

> **Note:** the default `secrets.h.example` points at the public
> `broker.hivemq.com` broker with no authentication, purely for quick
> testing. Anything published there is publicly readable/writable by
> anyone who knows the topic name — switch to a private or
> authenticated broker before using this with real meter data.

## License

No license file is currently included in this repository — contact the
repository owner if you need clarification on reuse terms.
