#include <SPI.h>
#define SPI SPI1

#include <Ethernet_Generic.h>
#include <HardwareSerial.h>
#include <string.h>
#include <math.h>

// ============================================================
//                 VEGA ARIES V2
//                 EM6400NG+
//                 RS485 + W5500
//                 MQTT + CSV
// ============================================================

// ============================================================
//                    RS485
// ============================================================

#define RS485_DE       8
#define RS485_RE       7

#define SLAVE_ID       52
#define MODBUS_BAUD    9600

HardwareSerial meter(1);

// ============================================================
//                    W5500
// ============================================================

#define W5500_CS       11

byte mac[] =
{
  0xDE,
  0xAD,
  0xBE,
  0xEF,
  0xFE,
  0x01
};

// ============================================================
//                    NETWORK
// ============================================================

IPAddress ip(
  192, 168, 137, 177
);

IPAddress dnsServer(
  8, 8, 8, 8
);

IPAddress gateway(
  192, 168, 137, 1
);

IPAddress subnet(
  255, 255, 255, 0
);

EthernetClient mqttClient;

// ============================================================
//                    MQTT
// ============================================================

const char* MQTT_HOST =
  "broker.hivemq.com";

const uint16_t MQTT_PORT =
  1883;

const char* MQTT_TOPIC =
  "aries/em6400/data";

// ============================================================
//                    TIMING
// ============================================================

const unsigned long METER_INTERVAL = 5000;

const unsigned long MQTT_RETRY_INTERVAL = 3000;

unsigned long lastMeterRead = 0;

unsigned long lastMQTTAttempt = 0;

// ============================================================
//                    METER VALUES
// ============================================================

float pf = 0.0;

float vll = 0.0;

float current = 0.0;

float currentA = 0.0;
float currentB = 0.0;
float currentC = 0.0;

float energy = 0.0;

float frequency = 0.0;

// ============================================================
//                    MODBUS CRC
// ============================================================

uint16_t modbusCRC(
  uint8_t *buffer,
  uint8_t length
)
{
  uint16_t crc = 0xFFFF;

  for (
    uint8_t pos = 0;
    pos < length;
    pos++
  )
  {
    crc ^= buffer[pos];

    for (
      uint8_t i = 0;
      i < 8;
      i++
    )
    {
      if (
        crc & 0x0001
      )
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

  delay(2);
}

// ============================================================
//                    INITIALIZE RS485
// ============================================================

void initRS485()
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
    MODBUS_BAUD
  );

  Serial.println();
  Serial.println(
    "RS485 READY"
  );

  Serial.print(
    "Baud      : "
  );

  Serial.println(
    MODBUS_BAUD
  );

  Serial.print(
    "Slave ID  : "
  );

  Serial.println(
    SLAVE_ID
  );
}

// ============================================================
//                    CLEAR RX
// ============================================================

void clearRS485()
{
  while (
    meter.available()
  )
  {
    meter.read();
  }
}

// ============================================================
//                    FLOAT CONVERSION
// ============================================================

float bytesToFloat(
  uint8_t b0,
  uint8_t b1,
  uint8_t b2,
  uint8_t b3
)
{
  uint32_t raw =
    ((uint32_t)b0 << 24) |
    ((uint32_t)b1 << 16) |
    ((uint32_t)b2 << 8) |
    b3;

  float value;

  memcpy(
    &value,
    &raw,
    sizeof(value)
  );

  return value;
}

// ============================================================
//                    PRINT MODBUS
// ============================================================

void printModbus(
  uint8_t* response,
  uint16_t count
)
{
  Serial.print(
    "RX ["
  );

  Serial.print(
    count
  );

  Serial.print(
    " bytes]: "
  );

  for (
    uint16_t i = 0;
    i < count;
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
}

// ============================================================
//          READ FLOAT32 - FUNCTION 03
// ============================================================

bool readFloatRegister(
  uint16_t address,
  float &value
)
{
  uint8_t request[8];

  request[0] =
    SLAVE_ID;

  request[1] =
    0x03;

  request[2] =
    highByte(address);

  request[3] =
    lowByte(address);

  request[4] =
    0x00;

  request[5] =
    0x02;

  uint16_t crc =
    modbusCRC(
      request,
      6
    );

  request[6] =
    lowByte(crc);

  request[7] =
    highByte(crc);

  clearRS485();

  // ----------------------------------------------------------
  // TRANSMIT
  // ----------------------------------------------------------

  rs485Transmit();

  meter.write(
    request,
    8
  );

  meter.flush();

  delay(3);

  rs485Receive();

  // ----------------------------------------------------------
  // RECEIVE
  // ----------------------------------------------------------

  uint8_t response[32];

  uint16_t count = 0;

  unsigned long start =
    millis();

  while (
    millis() - start < 1000
  )
  {
    while (
      meter.available()
    )
    {
      if (
        count <
        sizeof(response)
      )
      {
        response[count++] =
          meter.read();
      }
    }

    if (
      count >= 5 &&
      response[0] == SLAVE_ID &&
      response[1] == 0x83
    )
    {
      break;
    }

    if (
      count >= 9
    )
    {
      break;
    }

    delay(1);
  }

  if (
    count == 0
  )
  {
    Serial.print(
      "TIMEOUT @ "
    );

    Serial.println(
      address
    );

    return false;
  }

  printModbus(
    response,
    count
  );

  // ----------------------------------------------------------
  // EXCEPTION
  // ----------------------------------------------------------

  if (
    count >= 5 &&
    response[0] == SLAVE_ID &&
    response[1] == 0x83
  )
  {
    Serial.print(
      "MODBUS EXCEPTION @ "
    );

    Serial.print(
      address
    );

    Serial.print(
      " CODE=0x"
    );

    Serial.println(
      response[2],
      HEX
    );

    return false;
  }

  // ----------------------------------------------------------
  // LENGTH
  // ----------------------------------------------------------

  if (
    count < 9
  )
  {
    Serial.println(
      "SHORT RESPONSE"
    );

    return false;
  }

  // ----------------------------------------------------------
  // HEADER
  // ----------------------------------------------------------

  if (
    response[0] != SLAVE_ID ||
    response[1] != 0x03 ||
    response[2] != 0x04
  )
  {
    Serial.println(
      "HEADER ERROR"
    );

    return false;
  }

  // ----------------------------------------------------------
  // CRC
  // ----------------------------------------------------------

  uint16_t receivedCRC =
    response[7] |
    (
      (uint16_t)
      response[8]
      << 8
    );

  uint16_t calculatedCRC =
    modbusCRC(
      response,
      7
    );

  if (
    receivedCRC !=
    calculatedCRC
  )
  {
    Serial.print(
      "CRC ERROR @ "
    );

    Serial.println(
      address
    );

    return false;
  }

  // ----------------------------------------------------------
  // FLOAT
  // ----------------------------------------------------------

  value =
    bytesToFloat(
      response[3],
      response[4],
      response[5],
      response[6]
    );

  if (
    isnan(value) ||
    isinf(value)
  )
  {
    Serial.print(
      "INVALID FLOAT @ "
    );

    Serial.println(
      address
    );

    return false;
  }

  return true;
}

// ============================================================
//             READ 64-BIT ENERGY
// ============================================================

bool readEnergy64(
  uint16_t address,
  float &value
)
{
  uint8_t request[8];

  request[0] =
    SLAVE_ID;

  request[1] =
    0x03;

  request[2] =
    highByte(address);

  request[3] =
    lowByte(address);

  request[4] =
    0x00;

  request[5] =
    0x04;

  uint16_t crc =
    modbusCRC(
      request,
      6
    );

  request[6] =
    lowByte(crc);

  request[7] =
    highByte(crc);

  clearRS485();

  rs485Transmit();

  meter.write(
    request,
    8
  );

  meter.flush();

  delay(3);

  rs485Receive();

  uint8_t response[32];

  uint16_t count = 0;

  unsigned long start =
    millis();

  while (
    millis() - start < 1200
  )
  {
    while (
      meter.available()
    )
    {
      if (
        count <
        sizeof(response)
      )
      {
        response[count++] =
          meter.read();
      }
    }

    if (
      count >= 5 &&
      response[0] == SLAVE_ID &&
      response[1] == 0x83
    )
    {
      break;
    }

    if (
      count >= 13
    )
    {
      break;
    }

    delay(1);
  }

  if (
    count == 0
  )
  {
    Serial.print(
      "ENERGY TIMEOUT @ "
    );

    Serial.println(
      address
    );

    return false;
  }

  printModbus(
    response,
    count
  );

  // Exception
  if (
    count >= 5 &&
    response[0] == SLAVE_ID &&
    response[1] == 0x83
  )
  {
    Serial.print(
      "ENERGY EXCEPTION 0x"
    );

    Serial.println(
      response[2],
      HEX
    );

    return false;
  }

  if (
    count < 13
  )
  {
    Serial.println(
      "ENERGY SHORT RESPONSE"
    );

    return false;
  }

  if (
    response[0] != SLAVE_ID ||
    response[1] != 0x03 ||
    response[2] != 0x08
  )
  {
    Serial.println(
      "ENERGY HEADER ERROR"
    );

    return false;
  }

  uint16_t receivedCRC =
    response[11] |
    (
      (uint16_t)
      response[12]
      << 8
    );

  uint16_t calculatedCRC =
    modbusCRC(
      response,
      11
    );

  if (
    receivedCRC !=
    calculatedCRC
  )
  {
    Serial.println(
      "ENERGY CRC ERROR"
    );

    return false;
  }

  uint64_t raw = 0;

  for (
    uint8_t i = 0;
    i < 8;
    i++
  )
  {
    raw =
      (raw << 8) |
      response[3 + i];
  }

  // Wh -> kWh
  value =
    (float)
    (
      (double)raw /
      1000.0
    );

  return true;
}

// ============================================================
//                 READ EM6400NG+
// ============================================================

bool readMeter()
{
  bool success = true;

  float temp = 0.0;

  Serial.println();
  Serial.println(
    "========================================"
  );

  Serial.println(
    "           EM6400NG+ READ"
  );

  Serial.println(
    "========================================"
  );

  // ==========================================================
  // CURRENT A
  // ==========================================================

  Serial.println();
  Serial.println(
    "Current A - 2999"
  );

  if (
    readFloatRegister(
      2999,
      currentA
    )
  )
  {
    Serial.print(
      "Current A = "
    );

    Serial.print(
      currentA,
      3
    );

    Serial.println(
      " A"
    );
  }
  else
  {
    Serial.println(
      "Current A READ FAILED"
    );
  }

  // ==========================================================
  // CURRENT B
  // ==========================================================

  Serial.println();
  Serial.println(
    "Current B - 3001"
  );

  if (
    readFloatRegister(
      3001,
      currentB
    )
  )
  {
    Serial.print(
      "Current B = "
    );

    Serial.print(
      currentB,
      3
    );

    Serial.println(
      " A"
    );
  }
  else
  {
    Serial.println(
      "Current B READ FAILED"
    );
  }

  // ==========================================================
  // CURRENT C
  // ==========================================================

  Serial.println();
  Serial.println(
    "Current C - 3003"
  );

  if (
    readFloatRegister(
      3003,
      currentC
    )
  )
  {
    Serial.print(
      "Current C = "
    );

    Serial.print(
      currentC,
      3
    );

    Serial.println(
      " A"
    );
  }
  else
  {
    Serial.println(
      "Current C READ FAILED"
    );
  }

  // ==========================================================
  // CURRENT AVERAGE
  // ==========================================================

  Serial.println();
  Serial.println(
    "Current AVG - 3009"
  );

  if (
    readFloatRegister(
      3009,
      current
    )
  )
  {
    Serial.print(
      "Current AVG = "
    );

    Serial.print(
      current,
      3
    );

    Serial.println(
      " A"
    );
  }
  else
  {
    Serial.println(
      "Current AVG READ FAILED"
    );

    success = false;
  }

  // ==========================================================
  // VLL
  // ==========================================================

  Serial.println();
  Serial.println(
    "VLL AVG - 3025"
  );

  if (
    readFloatRegister(
      3025,
      vll
    )
  )
  {
    Serial.print(
      "VLL = "
    );

    Serial.print(
      vll,
      2
    );

    Serial.println(
      " V"
    );
  }
  else
  {
    Serial.println(
      "VLL READ FAILED"
    );

    success = false;
  }

  // ==========================================================
  // PF
  // ==========================================================

  Serial.println();
  Serial.println(
    "PF TOTAL - 3083"
  );

  if (
    readFloatRegister(
      3083,
      pf
    )
  )
  {
    Serial.print(
      "PF = "
    );

    Serial.println(
      pf,
      3
    );

    if (
      isnan(pf) ||
      isinf(pf) ||
      pf < -1.1 ||
      pf > 1.1
    )
    {
      Serial.println(
        "PF INVALID - TRYING 3191"
      );

      if (
        readFloatRegister(
          3191,
          pf
        )
      )
      {
        Serial.print(
          "PF 3191 = "
        );

        Serial.println(
          pf,
          3
        );
      }
      else
      {
        success = false;
      }
    }
  }
  else
  {
    Serial.println(
      "PF 3083 READ FAILED"
    );

    Serial.println(
      "Trying PF 3191..."
    );

    if (
      !readFloatRegister(
        3191,
        pf
      )
    )
    {
      success = false;
    }
  }

  // ==========================================================
  // FREQUENCY
  // ==========================================================

  Serial.println();
  Serial.println(
    "FREQUENCY - 3109"
  );

  if (
    readFloatRegister(
      3109,
      frequency
    )
  )
  {
    Serial.print(
      "Frequency = "
    );

    Serial.print(
      frequency,
      2
    );

    Serial.println(
      " Hz"
    );
  }
  else
  {
    Serial.println(
      "FREQUENCY READ FAILED"
    );

    success = false;
  }

  // ==========================================================
  // ENERGY
  // ==========================================================

  Serial.println();
  Serial.println(
    "ENERGY - 3203"
  );

  if (
    readEnergy64(
      3203,
      energy
    )
  )
  {
    Serial.print(
      "Energy = "
    );

    Serial.print(
      energy,
      3
    );

    Serial.println(
      " kWh"
    );
  }
  else
  {
    Serial.println(
      "ENERGY READ FAILED"
    );

    success = false;
  }

  // ==========================================================
  // SUMMARY
  // ==========================================================

  Serial.println();
  Serial.println(
    "----------------------------------------"
  );

  Serial.print(
    "PF        = "
  );

  Serial.println(
    pf,
    3
  );

  Serial.print(
    "VLL       = "
  );

  Serial.print(
    vll,
    2
  );

  Serial.println(
    " V"
  );

  Serial.print(
    "Current   = "
  );

  Serial.print(
    current,
    3
  );

  Serial.println(
    " A"
  );

  Serial.print(
    "Current A = "
  );

  Serial.print(
    currentA,
    3
  );

  Serial.println(
    " A"
  );

  Serial.print(
    "Current B = "
  );

  Serial.print(
    currentB,
    3
  );

  Serial.println(
    " A"
  );

  Serial.print(
    "Current C = "
  );

  Serial.print(
    currentC,
    3
  );

  Serial.println(
    " A"
  );

  Serial.print(
    "Energy    = "
  );

  Serial.print(
    energy,
    3
  );

  Serial.println(
    " kWh"
  );

  Serial.print(
    "Frequency = "
  );

  Serial.print(
    frequency,
    2
  );

  Serial.println(
    " Hz"
  );

  Serial.println(
    "----------------------------------------"
  );

  if (success)
  {
    Serial.println(
      "EM6400 READ SUCCESS"
    );
  }
  else
  {
    Serial.println(
      "EM6400 READ FAILED"
    );
  }

  Serial.println(
    "========================================"
  );

  return success;
}

// ============================================================
//                    CREATE CSV
// ============================================================

// ============================================================
//              FLOAT -> CSV TEXT WITHOUT dtostrf()
// ============================================================
// The VEGA ARIES V2 String(float, decimals) implementation itself
// calls dtostrf(), so String(float, ...) cannot be used here.
// This formatter uses only integer/String operations.

void appendFloatCSV(
  String &csv,
  float value,
  uint8_t decimals
)
{
  if (isnan(value) || isinf(value))
  {
    csv += "0";
    return;
  }

  bool negative = (value < 0.0f);

  if (negative)
  {
    value = -value;
    csv += "-";
  }

  unsigned long scale = 1;

  for (uint8_t i = 0; i < decimals; i++)
  {
    scale *= 10;
  }

  // Round before converting to an integer.
  unsigned long scaled =
    (unsigned long)(value * (float)scale + 0.5f);

  unsigned long whole = scaled / scale;
  unsigned long fraction = scaled % scale;

  csv += String(whole);

  if (decimals > 0)
  {
    csv += ".";

    unsigned long divider = scale / 10;

    while (divider > 0)
    {
      if (fraction < divider)
      {
        csv += "0";
      }
      else
      {
        break;
      }

      divider /= 10;
    }

    csv += String(fraction);
  }
}

void createCSV(
  char *payload,
  size_t size
)
{
  String csv = "";

  appendFloatCSV(csv, pf,        3);
  csv += ",";

  appendFloatCSV(csv, vll,       2);
  csv += ",";

  appendFloatCSV(csv, current,   3);
  csv += ",";

  appendFloatCSV(csv, energy,    3);
  csv += ",";

  appendFloatCSV(csv, frequency, 2);

  csv.toCharArray(payload, size);
}

// ============================================================
//                    ETHERNET CHECK
// ============================================================

bool ethernetReady()
{
  if (
    Ethernet.hardwareStatus()
    != EthernetW5500
  )
  {
    Serial.println(
      "W5500 NOT DETECTED"
    );

    return false;
  }

  if (
    Ethernet.linkStatus()
    != LinkON
  )
  {
    Serial.println(
      "ETHERNET LINK DOWN"
    );

    return false;
  }

  return true;
}


// ============================================================
//                    MQTT CONNECT
//                    NON-BLOCKING RETRY
// ============================================================

bool mqttConnect()
{
  if (mqttClient.connected())
  {
    return true;
  }

  if (millis() - lastMQTTAttempt < MQTT_RETRY_INTERVAL)
  {
    return false;
  }

  lastMQTTAttempt = millis();

  if (!ethernetReady())
  {
    Serial.println("ETHERNET NOT READY");
    return false;
  }

  Serial.println();
  Serial.println("MQTT CONNECTING...");

  mqttClient.stop();
  delay(20);

  // ----------------------------------------------------------
  // TCP
  // ----------------------------------------------------------

  Serial.print("TCP CONNECTION... ");

  if (!mqttClient.connect(MQTT_HOST, MQTT_PORT))
  {
    Serial.println("FAILED");
    mqttClient.stop();
    return false;
  }

  Serial.println("SUCCESS");

  // ----------------------------------------------------------
  // CLIENT ID
  // ----------------------------------------------------------

  String clientID = "ARIES-";
  clientID += String(random(100000, 999999));

  // ----------------------------------------------------------
  // MQTT CONNECT PACKET
  // ----------------------------------------------------------

  uint8_t packet[128];
  uint16_t p = 0;

  packet[p++] = 0x10;

  uint16_t remaining =
    12 + clientID.length();

  packet[p++] = (uint8_t)remaining;

  // Protocol name: MQTT
  packet[p++] = 0x00;
  packet[p++] = 0x04;
  packet[p++] = 'M';
  packet[p++] = 'Q';
  packet[p++] = 'T';
  packet[p++] = 'T';

  // MQTT 3.1.1
  packet[p++] = 0x04;

  // Clean session
  packet[p++] = 0x02;

  // Keep alive = 60 seconds
  packet[p++] = 0x00;
  packet[p++] = 0x3C;

  // Client ID length
  packet[p++] = highByte(clientID.length());
  packet[p++] = lowByte(clientID.length());

  memcpy(
    &packet[p],
    clientID.c_str(),
    clientID.length()
  );

  p += clientID.length();

  mqttClient.write(packet, p);
  mqttClient.flush();

  // ----------------------------------------------------------
  // WAIT FOR CONNACK
  // ----------------------------------------------------------

  unsigned long start = millis();

  while (millis() - start < 3000)
  {
    if (mqttClient.available() >= 4)
    {
      uint8_t response[4];

      mqttClient.read(response, 4);

      Serial.print("CONNACK : ");

      for (uint8_t i = 0; i < 4; i++)
      {
        if (response[i] < 0x10)
        {
          Serial.print("0");
        }

        Serial.print(response[i], HEX);
        Serial.print(" ");
      }

      Serial.println();

      if (
        response[0] == 0x20 &&
        response[1] == 0x02 &&
        response[2] == 0x00 &&
        response[3] == 0x00
      )
      {
        Serial.println("MQTT CONNECTED");
        return true;
      }

      Serial.println("MQTT REFUSED");
      mqttClient.stop();
      return false;
    }

    delay(2);
  }

  Serial.println("CONNACK TIMEOUT");
  mqttClient.stop();

  return false;
}


// ============================================================
//                    MQTT KEEP ALIVE
// ============================================================

void mqttKeepAlive()
{
  static unsigned long lastPing = 0;

  if (!mqttClient.connected())
  {
    return;
  }

  if (millis() - lastPing < 30000)
  {
    return;
  }

  lastPing = millis();

  uint8_t ping[2];

  ping[0] = 0xC0;
  ping[1] = 0x00;

  mqttClient.write(ping, 2);
  mqttClient.flush();

  Serial.println("MQTT PINGREQ");
}


// ============================================================
//                    MQTT PUBLISH CSV
// ============================================================

bool mqttPublish(
  const char *topic,
  const char *payload
)
{
  if (!mqttClient.connected())
  {
    return false;
  }

  uint16_t topicLength =
    strlen(topic);

  uint16_t payloadLength =
    strlen(payload);

  uint16_t remaining =
    2 +
    topicLength +
    payloadLength;

  if (remaining > 255)
  {
    Serial.println("MQTT PACKET TOO LARGE");
    return false;
  }

  mqttClient.write((uint8_t)0x30);
  mqttClient.write((uint8_t)remaining);

  mqttClient.write(
    (uint8_t)highByte(topicLength)
  );

  mqttClient.write(
    (uint8_t)lowByte(topicLength)
  );

  mqttClient.write(
    (const uint8_t*)topic,
    topicLength
  );

  mqttClient.write(
    (const uint8_t*)payload,
    payloadLength
  );

  mqttClient.flush();

  // QoS 0 does not return an acknowledgement.
  // Only verify that the TCP socket is still connected.
  if (!mqttClient.connected())
  {
    Serial.println("MQTT CONNECTION LOST");
    mqttClient.stop();
    return false;
  }

  return true;
}


// ============================================================
//                    SETUP
// ============================================================

void setup()
{
  Serial.begin(
    115200
  );

  delay(2000);

  randomSeed(
    micros()
  );

  Serial.println();
  Serial.println(
    "========================================"
  );
  Serial.println(
    "          VEGA ARIES V2"
  );
  Serial.println(
    "          EM6400NG+"
  );
  Serial.println(
    "          RS485 + W5500"
  );
  Serial.println(
    "          ETHERNET + MQTT"
  );
  Serial.println(
    "          CSV OUTPUT"
  );
  Serial.println(
    "========================================"
  );

  // ==========================================================
  // RS485
  // ==========================================================

  initRS485();

  // ==========================================================
  // SPI1
  // ==========================================================

  pinMode(
    W5500_CS,
    OUTPUT
  );

  digitalWrite(
    W5500_CS,
    HIGH
  );

  Serial.println();
  Serial.println(
    "Starting SPI1..."
  );

  SPI1.begin();

  Serial.println(
    "SPI1 READY"
  );

  // ==========================================================
  // W5500
  // ==========================================================

  Ethernet.init(
    W5500_CS
  );

  Serial.println(
    "Starting W5500..."
  );

  Ethernet.begin(
    mac,
    ip,
    dnsServer,
    gateway,
    subnet
  );

  delay(1000);

  // ==========================================================
  // ETHERNET INFORMATION
  // ==========================================================

  Serial.println();
  Serial.println(
    "========================================"
  );
  Serial.println(
    "             ETHERNET"
  );
  Serial.println(
    "========================================"
  );

  Serial.print(
    "IP       : "
  );

  Serial.println(
    Ethernet.localIP()
  );

  Serial.print(
    "Gateway  : "
  );

  Serial.println(
    Ethernet.gatewayIP()
  );

  Serial.print(
    "Subnet   : "
  );

  Serial.println(
    Ethernet.subnetMask()
  );

  Serial.print(
    "DNS      : "
  );

  Serial.println(
    Ethernet.dnsServerIP()
  );

  Serial.print(
    "W5500    : "
  );

  if (
    Ethernet.hardwareStatus()
    == EthernetW5500
  )
  {
    Serial.println(
      "DETECTED"
    );
  }
  else
  {
    Serial.println(
      "NOT DETECTED"
    );
  }

  Serial.print(
    "LINK     : "
  );

  if (
    Ethernet.linkStatus()
    == LinkON
  )
  {
    Serial.println(
      "CONNECTED"
    );
  }
  else
  {
    Serial.println(
      "DOWN"
    );
  }

  // ==========================================================
  // TCP TEST
  // ==========================================================

  Serial.println();

  Serial.print(
    "HiveMQ TCP TEST : "
  );

  EthernetClient test;

  if (
    test.connect(
      MQTT_HOST,
      MQTT_PORT
    )
  )
  {
    Serial.println(
      "SUCCESS"
    );

    test.stop();
  }
  else
  {
    Serial.println(
      "FAILED"
    );
  }

  // ==========================================================
  // READY
  // ==========================================================

  Serial.println();
  Serial.println(
    "========================================"
  );

  Serial.println(
    "             SYSTEM READY"
  );

  Serial.println(
    "========================================"
  );

  Serial.println(
    "ETHERNET : ENABLED"
  );

  Serial.println(
    "WIFI     : NOT USED"
  );

  Serial.println(
    "MQTT     : HiveMQ"
  );

  Serial.println(
    "CSV      : ENABLED"
  );

  Serial.println(
    "========================================"
  );
}


// ============================================================
//                    LOOP
// ============================================================

void loop()
{
  // ==========================================================
  // 1. READ THE METER FIRST
  //    MQTT/Ethernet must NEVER stop the meter cycle.
  // ==========================================================
  if (millis() - lastMeterRead >= METER_INTERVAL)
  {
    lastMeterRead = millis();

    Serial.println();
    Serial.println(">>> START METER CYCLE");

    bool meterOK = readMeter();

    Serial.println(">>> AFTER READMETER");

    if (!meterOK)
    {
      Serial.println("METER READ FAILED");
      Serial.println("SYSTEM CONTINUES");
      return;
    }

    // ========================================================
    // 2. CREATE CSV
    // ========================================================
    char payload[128];

    Serial.println(">>> CREATING CSV");

    createCSV(
      payload,
      sizeof(payload)
    );

    Serial.println(">>> CSV CREATED");
    Serial.print("CSV = ");
    Serial.println(payload);

    // ========================================================
    // 3. MQTT CONNECTION / PUBLISH
    //    Only after the meter has been successfully read.
    // ========================================================
    if (!mqttClient.connected())
    {
      Serial.println("MQTT NOT CONNECTED - TRYING RECONNECT...");
      mqttConnect();
    }

    if (mqttClient.connected())
    {
      Serial.println(">>> PUBLISHING CSV");

      if (mqttPublish(MQTT_TOPIC, payload))
      {
        Serial.println(">>> PUBLISH SUCCESS");
        Serial.println(">>> DATA SENT TO HIVEMQ");
      }
      else
      {
        Serial.println(">>> PUBLISH FAILED");
        mqttClient.stop();
      }
    }
    else
    {
      Serial.println(">>> MQTT OFFLINE");
      Serial.println(">>> CSV WILL BE RETRIED ON NEXT CYCLE");
    }

    Serial.println(">>> END CYCLE");
    Serial.println();
  }

  // ==========================================================
  // 4. MQTT KEEP ALIVE
  // ==========================================================
  mqttKeepAlive();

  // Small delay only; loop remains continuously active.
  delay(10);
}
