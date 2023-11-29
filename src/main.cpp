#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "PubSubClient.h"
#include <ArduinoJson.h>
#include <ModbusMaster.h>   // https://github.com/4-20ma/ModbusMaster
#include <SoftwareSerial.h> // https://github.com/plerup/espsoftwareserial

char ssid[] = "OborkaNet";
char password[] = "m123456f";
WiFiClient espClient;
PubSubClient client(espClient);
void setup_wifi();
void reconnect();
void callback(char*, byte*, unsigned int);
bool sendEASUN_Status();
void readNode();
int getRealTemp(int);

const char *mqtt_server = "wwa-server.duckdns.org";
const char *topicEASUN = "Obora/esp8266EASUN/Modbus/Status";
const char *topicControl = "Obora/esp8266EASUN/Modbus/Control";
const char *mqtt_username = "esp8266EASUN";
const char *mqtt_password = "m123456f";
const char *willTopic = "Obora/esp8266EASUN/Relay/will";
const char *willmsg = "Ill be back";
const int mqtt_port = 1883;

int status=0,resetCounter=0;
long lastMsg = 0;


//#define RELAY_PIN 12
#define MAX3232_RX 5 // RX pin.
#define MAX3232_TX 4 // TX pin.
/*
    Modbus Constants
    All charge controllers will respond to address 255 no matter what their actual address is, this is useful if you do not know what address to use.
    You can try using 1 here instead of 255 if you don't want to make changes to the library, but it is not guaranteed to work.
*/
#define NUM_REGISTERS 35
#define MODBUS_SLAVE_ADDR 
#define MODBUS_REQUEST_START_ADDR 256255

/*
    Other settings.
*/ 
#define REQUEST_DELAY 3000         // Delay in ms between requests to the charge controller over modbus.
#define JSON_BUFFER_SIZE 2048      // Maximum size for the JSON.

const char *chargeModes[7] = {
    "OFF",      // 0
    "NORMAL",   // 1
    "MPPT",     // 2
    "EQUALIZE", // 3
    "BOOST",    // 4
    "FLOAT",    // 5
    "CUR_LIM"   // 6 (Current limiting)
};

const char *faultCodes[15] = {
    "Charge MOS short circuit",      // (16384 | 01000000 00000000)
    "Anti-reverse MOS short",        // (8192  | 00100000 00000000)
    "PV panel reversely connected",  // (4096  | 00010000 00000000)
    "PV working point over voltage", // (2048  | 00001000 00000000)
    "PV counter current",            // (1024  | 00000100 00000000)
    "PV input side over-voltage",    // (512   | 00000010 00000000)
    "PV input side short circuit",   // (256   | 00000001 00000000)
    "PV input overpower",            // (128   | 00000000 10000000)
    "Ambient temp too high",         // (64    | 00000000 01000000)
    "Controller temp too high",      // (32    | 00000000 00100000)
    "Load over-power/current",       // (16    | 00000000 00010000)
    "Load short circuit",            // (8     | 00000000 00001000)
    "Battery undervoltage warning",  // (4     | 00000000 00000100)
    "Battery overvoltage",           // (2     | 00000000 00000010)
    "Battery over-discharge"         // (1     | 00000000 00000001)
};

// Create the modbus node for the charge controller.
ModbusMaster node;

// Create software serial for communicating with Charge Controller.
SoftwareSerial mySerial(MAX3232_RX, MAX3232_TX);

// Store all the raw data collected from the charge controller.
uint16_t chargeControllerRegisterData[NUM_REGISTERS];

// Was there an error when reading from the charge controller?
uint8_t modbusErr;

char registerDataJson[JSON_BUFFER_SIZE];

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 30000;        // Interval at which to publish sensor readings


void setup() {
  Serial.begin(9600);
  Serial.println();

  mySerial.begin(9600);
  node.begin(MODBUS_SLAVE_ADDR, mySerial);

  //MQTT
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Serial.println("Exit Setup");

}

void loop() {
  long unsigned now = millis();
  if (!client.connected() and millis()<now+5000) {
    reconnect();
    delay(500);
    }
  client.loop();
  now = millis();
  if (now - lastMsg > 10*1000) {
    lastMsg = now;
    readNode();
    if (sendEASUN_Status()) {Serial.println("ESP8266 Status Message published");}
    if (WiFi.status() != WL_CONNECTED) {
      resetCounter++;
      if (resetCounter>20) {
        Serial.print("Rebooting");
        ESP.restart();
      }
    }
  }
}


bool sendEASUN_Status() {
   StaticJsonDocument<JSON_BUFFER_SIZE> doc;
   if (modbusErr) {
        doc["modbusError"] = true;
    } else {
        // We need to account for the load being on when determining the charging mode.
        int loadOffset = chargeControllerRegisterData[32] > 6 ? 32768 : 0;

        doc["modbusError"] = false;
        doc["sensor"] = "EASUN";
        doc["WifiRSSI"]= WiFi.RSSI();

        JsonObject controller = doc.createNestedObject("controller");
        controller["chargingMode"] = chargeModes[chargeControllerRegisterData[32] - loadOffset];
        controller["temperature"] = getRealTemp(chargeControllerRegisterData[3] >> 8);
        controller["days"] = chargeControllerRegisterData[21];
        controller["overDischarges"] = chargeControllerRegisterData[22];
        controller["fullCharges"] = chargeControllerRegisterData[23];

        JsonObject charging = doc.createNestedObject("charging");
        charging["amps"] = chargeControllerRegisterData[2] * 0.01;
        charging["maxAmps"] = chargeControllerRegisterData[13] * 0.01;
        charging["watts"] = chargeControllerRegisterData[9];
        charging["maxWatts"] = chargeControllerRegisterData[15];
        charging["dailyAmpHours"] = chargeControllerRegisterData[17];
        charging["totalAmpHours"] = ((chargeControllerRegisterData[24] * 65536 + chargeControllerRegisterData[25]) * 0.001);
        charging["dailyPower"] = chargeControllerRegisterData[19] * 0.001;
        charging["totalPower"] = ((chargeControllerRegisterData[28] * 65536 + chargeControllerRegisterData[29]) * 0.001);

        JsonObject battery = doc.createNestedObject("battery");
        battery["stateOfCharge"] = chargeControllerRegisterData[0];
        battery["volts"] = chargeControllerRegisterData[1] * 0.1;
        battery["minVolts"] = chargeControllerRegisterData[11] * 0.1;
        battery["maxVolts"] = chargeControllerRegisterData[12] * 0.1;
        battery["temperature"] = getRealTemp(chargeControllerRegisterData[3] & 0xFF);

        JsonObject panels = doc.createNestedObject("panels");
        panels["volts"] = chargeControllerRegisterData[7] * 0.1;
        panels["amps"] = chargeControllerRegisterData[8] * 0.01;

        JsonObject load = doc.createNestedObject("load");
        load["state"] = chargeControllerRegisterData[10] ? true : false;
        load["volts"] = chargeControllerRegisterData[4] * 0.1;
        load["amps"] = chargeControllerRegisterData[5] * 0.01;
        load["watts"] = chargeControllerRegisterData[6];
        load["maxAmps"] = chargeControllerRegisterData[14] * 0.01;
        load["maxWatts"] = chargeControllerRegisterData[16];
        load["dailyAmpHours"] = chargeControllerRegisterData[18];
        load["totalAmpHours"] = ((chargeControllerRegisterData[26] * 65536 + chargeControllerRegisterData[27]) * 0.001);
        load["dailyPower"] = chargeControllerRegisterData[20] * 0.001;
        load["totalPower"] = ((chargeControllerRegisterData[30] * 65536 + chargeControllerRegisterData[31]) * 0.001);

        JsonArray faults = doc.createNestedArray("faults");
        int faultId = chargeControllerRegisterData[34];
        uint8_t count = 0;
        while (faultId != 0) {
            if (faultId >= pow(2, 15 - count)) {
                faults.add(faultCodes[count - 1]);
                faultId -= pow(2, 15 - count);
            }
            count += 1;
        }
    }
    serializeJson(doc, registerDataJson);
    bool publishSuccess=client.publish(topicEASUN, registerDataJson);
    return publishSuccess;
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  StaticJsonDocument<1024> doc;
  deserializeJson(doc, payload, length);
  if (String(topic) == topicControl) {
    
  }
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  unsigned long currentMillis = millis();
  while (WiFi.status() != WL_CONNECTED and millis()<currentMillis+15000) {
      delay(500);
      Serial.print(".");
  }
}

void reconnect() {
  // Loop until we're reconnected
  unsigned long currentMillis = millis();
  while (!client.connected() and millis()<currentMillis+10000) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("esp8266EASUN",mqtt_username,mqtt_password,willTopic,1,false,willmsg,false)) {
      Serial.println("connected");
      // Subscribe
      Serial.print("Subscribing to: ");
      Serial.print(topicControl);
      Serial.print(", Status: ");
      Serial.println(client.subscribe(topicControl));
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}

int getRealTemp(int temp) {
    return temp / 128 ? -(temp % 128) : temp;
}

void readNode() {
    static uint32_t i;
    i++;
    // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
    node.setTransmitBuffer(0, lowWord(i));
    // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
    node.setTransmitBuffer(1, highWord(i));

    uint8_t result = node.readHoldingRegisters(MODBUS_REQUEST_START_ADDR, NUM_REGISTERS);
    if (result == node.ku8MBSuccess) {
        modbusErr = 0;
        Serial.println(F("Successfully read from CC"));

        for (int j = 0; j < NUM_REGISTERS; j++) {
            chargeControllerRegisterData[j] = node.getResponseBuffer(j);
        }
    } else {
        modbusErr = 1;
        Serial.print(F("Failed to read from CC"));
        Serial.print(F(" ("));
        Serial.print(result, HEX);
        Serial.println(F(")"));
    }
}

