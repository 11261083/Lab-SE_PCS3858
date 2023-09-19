#include <WiFi.h>
#include "time.h"
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <LoRa.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TimeLib.h>

// Replace with your network credentials
const char* ssid = "tcclucas";
const char* password = "tcctcctcc";

const char* mqttServer = "mqtt.tago.io";
const int mqttPort = 1883;
const char* mqttUser = "TccTagoIO"; // Replace with your TagoIO token
const char* mqttPassword = "862b7c31-7caf-4420-8dfd-2cca33c6f587"; 

/* GPIO do módulo WiFi LoRa 32(V2) que o pino de comunicação do sensor está ligado. */
#define DHTPIN    13 /* (GPIO 13) */

/* Definicoes para comunicação com radio LoRa */
#define SCK_LORA           5
#define MISO_LORA          19
#define MOSI_LORA          27
#define RESET_PIN_LORA     14
#define SS_PIN_LORA        18

#define HIGH_GAIN_LORA     20  /* dBm */
#define BAND               915E6  /* 915MHz de frequencia */


// NTP server to request epoch time
const char* ntpServer = "pool.ntp.org";
unsigned long epochTime; 
long timezone = -3;
byte daysavetime = 1;

WiFiClient espClient; // Cria o objeto espClient
PubSubClient MQTT(espClient); // Instancia o Cliente MQTT passando o objeto espClient

// Setup unix
void printTime(time_t timeInput, char* output) {
  // Format the time components into the output char array
  sprintf(output, "Year: %d",
          year(timeInput));
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('\n');
    Serial.print(WiFi.status());
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  initWiFi();
  configTime(3600 * timezone, 0, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");

  MQTT.setServer(mqttServer, mqttPort);
  while (!MQTT.connected()) {
    Serial.println("Connecting to MQTT...");
    if (MQTT.connect("TCCTCCTCC", mqttUser, mqttPassword )) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed with state: ");
      Serial.print(MQTT.state());
      delay(2000);
    }
  }
}

void loop() {
  struct tm tmstruct ;

  tmstruct.tm_year = 0;
  getLocalTime(&tmstruct);

  String date = (String((tmstruct.tm_year) + 1900) + "-" + String(( tmstruct.tm_mon) + 1) + "-" + String(tmstruct.tm_sec));
  
  String hour = (String(tmstruct.tm_hour) + ":" + String(tmstruct.tm_min) + ":" + String(tmstruct.tm_sec));
  const char* hourChar = hour.c_str();

  Serial.println("Date: " + date + " - Time: " + hour);

  // Serial.print("Epoch Time: ");
  // Serial.println(epochTime);
  delay(3000);
  MQTT.publish("TccTagoIO/teste", hourChar);
}