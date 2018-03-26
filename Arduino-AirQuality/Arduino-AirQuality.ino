/**
 * Created by horyu1234 on 2017-10-10.
 * 제작: horyu1234
 * 
 * 참고한 자료
 * MQ135: https://github.com/ViliusKraujutis/MQ135/tree/fb1bed6fff7418777930cdf8173bb6fb5c7e70a9
 * DHT22: https://github.com/adafruit/DHT-sensor-library
 */
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "DHT.h"
#include "MQ135.h"

#define DEBUG_NTPClient

#define DHT_DATA_PIN 2 // Digital
#define DHT_TYPE DHT22

#define MQ135_DATA_PIN 0 // Analog

#define SERIAL_DATA_SPLIT_CHAR "-"

#define LOOP_DELAY 5000

const char* ssid     = "<WiFi SSID>";
const char* password = "<WiFi Password>";

const char* host = "<Server Host>";
const char* path = "<Server Path>";
const int port = 9100;

const char* id = "<Sensor Id>";
const char* privateKey = "<Server Private Key>";

const char* timeServer = "time.google.com";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, timeServer, 0, 60000);

DHT dht(DHT_DATA_PIN, DHT_TYPE);
MQ135 mq135 = MQ135(MQ135_DATA_PIN);

float currentTime;
float humidity;
float temperature;
float correctedPPM;

void setup() {
  Serial.begin(115200);

  connectWiFi();

  timeClient.begin();
}

void connectWiFi() {
  WiFi.disconnect();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Try to connect WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  String messagePrefix = "[" + String(timeClient.getEpochTime()) + "] ";
  timeClient.update();

  Serial.println(messagePrefix + "Fetching sensor data...");
  getDHT22Data();
  getAirQualityData();

  Serial.print(messagePrefix + "Sending data to server...");
  sendDataToServer();

  delay(LOOP_DELAY);
}

void sendDataToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("WiFi connection lost, try again...");
    connectWiFi();
    return;
  }

  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("Connection failed!");
    return;
  }

  String dataUrl = path;
  dataUrl += "?time=" + String(timeClient.getEpochTime());
  dataUrl += "&humidity=" + String(humidity);
  dataUrl += "&temperature=" + String(temperature);
  dataUrl += "&airQuality=" + String(correctedPPM);
  dataUrl += "&id=" + String(id);
  dataUrl += "&privateKey=" + String(privateKey);

  // This will send the request to the server
  client.print(String("GET ") + dataUrl + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");

  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

  // Read all the lines of the reply from server and print them to Serial
  while (client.available()) {
    client.readStringUntil('\r');
  }

  Serial.println("success");
}

void getDHT22Data() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
}

void getAirQualityData() {
  float rzero = mq135.getRZero();
  float correctedRZero = mq135.getCorrectedRZero(temperature, humidity);
  float resistance = mq135.getResistance();
  float ppm = mq135.getPPM();
  correctedPPM = mq135.getCorrectedPPM(temperature, humidity);
}
