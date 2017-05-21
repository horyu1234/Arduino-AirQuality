/**
 * Created by horyu1234 on 2017-05-20.
 * 제작: horyu1234
 * 
 * 참고한 자료
 * MQ135: https://github.com/ViliusKraujutis/MQ135/tree/fb1bed6fff7418777930cdf8173bb6fb5c7e70a9
 * GP2Y1014AU0F: https://github.com/Trefex/arduino-airquality/blob/master/Module_Dust-Sensor/dustSensor/dustSensor.ino
 * DHT22: https://github.com/adafruit/DHT-sensor-library
 */

#include "MQ135.h"
#include "DHT.h"

#define MQ135_DATA_PIN A5 // Analog
#define GP2Y1014AU0F_DATA_PIN 0 // Analog
#define GP2Y1014AU0F_LED_PIN 2 // Digital
#define DHT22_DATA_PIN 7 // Digital

#define GP2Y1014AU0F_SAMPLING_TIME 280
#define GP2Y1014AU0F_DELTA_TIME 40
#define GP2Y1014AU0F_SLEEP_TIME 9680

#define SERIAL_DATA_SPLIT_CHAR "-"

#define LOOP_DELAY 1000

DHT dht;
MQ135 mq135 = MQ135(MQ135_DATA_PIN);

float correctedPPM;
float calcVoltage;
float dustDensity;
float humidity;
float temperature;

void setup() {
  Serial.begin(9600);

  initPins();
  initSensors();
}

void initPins() {
  pinMode(GP2Y1014AU0F_LED_PIN, OUTPUT);
}

void initSensors() {
  dht.setup(DHT22_DATA_PIN);
}

void loop() {
  getGP2Y1014AU0FData();
  getDHT22Data();
  getMQ135Data();

  sendDataToSerial();

  delay(LOOP_DELAY);
}

void sendDataToSerial() {
  Serial.print(calcVoltage);
  Serial.print(SERIAL_DATA_SPLIT_CHAR);
  Serial.print(dustDensity);
  Serial.print(SERIAL_DATA_SPLIT_CHAR);
  Serial.print(humidity, 1);
  Serial.print(SERIAL_DATA_SPLIT_CHAR);
  Serial.print(temperature, 1);
  Serial.print(SERIAL_DATA_SPLIT_CHAR);
  Serial.println(correctedPPM);
}

void getMQ135Data() {
  float rzero = mq135.getRZero();
  float correctedRZero = mq135.getCorrectedRZero(temperature, humidity);
  float resistance = mq135.getResistance();
  float ppm = mq135.getPPM();
  correctedPPM = mq135.getCorrectedPPM(temperature, humidity);
}

void getGP2Y1014AU0FData() {
  digitalWrite(GP2Y1014AU0F_LED_PIN, LOW); // power on the LED
  delayMicroseconds(GP2Y1014AU0F_SAMPLING_TIME);

  float voMeasured = analogRead(GP2Y1014AU0F_DATA_PIN); // read the dust value

  delayMicroseconds(GP2Y1014AU0F_DELTA_TIME);
  digitalWrite(GP2Y1014AU0F_LED_PIN, HIGH); // turn the LED off
  delayMicroseconds(GP2Y1014AU0F_SLEEP_TIME);

  // 0 - 5.0V mapped to 0 - 1023 integer values
  calcVoltage = voMeasured * (5.0 / 1024);

  // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // Chris Nafis (c) 2012
  dustDensity = (0.17 * calcVoltage - 0.1) * 1000;
  if (dustDensity < 0) {
    dustDensity = 0;
  }
}

void getDHT22Data() {
  humidity = dht.getHumidity();
  temperature = dht.getTemperature();
}
