#include <SoftwareSerial.h>

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

ESP8266WiFiMulti WiFiMulti;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  WiFi.mode(WIFI_STA);
  WiFiMulti.addAP("ganesis", "kriITB2021");
}

void loop() {
  if ((WiFiMulti.run() == WL_CONNECTED)) {
    WiFiClient client;
    HTTPClient http;
    if (http.begin(client, "http://10.5.103.5/punyaku/site2.php")) {
      int httpCode = http.GET();
      if (httpCode > 0) {
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
          String payload2 = http.getString();
          Serial.println(payload2);
          Serial1.println(payload2);
          delay(10);
        }
      }
      http.end();
    }
  }
}
