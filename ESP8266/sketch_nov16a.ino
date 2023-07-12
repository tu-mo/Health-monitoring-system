#include <ESP8266WiFi.h>
#include "FirebaseESP8266.h"
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
SoftwareSerial s(D6, D5);
#define FIREBASE_HOST "test-c8474-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "22aUyTvHaXiF3jc0wmeBTr7eUem8iJ997B0VMVCp"
#define WIFI_SSID "TP-LINK_D4F2"
#define WIFI_PASSWORD "30058682"

FirebaseData firebaseData;
String path = "/";
FirebaseJson json;
unsigned long t1 = 0;

void setup() {
  Serial.begin(9600);
  s.begin(9600);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  if (!Firebase.beginStream(firebaseData, path))
  {
    Serial.println("Reason" + firebaseData.errorReason());
    Serial.println();
  }
}

double data[6];


void loop() {
  int i = 0;
  while (s.available())
  {
    data[i++] = s.parseFloat();

  }
  if (millis() - t1 > 2000)
  {
    t1 = millis();
    for (i = 0; i < 5; i++) {
      Serial.println(data[i]);
    }
    Serial.println("done");
    Firebase.setDouble(firebaseData, path+"/spO2", data[0]);
    Firebase.setDouble(firebaseData, path+"/pulse_rate", data[1]); 
    Firebase.setFloat(firebaseData, path+"/temperature", data[2]); 
    Firebase.setFloat(firebaseData, path+"/latitude", data[3]); 
    Firebase.setDouble(firebaseData, path+"/longtitude", data[4]); 
  }
  
}
