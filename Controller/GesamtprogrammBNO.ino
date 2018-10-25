//I2C Schnittstelle
#include <Wire.h>

//BNO055 Sensor
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//WSS
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

#include "GesamtprogrammBNO.h"

Adafruit_BNO055 bno = Adafruit_BNO055();

WiFiMulti wifiMulti;
WebSocketsClient webSocket;
const char* wssID = "1";
int currentIP = 0;


const int XAXIS = 0, YAXIS = 1, ZAXIS = 2;
const int MAXSENSORAMOUNT = 10;
int currentSensorSpot = 0;
int  X[MAXSENSORAMOUNT], Y[MAXSENSORAMOUNT], Z[MAXSENSORAMOUNT];
int lastX = 0, lastY = 0, lastZ = 0;
int counter = 0;
int messageCounter = 0;
unsigned long timeStart;

void setup() {
  //  Serial.begin(9600);
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Setup");
  setGyroscope();
  setWSS();
  Serial.println("Setup Done!");

}


void loop() {
  if (counter == 0) {
    timeStart = millis();
  }

  if (millis() - timeStart >= 1000) {
    Serial.println((String)"Cycles per Second: " + counter + "\t\tMessages per Second: " + messageCounter);
    timeStart = millis();
    counter = 0;
    messageCounter = 0;
  } else {
    ++counter;
  }
  checkGyroscope();

  //delay(50);
  webSocket.loop();
  sendJsonUpdate();
}

void sendJsonUpdate() {

  //Nur senden, wenn Aenderung vorhanden!
  const int currentX = getAverageFromAxis(XAXIS);
  const int currentY = getAverageFromAxis(YAXIS);
  const int currentZ = getAverageFromAxis(ZAXIS);
  
  if (currentX == lastX && currentY == lastY && currentZ == lastZ) {
    return;
  } else {
    lastX = currentX;
    lastY = currentY;
    lastZ = currentZ;
  }
  StaticJsonBuffer<200> doc;
  JsonObject& root = doc.createObject();
  root["type"] = "update";
  root["id"] = wssID;
  root["rotX"] = currentX;
  root["rotY"] = currentY;
  root["rotZ"] = currentZ;
  String out;
  root.printTo(out);
  ++messageCounter;
  webSocket.sendTXT(out);

}

int getAverageFromAxis(int axis){
  int *p;
  switch(axis) {
    case XAXIS:
      p = X;
      break;
    case YAXIS:
      p = Y;
      break;
    case ZAXIS:
      p = Z;
      break;
  }
  int average = 0;
  for(int i=0; i<MAXSENSORAMOUNT; ++i){
    average += p[i];
  }
  return average / MAXSENSORAMOUNT;
}

void setGyroscope() {
  Serial.println("Waiting for BNO-Sensor ");
  /* Initialise the sensor without Magnetfield*/
  if (!bno.begin(bno.OPERATION_MODE_IMUPLUS))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  Serial.print("Waiting!");
  for (int i = 0; i < 10; ++i) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("Done!");

  bno.setExtCrystalUse(true);
}

void setWSS() {

  //wifiMulti.addAP("SSID", "PASSPASS");
  wifiMulti.addAP("dh2018", "digitalhumanities");
  Serial.println("");
  Serial.print("Connecting ");
  int counter = 0;
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
  }
  Serial.println(" Wifi Connected!");

  webSocket.begin("192.168.1.1", 2000);
  Serial.println("WebSocket connected(?)");

  webSocket.onEvent(webSocketEvent);
  Serial.println("OnEvent");

}

void checkGyroscope() {

  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  /* Standardausrichtung
    X = euler.x();
    Y = euler.y();
    Z = euler.z();
  */

  //Ausrichtung für DH2018 und Holzwürfel
  X[currentSensorSpot] = euler.z();
  Y[currentSensorSpot] = euler.x() * -1.0;
  Z[currentSensorSpot] = euler.y() * -1.0;
  currentSensorSpot = (currentSensorSpot + 1) % MAXSENSORAMOUNT;
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  Serial.print("WebSocketEvent:  ");
  Serial.println(type);

  switch (type) {
    case WStype_DISCONNECTED: {
        Serial.printf("[WSc] Disconnected!\n");
        break;
      }

    case WStype_CONNECTED: {
        Serial.printf("[WSc] Connected to url: %s\n",  payload);
        break;
      }

    case WStype_TEXT: {
        Serial.printf("[WSc] get text: %s\n", payload);
        StaticJsonBuffer<200> doc;
        JsonObject& in = doc.parseObject(payload);
        //wssID = in["id"];


        //Sending create-Message
        StaticJsonBuffer<200> docOut;
        JsonObject& root = docOut.createObject();
        root["type"] = "create";
        root["color"] = "#0000a0";
        root["id"] = wssID;
        String out;
        root.printTo(out);
        webSocket.sendTXT(out);
      }
      break;
  }

}

