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

const int joyX = A2, joyY = A3;
const int joyPress = 12;

const int buttonModel = 15;
const int buttonZoomIn = 32, buttonZoomOut = 33;
const int ledSetup = 21, ledDone = 27;

const int modelMaxTrigger = 20;
const int zoomMaxTrigger = 20;

WiFiMulti wifiMulti;
WebSocketsClient webSocket;
const char* wssID = "1";
int currentIP = 0;

double  X = 0, Y = 0, Z = 0;
double lastX = 0, lastY = 0, lastZ = 0;
int counter = 0;
int messageCounter = 0;
unsigned long timeStart;

int zoomInCounter = 0, zoomOutCounter = 0;
int modelCounter = 0;

int modelPos = 0;
double zoomFactor = 0;
boolean modelSend= false;

double currentXPos = 0.0, currentYPos = 0.0;
int medianXPos = 0, medianYPos = 0;

void setup() {
  pinMode(joyX, INPUT);
  pinMode(joyY, INPUT);
  
  for(int i=0; i<100;++i){
    medianXPos += analogRead(joyX) - 2048;
    medianYPos += analogRead(joyY) - 2048;
    delay(10);
  }
  medianXPos = medianXPos / 100;
  medianYPos = medianYPos / 100;
  
  pinMode(ledSetup, OUTPUT);
  pinMode(ledDone, OUTPUT);
  digitalWrite(ledDone, LOW);
  digitalWrite(ledSetup, HIGH);
  
  Serial.begin(115200);
  Wire.begin();

  //pinMode for Buttons
  pinMode(buttonModel, INPUT);
  pinMode(buttonZoomIn, INPUT);
  pinMode(buttonZoomOut, INPUT);

  Serial.println("Setup");
  setGyroscope();
  setWSS();
  Serial.println("Setup Done!");
  
  digitalWrite(ledSetup, LOW);
  digitalWrite(ledDone, HIGH);
}


void loop() {

  int tmpXPos = analogRead(joyX);
  int tmpYPos = analogRead(joyY);
  tmpXPos = (tmpXPos - medianXPos - 2048);
  currentXPos = (tmpXPos - (tmpXPos % 100)) / 10000.0;
  tmpYPos = (tmpYPos - medianYPos - 2048 );
  currentYPos = (tmpYPos - (tmpYPos % 100)) / 10000.0;
  if (counter == 0) {
    timeStart = millis();
  }

  if (digitalRead(buttonModel) == HIGH) {
    ++modelCounter;
    if(modelCounter >= modelMaxTrigger){
      modelSend = true;
      modelCounter = modelCounter >= modelMaxTrigger ? (modelMaxTrigger / 2) : modelCounter;
    }
  } else {
    modelCounter += modelCounter > 0 ? -2 : 0;
  }

  if (digitalRead(buttonZoomIn) == HIGH ) {
    zoomInCounter = zoomInCounter >= zoomMaxTrigger ? (zoomMaxTrigger / 2) : zoomInCounter + 1;
  } else if (digitalRead(buttonZoomOut) == HIGH) {
    zoomOutCounter = zoomOutCounter >= zoomMaxTrigger ? (zoomMaxTrigger / 2) : zoomOutCounter + 1;
  } else {
    zoomInCounter += zoomInCounter > 0 ? -2 : 0;
    zoomOutCounter += zoomOutCounter > 0 ? -2 : 0;
  }
  if (millis() - timeStart >= 1000) {
    Serial.println((String)"Cycles per Second: " + counter + "\t\tMessages per Second: " + messageCounter+"\t\t"+medianXPos + " / " + medianYPos);
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
  if (  X == lastX && Y == lastY && Z == lastZ  //Gyroskop
        && zoomInCounter  < (zoomMaxTrigger / 2) && zoomOutCounter < (zoomMaxTrigger / 2) //Zoom-Buttons
        && (!modelSend || modelCounter != 0) //Model-Button
        && abs(currentXPos) <= 0.01 && abs(currentYPos) <= 0.01) { //Joystick
    return;
  }
//  Serial.println((String)"Sending because of:\nX: " + (X == lastX) + "\tY: "+ (Y == lastY) + "\tZ: " + (Z == lastZ)
//                                                    +"\nZIC: " + (zoomInCounter < (zoomMaxTrigger / 2)) + "\tZOC: " + (zoomOutCounter < (zoomMaxTrigger / 2)) 
//                                                    +"\tMPC: " + !modelSend + "/" + (modelCounter > 0));
  StaticJsonBuffer<200> doc;
  JsonObject& root = doc.createObject();
  root["type"] = "update";
  root["id"] = wssID;
  root["rotX"] = (abs(X - lastX)) > 5 ? X : ((double)X + (double)lastX) / 2.0;
  root["rotY"] = (abs(Y - lastY)) > 5 ? Y : ((double)Y + (double)lastY) / 2.0;
  root["rotZ"] = (abs(Z - lastZ)) > 5 ? Z : ((double)Z + (double)lastZ) / 2.0;
  if(zoomInCounter >= (zoomMaxTrigger / 2) || zoomOutCounter >= (zoomMaxTrigger / 2)){
    root["zoom"] = zoomInCounter > zoomOutCounter ? 0.0125 : -0.0125;
  }else{
    root["zoom"] = 0;
  }
  if(modelSend && modelCounter <= 0 ){
    root["currentPosModel"] = 1;
    modelSend = false;
  }else{
    root["currentPosModel"] = 0;
  }
  root["posX"] = currentXPos;
  root["posY"] = currentYPos;
  lastX = X;
  lastY = Y;
  lastZ = Z;
  zoomFactor = 0;
  modelPos = 0;
  String out;
  root.printTo(out);
  ++messageCounter;
  webSocket.sendTXT(out);

}

void setGyroscope() {
  Serial.println("Waiting for BNO-Sensor ");
  /* Initialise the sensor without Magnetfield*/
  if (!bno.begin(bno.OPERATION_MODE_IMUPLUS))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    digitalWrite(ledSetup, LOW);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
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
  X = euler.z();
  Y = euler.x() * -1.0;
  Z = euler.y() * -1.0;
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
