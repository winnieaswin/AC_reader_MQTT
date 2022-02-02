#include <Arduino.h>

#include <WiFi.h>             //Wifi http
#include <ESPAsyncWebServer.h> //html
#include <SPIFFS.h>
#include <FS.h>           // file read apprend
#include <PubSubClient.h> //mqtt
#include <ESPmDNS.h>    //OTA
#include <WiFiUdp.h>    //OTA
#include <ArduinoOTA.h> //OTA
#include <string.h>
#include <stdio.h>
#include "initbase.h"

const char *ssid = "CLV";
const char *password = "Pi@Riya*1";
//pin configuration
int Ledboard = 2;

//variable  
int mQtyFailCt = 5;
int i = 5;  // variable for loop
int y = 10; // variable for wifi reset
unsigned int intResetCount;
char charResetCount[200];

bool internet_connected = false;

// my time
// int day, hours, minutes, seconds, year, month, date, minuteSave;
// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP, "0.pool.ntp.org", 25200, 0);
struct tm timeinfo;
time_t now;
char strftime_buf[64];              // time for webserver
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;
const int daylightOffset_sec = 0;


char C_ip_adress[14] = "IP adress"; // for Mqtt ID
char C_mac_adr[18];                 // for Mqtt ID
char C_idHostname[40];

// mqtt configuration & wifi
const char *mqtt_serverInt = "192.168.0.50";
WiFiClient espClientInt;
PubSubClient clientInit(espClientInt);

// SPIFFS read & write
String readFile(fs::FS &fs, const char *path) {
  // Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");
  if (!file || file.isDirectory()) {
    // Serial.println("- empty file or failed to open file");
    return String();
  }
  // Serial.println("- read from file:");
  String fileContent;
  while (file.available()) {
    fileContent += String((char)file.read());
  }
  Serial.println(fileContent);
  return fileContent;
}
// SPIFFS read & write
void writeFile(fs::FS &fs, const char *path, const char *message) {
  // Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, "w");
  if (!file) {
    // Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    // Serial.println("- file written");
  } else {
    // Serial.println("- write failed");
  }
}

void init_time() {
  struct tm timeinfo;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  // wait for time to be set
  time_t now = 0;
  timeinfo = {0};
  int retry = 0;
  const int retry_count = 10;
  while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
    Serial.printf("Waiting for system time to be set... (%d/%d)\n", retry,
                  retry_count);
    delay(2000);
    time(&now);
    localtime_r(&now, &timeinfo);
  }
}
bool init_wifi() {
  int connAttempts = 0;
  Serial.println("\r\nConnecting to: " + String(ssid));
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  readFile(SPIFFS, "/idHostname.txt").toCharArray(C_idHostname, 40);
  WiFi.setHostname(C_idHostname);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.print(".");
    if (connAttempts > 10)
      return false;
    connAttempts++;
  }

  WiFi.localIP().toString().toCharArray(
        C_ip_adress, 14); // Convert IP adress to String then to Char Array
  WiFi.macAddress().toCharArray(C_mac_adr,
                                  18); // Convert Mac adr to Char array
  clientInit.setServer(mqtt_serverInt, 1883); // start mqtt
  clientInit.setCallback(callback);
  Serial.write("Wifi connected");
  return true;
}


void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // Switch on the LED if an 1 was received as first character
  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("on");
      digitalWrite(Ledboard, HIGH);
    } else if (messageTemp == "off") {
      Serial.println("off");
      digitalWrite(Ledboard, LOW);
    }
  }
}

void reconnect() // reconnect mqtt server
{
  // Loop until we're reconnected
  while (!clientInit.connected() && (mQtyFailCt >= 0)) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = C_idHostname;
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (clientInit.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      clientInit.publish("outTopic", "hello world");
      // ... and resubscribe
      clientInit.subscribe("esp32/output");
      mQtyFailCt = 5;
    } else if (mQtyFailCt == 0) {
      Serial.println("Mqtt fail 5 time restart esp32");
      ESP.restart();
    } else {
      Serial.print("failed, rc=");
      Serial.print(clientInit.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      mQtyFailCt--;
    }
  }
}
// code OTA
void init_OTA() {
  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
        // using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

  ArduinoOTA.begin();
}

void ArduinoOTArun(){
    ArduinoOTA.handle();
    clientInit.loop();

}
void checkConnection() {
  if (WiFi.status() == WL_CONNECTED) {
    delay(10);
    // Serial.println("Wifi Connected");
    y = 10;
  } else if ((WiFi.status() != WL_CONNECTED) && (y > 0)) {
    WiFi.reconnect();
    delay(100);
    Serial.print("Wifi no connected : ");
    Serial.println(y);
    --y; // decrease in interrupt
  } else if (y == 0) {
    Serial.println("Wifi No Connected need to reboot");
    intResetCount = (readFile(SPIFFS, "/resetCount.txt")).toInt() + 1;
    writeFile(SPIFFS, "/resetCount.txt",
              itoa(intResetCount, charResetCount, 10));
    ESP.restart();
  }
}

String timeNow (){
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%F_%H_%M_%S", &timeinfo);
    return String (strftime_buf);
}

void sendPublish(char topicChar[40], char messageToSend[20]){
    clientInit.publish(topicChar,messageToSend);
}
