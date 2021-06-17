#include <Arduino.h>
#include <Filters.h> 
#include <ESP32AnalogRead.h>
#include <math.h>
#include "soc/soc.h"          // Disable brownour problems
#include "soc/rtc_cntl_reg.h" // Disable brownour problems
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

//pin configuration
const int acPin = 34; // adc on GPIO 34
int Ledboard = 2;
const int ledPin =2;

//variable for filter 
float testFrequency = 50;                     // test signal frequency (Hz)
float windowLength = 40.0/testFrequency;     // how long to average the signal, for statistist
int Sensor = 34; //Sensor analog input, here it's A0
float intercept = -0.04; // to be adjusted based on calibration testing
float slope = 0.0405; // to be adjusted based on calibration testing
float current_Volts; // Voltage
unsigned long printPeriod = 1000; //Refresh rate
unsigned long previousMillis = 0;

//variable  
float phVol = 0.0;
int mQtyFailCt = 5;

int i = 5;  // variable for loop
int y = 10; // variable for wifi reset
String S_ResetCount;
unsigned int timer1s;
unsigned int intResetCount;
char charResetCount[200];

// timer interrupt
volatile int interruptCounter1;
volatile int interruptCounter1000;

int timerCount; // test statement for each step in second
char c_timerCount[8];
boolean flagEx = false; // flag to excute 1 time the statement

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// mqtt configuration
const char *mqtt_server = "192.168.0.50";
WiFiClient espClient;
PubSubClient client(espClient);

// wifi configuration
const char *ssid = "CLV";
const char *password = "Pi@Riya*1";
bool internet_connected = false;
struct tm timeinfo;
time_t now;
char strftime_buf[64];              // time for webserver
char C_ip_adress[14] = "IP adress"; // for Mqtt ID
char C_mac_adr[18];                 // for Mqtt ID
char C_idHostname[40];
char C_topic_ac_Hostname[40] = "esp32/ac/";

// my time
// int day, hours, minutes, seconds, year, month, date, minuteSave;
// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP, "0.pool.ntp.org", 25200, 0);
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 25200;
const int daylightOffset_sec = 0;

// variable for webserver
const char *PARAM_ipAdress = "ipAdress";
const char *PARAM_macAdress = "macAdress";
const char *PARAM_idHostname = "idHostname";
const char *PARAM_timeCycle = "timeCycle";

int Int_timeCycle;

String S_ipAdress;
String S_macAdress;
String S_idHostname;
String S_timeCycle;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
RunningStatistics inputStats; 

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
bool init_wifi() {
  int connAttempts = 0;
  Serial.println("\r\nConnecting to: " + String(ssid));
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);

  S_idHostname = readFile(SPIFFS, "/idHostname.txt");
  S_idHostname.toCharArray(C_idHostname, 40);
  WiFi.setHostname(C_idHostname);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.print(".");
    if (connAttempts > 10)
      return false;
    connAttempts++;
  }
  return true;
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
void adcToVolt() {

  inputStats.setWindowSecs( windowLength );
  Sensor = analogRead(acPin);  // read the analog in value:
  inputStats.input(Sensor);  // log to Stats function
  current_Volts = intercept + slope * inputStats.sigma(); //Calibartions for offset and amplitude
  current_Volts= current_Volts*(53.0223);                //Further calibrations for the amplitude
  
 
}

// Processor read back to value on website
String processor(const String &var) // display value on http
{
  if (var == "idHostname") {
    S_idHostname = readFile(SPIFFS, "/idHostname.txt");
    return readFile(SPIFFS, "/idHostname.txt");
  } else if (var == "timeNow") {
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%F_%H_%M_%S", &timeinfo);
    return String(strftime_buf);
  } else if (var == "ipAdress") {
    return String(WiFi.localIP().toString());
  } else if (var == "macAdress") {
    return String(WiFi.macAddress());
  } else if (var == "resetCount") {
    return readFile(SPIFFS, "/resetCount.txt");
  } else if (var == "timeCycle") {
    S_timeCycle = readFile(SPIFFS, "/timeCycle.txt");
    return readFile(SPIFFS, "/timeCycle.txt");
  }

  return String();
}
void init_server() // Server init
{
  File file = SPIFFS.open("/index.html", "r");
  if (!file) {
    Serial.println("file open failed");
  }
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  // Read hostname
  S_idHostname = readFile(SPIFFS, "/idHostname.txt");
  // Read timeCycle
  S_timeCycle = readFile(SPIFFS, "/timeCycle.txt");
  Int_timeCycle = S_timeCycle.toInt();

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
    writeFile(SPIFFS, "/resetCount.txt", "0");
    delay(10);
    ESP.restart();
  });
 
  // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request) {
    String inputMessage;
    // String inputParam; //no used
    // GET timeBetween value on <ESP_IP>/get?timeBetween=<inputMessage>

    if (request->hasParam(PARAM_idHostname)) {
      inputMessage = request->getParam(PARAM_idHostname)->value();
      writeFile(SPIFFS, "/idHostname.txt", inputMessage.c_str());
    } else if (request->hasParam(PARAM_timeCycle)) {
      inputMessage = request->getParam(PARAM_timeCycle)->value();
      writeFile(SPIFFS, "/timeCycle.txt", inputMessage.c_str());
    } else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/text", inputMessage);
  });
  server.begin();
} // end Server init
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
  while (!client.connected() && (mQtyFailCt >= 0)) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = C_idHostname;
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("esp32/output");
      mQtyFailCt = 5;
    } else if (mQtyFailCt == 0) {
      Serial.println("Mqtt fail 5 time restart esp32");
      ESP.restart();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
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
    S_ResetCount = readFile(SPIFFS, "/resetCount.txt");
    intResetCount = S_ResetCount.toInt() + 1;
    writeFile(SPIFFS, "/resetCount.txt",
              itoa(intResetCount, charResetCount, 10));
    ESP.restart();
  }
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter1++;
  interruptCounter1000++;
  portEXIT_CRITICAL_ISR(&timerMux);
}
void setup() {
 // put your setup code here, to run once:
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector

  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000, true);
  Serial.write("Hello world");
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  } else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }
  if (init_wifi()) { // Connected to WiFi
    internet_connected = true;
    Serial.println("Internet connected");
    // Print ESP32 Local IP Address
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.macAddress());
    init_time();
    time(&now);
    // setenv("TZ", "GMT0BST,M3.5.0/01,M10.5.0/02", 1);
    // tzset();
    WiFi.localIP().toString().toCharArray(
        C_ip_adress, 14); // Convert IP adress to String then to Char Array
    WiFi.macAddress().toCharArray(C_mac_adr,
                                  18); // Convert Mac adr to Char array
    Serial.write("Wifi connected");
  }
 
  init_server();                       // start server
  client.setServer(mqtt_server, 1883); // start mqtt
  client.setCallback(callback);
  strcat(C_topic_ac_Hostname, C_idHostname);


  // Init pin mode
  pinMode(Ledboard, OUTPUT);
  // OTA init
  init_OTA();
  timerAlarmEnable(timer);
  // Initialise the first sensor value store. We want this to be the simple
  // average of the last 10 values.
  // Note: The more values you store, the more memory will be used.
  analogReadResolution(10);
  adcAttachPin(acPin);
  RunningStatistics inputStats;  //for Filters.h
}

void loop() {
  adcToVolt();
  client.loop();
  ArduinoOTA.handle();
  if (interruptCounter1 > 1) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter1 = 0;
    portEXIT_CRITICAL(&timerMux);
  }
  if (interruptCounter1000 >= 1000) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter1000 = 0;
    portEXIT_CRITICAL(&timerMux);
    checkConnection();

    timerCount++;
    if (!client.connected()) {
      reconnect();
    }
    // writeFile(SPIFFS, "/timerCount.txt", itoa(timerCount, c_timerCount, 10));
    // S_timeCycle = readFile(SPIFFS, "/timeCycle.txt");
    // Int_timeCycle = S_timeCycle.toInt();
    // flagEx = false;

    if (digitalRead(ledPin) == HIGH) {
      digitalWrite(ledPin, LOW);
    } else {
      digitalWrite(ledPin, HIGH);
    }
    timer1s++;
    current_Volts = intercept + slope * inputStats.sigma(); //Calibartions for offset and amplitude
    current_Volts= current_Volts*(53.0223);                //Further calibrations for the amplitude
    Serial.print( "\tVoltage, " );
    Serial.println( current_Volts );
  }
}
