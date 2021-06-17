#include <Arduino.h>
#include "soc/soc.h"          // Disable brownour problems
#include "soc/rtc_cntl_reg.h" // Disable brownour problems
#include <SPIFFS.h>
#include <FS.h>
#include <WiFi.h>             //Wifi http
#include <ESPAsyncWebServer.h> //html
#include <PubSubClient.h> //mqtt
#include <Filters.h> 


#include "initbase.h"

// variable hardware
const int ledPin = 2;
const int acPin = 34;

// timer interrupt
volatile int interruptCounter1;
volatile int interruptCounter1000;
int timerCount; // test statement for each step in second
char c_timerCount[8];
boolean flagEx = false; // flag to excute 1 time the statement
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


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
String S_currentVoltage;

char C_topic_ac_Hostname[40] = "esp32/ac/";
char Cm_idHostname[40];
char C_currentVoltage[20];

//wifi 
WiFiClient espClient;
PubSubClient client(espClient);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);


//filter 
//variable for filter 
float testFrequency = 50;                     // test signal frequency (Hz)
float windowLength = 40.0/testFrequency;     // how long to average the signal, for statistist
int Sensor ; //Sensor analog input, here it's A0
float intercept = -0.04; // to be adjusted based on calibration testing
float slope = 0.0405; // to be adjusted based on calibration testing
float current_Volts; // Voltage

RunningStatistics inputStats; 


// Processor read back to value on website
String processor(const String &var) // display value on http
{
  if (var == "idHostname") {
    S_idHostname = readFile(SPIFFS, "/idHostname.txt");
    return readFile(SPIFFS, "/idHostname.txt");
  } else if (var == "timeNow") {
    return timeNow();
  } else if (var == "ipAdress") {
    return String(WiFi.localIP().toString());
  } else if (var == "macAdress") {
    return String(WiFi.macAddress());
  } else if (var == "resetCount") {
    return readFile(SPIFFS, "/resetCount.txt");
  } else if (var == "timeCycle") {
    S_timeCycle = readFile(SPIFFS, "/timeCycle.txt");
    return readFile(SPIFFS, "/timeCycle.txt");
  } else if (var == "timerCount") {
    return readFile(SPIFFS, "/timerCount.txt"); 
  } else if (var == "currentVoltage") {
    return String(current_Volts); 
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


void adcToVolt() {

  inputStats.setWindowSecs( windowLength );
  Sensor = analogRead(acPin);  // read the analog in value:
  inputStats.input(Sensor);  // log to Stats function
  current_Volts = intercept + slope * inputStats.sigma(); //Calibartions for offset and amplitude
  current_Volts= current_Volts*(53.0223);                //Further calibrations for the amplitude
}

void voltPublish(){
  dtostrf(current_Volts, 3, 2, C_currentVoltage);
  // client.publish(C_topic_ac_Hostname, C_currentVoltage);
  sendPublish(C_topic_ac_Hostname, C_currentVoltage);
  Serial.print("current voltage : ");
  Serial.println(current_Volts);
  Serial.print("hostname : ");
  Serial.println(C_topic_ac_Hostname);
  Serial.print("C_currentVoltage : ");
  Serial.println(C_currentVoltage);
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
    Serial.println("Internet connected");
    // Print ESP32 Local IP Address
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.macAddress());
    init_time();
  }
  init_server();                       // start server
  init_OTA();
  timerAlarmEnable(timer);
  readFile(SPIFFS, "/idHostname.txt").toCharArray(Cm_idHostname, 40);  
  strcat(C_topic_ac_Hostname,Cm_idHostname);
  analogReadResolution(10);
  adcAttachPin(acPin);
}

void loop() {

  ArduinoOTArun();
  adcToVolt();
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
    reconnect();
    timerCount++;
    writeFile(SPIFFS, "/timerCount.txt", itoa(timerCount, c_timerCount, 10));
    S_timeCycle = readFile(SPIFFS, "/timeCycle.txt");
    Int_timeCycle = S_timeCycle.toInt();
    flagEx = false;
    Serial.print("timeCount :");
    Serial.println(readFile(SPIFFS, "/timerCount.txt"));
    
    if (digitalRead(ledPin) == HIGH) {
      digitalWrite(ledPin, LOW);
    } else {
      digitalWrite(ledPin, HIGH);
    }

    if (timerCount == Int_timeCycle) {
      if (flagEx == false) {
        Serial.print("timeCycle :");
        Serial.println(readFile(SPIFFS, "/timeCycle.txt"));
        voltPublish();
        flagEx = true;
        timerCount = 0;
      }
  }  else if (timerCount > Int_timeCycle) {
    if (flagEx == false) {
      flagEx = true;
      timerCount = 0;
    }
  }

  }
}