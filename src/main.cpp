#include <Arduino.h>
#include "soc/soc.h"          // Disable brownour problems
#include "soc/rtc_cntl_reg.h" // Disable brownour problems
#include <SPIFFS.h>
#include <FS.h>
#include <WiFi.h>              //Wifi http
#include <ESPAsyncWebServer.h> //html
#include <PubSubClient.h>      //mqtt
#include <Filters.h>

#include "initbase.h"
#include <Smoothed.h>
Smoothed<float> mySensor;
float smoothedSensorValueAvg;
float totalSum;

// From Solarduino
int decimalPrecision = 2; // decimal places for all values shown in LED Display & Serial Monitor

/* 1- AC Voltage Measurement */

int VoltageAnalogInputPin = 34; // Which pin to measure voltage Value (Pin A0 is
                                // reserved for button function)
float voltageSampleRead =
    0; /* to read the value of a sample in analog including voltageOffset1 */
float voltageLastSample = 0;  /* to count time for each sample. Technically 1
                                 milli second 1 sample is taken */
float voltageSampleSum = 0;   /* accumulation of sample readings */
float voltageSampleCount = 0; /* to count number of sample. */
float voltageMean; /* to calculate the average value from all samples, in analog
           values*/
float RMSVoltageMean; /* square roof of voltageMean without offset value, in
                 analog value*/
float adjustRMSVoltageMean;
float FinalRMSVoltage; /* final voltage value with offset value*/
                       /* 1.1- AC Voltage Offset */

float voltageOffset1 = 0.00; // to Offset deviation and accuracy. Offset any
                             // fake current when no current operates.
// Offset will automatically callibrate when SELECT Button on the LCD Display
// Shield is pressed.
// If you do not have LCD Display Shield, look into serial monitor to add or
// minus the value manually and key in here.
// 26 means add 26 to all analog value measured.
float voltageOffset2 = 0.00; // too offset value due to calculation error from
                             // squared and square root
int offsetSampleCount = 0;   // offset counter
int OffsetStatus = 0;        // step offest status
float offsetVoltageMean = 0.0;

float offsetSampleReadV;    /* sample for offset purpose */
float offsetSampleSumV = 0; /* accumulation of sample readings for offset */
// end variable from solarduino

// variable hardware
const int ledPin = 2;
const int acPin = 34;
const int floatingSensor = 15;

// variable for count Floating
float floatingCnt = 0;
float SendValueAvg = 900; // number windows for count 3600 sec = 60min by defaut
float forAvgCalc  = 900;
float floatingAverge = 0.0;
char C_floatingAverge[20];

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
const char *PARAM_cycleAvg = "cycleAvg";
const char *PARAM_offset1 = "offset1";
const char *PARAM_offset2 = "offset2";

int Int_timeCycle;
int Int_cycleAvg;
String S_ipAdress;
String S_macAdress;
String S_idHostname;
String S_timeCycle;
String S_cycleAvg;
String S_currentVoltage;
String S_voltageOffset1;
String S_voltageOffset2;

char C_topic_ac_Hostname[40] = "esp32/ac/";
char C_topic_floatSwitch_Hostname[40] = "esp32/floatSwitch/";
char Cm_idHostname[40];
char C_currentVoltage[20];
char C_voltRMS[20];
char C_smoothVoltRMS[20];
char C_voltageOffset1[20];
char C_voltageOffset2[20];

// wifi
WiFiClient espClient;
PubSubClient client(espClient);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// filter
// variable for filter
float testFrequency = 50; // test signal frequency (Hz)
float windowLength =
    40.0 / testFrequency; // how long to average the signal, for statistist
int Sensor;               // Sensor analog input, here it's A0
float intercept = -0.04;  // to be adjusted based on calibration testing
float slope = 0.0405;     // to be adjusted based on calibration testing
float current_Volts;      // Voltage
float voltRMS;
float max_v;

RunningStatistics inputStats;

int countRead50Hz;
int countReadNumber = 200;
// Processor read back to value on website
String processor(const String &var) // display value on http
{
  if (var == "idHostname") {
    // S_idHostname = readFile(SPIFFS, "/idHostname.txt");
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
  } else if (var == "cycleAvg") {
    S_cycleAvg = readFile(SPIFFS, "/cycleAvg.txt");
    return readFile(SPIFFS, "/cycleAvg.txt");
  } else if (var == "timerCount") {
    return readFile(SPIFFS, "/timerCount.txt");
  } else if (var == "currentVoltage") {
    return String(FinalRMSVoltage);
  } else if (var == "floatSwitchAvg") {
    return String(floatingAverge);
  } else if (var == "floatCnt") {
    return String(floatingCnt);
  } else if (var == "offset1") {
    return readFile(SPIFFS, "/voltageOffset1.txt");
  } else if (var == "offset2") {
    return readFile(SPIFFS, "/voltageOffset2.txt");
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

  S_cycleAvg = readFile(SPIFFS, "/cycleAvg.txt");
  Int_cycleAvg = S_cycleAvg.toInt();

  server.on("/Calibration", HTTP_GET, [](AsyncWebServerRequest *request) {
    OffsetStatus = 1;
    Serial.print("OffsetStatus : ");
    Serial.println(OffsetStatus);
    request->redirect("/");
  });

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
    } else if (request->hasParam(PARAM_cycleAvg)) {
      inputMessage = request->getParam(PARAM_cycleAvg)->value();
      writeFile(SPIFFS, "/cycleAvg.txt", inputMessage.c_str());
    } else if (request->hasParam(PARAM_offset1)) {
      inputMessage = request->getParam(PARAM_offset1)->value();
      writeFile(SPIFFS, "/voltageOffset1.txt", inputMessage.c_str());
    } else if (request->hasParam(PARAM_offset2)) {
      inputMessage = request->getParam(PARAM_offset2)->value();
      writeFile(SPIFFS, "/voltageOffset2.txt", inputMessage.c_str());

    } else {
      inputMessage = "No message sent";
    }
    request->send(200, "text/text", inputMessage);
  });
  server.begin();
} // end Server init

void calculSpeedWater() {
  floatingAverge = (floatingCnt / readFile(SPIFFS, "/cycleAvg.txt").toInt())*100 ;
  Serial.print("floatingAverge %: ");
  Serial.print(floatingAverge);
  Serial.println(" % ");
  dtostrf(floatingAverge, 3, 2, C_floatingAverge);
  sendPublish(C_topic_floatSwitch_Hostname, C_floatingAverge);
  Serial.print("Topic : ");
  Serial.println(C_topic_floatSwitch_Hostname);
  Serial.print("C_floatingAverge : ");
  Serial.println(C_floatingAverge);
  floatingCnt = 0;
}

void adcToVolt() {
  /* 1- AC Voltage Measurement */

  offsetSampleReadV = (analogRead(VoltageAnalogInputPin) -
                       512); /* read the sample value for offset purpose*/
  offsetSampleSumV = offsetSampleSumV + offsetSampleReadV;

  voltageSampleRead =
      (analogRead(VoltageAnalogInputPin) - 512) +
      voltageOffset1; /* read the sample value including offset value*/
  voltageSampleSum = voltageSampleSum +
                     sq(voltageSampleRead); /* accumulate total analog values
                                               for each sample readings*/
  voltageSampleCount =
      voltageSampleCount + 1; /* to move on to the next following count */

  if (voltageSampleCount ==
      1000) /* after 4000 count or 800 milli seconds (0.8 second), do the
               calculation and display value*/
  {
    offsetVoltageMean = offsetSampleSumV / voltageSampleCount;
    voltageMean = voltageSampleSum /
                  voltageSampleCount; /* calculate average value of all sample
                                         readings taken*/
    RMSVoltageMean = (sqrt(voltageMean)) * 1.5; // The value X 1.5 means the
                                                // ratio towards the module
                                                // amplification.
    adjustRMSVoltageMean = RMSVoltageMean + voltageOffset2;
    /* square root of the average value including offset value */ /* square
                                                                     root of
                                                                     the
                                                                     average
                                                                     value*/
    FinalRMSVoltage =
        RMSVoltageMean + voltageOffset2; /* this is the final RMS voltage*/
    if (FinalRMSVoltage <= 2.5) /* to eliminate any possible ghost value*/
    {
      FinalRMSVoltage = 0;
    }
    // Serial.print(" The Voltage RMS value is: ");
    // Serial.print(FinalRMSVoltage, decimalPrecision);
    // Serial.println(" V ");
    dtostrf(FinalRMSVoltage, 3, 2, C_voltRMS);
    offsetSampleSumV = 0;
    voltageSampleSum =
        0; /* to reset accumulate sample values for the next cycle */
    voltageSampleCount = 0; /* to reset number of sample for the next cycle */
    mySensor.add(FinalRMSVoltage);
    smoothedSensorValueAvg = mySensor.get();
    dtostrf(smoothedSensorValueAvg, 3, 2, C_smoothVoltRMS);
    // Serial.print("smooth voltage : ");
    // Serial.println(C_smoothVoltRMS);
  }
}
void offsetAC() {

  if (OffsetStatus == 1) {
    offsetSampleCount = offsetSampleCount + 1;
    if (offsetSampleCount == 1500) /* after 1.5 seconds, run this codes.  */
    {
      voltageOffset1 = -offsetVoltageMean; /* to offset values */
      dtostrf(voltageOffset1, 3, 2, C_voltageOffset1);
      writeFile(SPIFFS, "/voltageOffset1.txt", C_voltageOffset1);
      OffsetStatus = 2; /* go for the next offset setting*/
      offsetSampleCount =
          0; /* to reset the time again so that next cycle can start again */
    }
  }

  if (OffsetStatus == 2) /* second offset is continued */
  {
    voltageOffset2 = 0; /* set back voltageOffset2 as default*/
    offsetSampleCount = offsetSampleCount + 1;

    if (offsetSampleCount == 2500) /* after 2.5 seconds, run this codes.  */
    {
      voltageOffset2 = -RMSVoltageMean; /* to offset values */
      dtostrf(voltageOffset2, 3, 2, C_voltageOffset2);
      writeFile(SPIFFS, "/voltageOffset2.txt", C_voltageOffset2);
      OffsetStatus = 0;
      /* go for the next offset setting*/ /* until next offset button is
                                             pressed*/
      offsetSampleCount =
          0; /* to reset the time again so that next cycle can start again */
    }
  }
}
// void voltPublish() { sendPublish(C_topic_ac_Hostname, C_voltRMS); }
void voltPublish() { sendPublish(C_topic_ac_Hostname, C_smoothVoltRMS); }

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
  init_server(); // start server
  init_OTA();
  timerAlarmEnable(timer);
  S_idHostname.toCharArray(Cm_idHostname, 40);
  strcat(C_topic_ac_Hostname, Cm_idHostname);
  strcat(C_topic_floatSwitch_Hostname, Cm_idHostname);
  mySensor.begin(SMOOTHED_AVERAGE, 10);
  mySensor.clear();
  analogReadResolution(10);
  adcAttachPin(acPin);
  pinMode(floatingSensor, INPUT_PULLDOWN);
}

void loop() {

  ArduinoOTArun();

  if (interruptCounter1 > 1) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter1 = 0;
    portEXIT_CRITICAL(&timerMux);
    adcToVolt();
    offsetAC();
  }

  if (interruptCounter1000 >= 1000) {
    portENTER_CRITICAL(&timerMux);
    interruptCounter1000 = 0;
    portEXIT_CRITICAL(&timerMux);

    voltageOffset1 = (readFile(SPIFFS, "/voltageOffset1.txt")).toFloat();
    voltageOffset2 = (readFile(SPIFFS, "/voltageOffset2.txt")).toFloat();
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

    Int_cycleAvg--;

    if (digitalRead(floatingSensor) == HIGH) {
      floatingCnt++;
      Serial.print("floatingCnt :");
      Serial.println(floatingCnt);
    }
    if (Int_cycleAvg == 0) {
      calculSpeedWater();
      Int_cycleAvg= readFile(SPIFFS, "/cycleAvg.txt").toInt();

    }

    if (timerCount == Int_timeCycle) {
      if (flagEx == false) {
        Serial.print("timeCycle :");
        Serial.println(readFile(SPIFFS, "/timeCycle.txt"));
        voltPublish();
        flagEx = true;
        timerCount = 0;
      }
    } else if (timerCount > Int_timeCycle) {
      if (flagEx == false) {
        flagEx = true;
        timerCount = 0;
      }
    }
  }
}