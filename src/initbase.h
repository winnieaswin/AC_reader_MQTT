#ifndef initbase_FILE
#define initbase_FILE


#include <FS.h>
#include "initbase.h"


// SPIFFS read & write
String readFile(fs::FS &fs, const char *path) ;
// SPIFFS read & write
void writeFile(fs::FS &fs, const char *path, const char *message);
void init_time();
bool init_wifi();
void callback(char *topic, byte *payload, unsigned int length);
void reconnect();
void init_OTA();
void checkConnection();
String timeNow ();
void ArduinoOTArun();
void sendPublish(char topicChar[40], char messageToSend[20]);
#endif
