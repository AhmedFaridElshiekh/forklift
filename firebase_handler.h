// firebase_handler.h
#ifndef FIREBASE_HANDLER_H
#define FIREBASE_HANDLER_H

#include <WiFi.h>
#include <FirebaseESP32.h>
#include "types.h"

#define WIFI_SSID "SSID2"
#define WIFI_PASSWORD "11111111"
#define FIREBASE_HOST "fork-app-2200b-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "AIzaSyBs7SjQz_GG338jTm3Uxdxjq0ZgeRgOmy8"
#define FIREBASE_PATH "/robots/qq"

// Firebase objects
extern FirebaseData fbdo;
extern FirebaseAuth auth;
extern FirebaseConfig config;

void initWiFi();
void initFirebase();
int fetchPathPoints(int pathArray[20][2]);
void sendStatus(const String& status);

#endif
