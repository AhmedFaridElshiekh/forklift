// firebase_handler.cpp
#include "firebase_handler.h"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson jsonData;

Point path[20];
int pathLength = 0;

void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("Wi-Fi connected.");
}

void initFirebase() {
  config.database_url = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("Firebase initialized.");
}

void sendStatus(const String& status) {
  int x = 2; // replace with actual
  int y = 3;
  int battery = 83;

  time_t now;
  time(&now);

  jsonData.clear();
  jsonData.set("x", x);
  jsonData.set("y", y);
  jsonData.set("batteryLevel", battery);
  jsonData.set("lastSeen", now * 1000);
  jsonData.set("status", status);

  if (Firebase.setJSON(fbdo, FIREBASE_PATH, jsonData)) {
    Serial.println("✅ Status sent.");
  } else {
    Serial.println("❌ Failed to send status: " + fbdo.errorReason());
  }
}
int fetchPathPoints(int pathArray[20][2]) {
  int length = 0;

  if (Firebase.getJSON(fbdo, "tasks/qq")) {
    FirebaseJson& json = fbdo.jsonObject();
    FirebaseJsonData jsonData;

    if (json.get(jsonData, "path") && jsonData.type == "array") {
      FirebaseJsonArray arr;
      arr.setJsonArrayData(jsonData.stringValue);

      for (size_t i = 0; i < arr.size(); i++) {
        FirebaseJsonData pointData;
        arr.get(pointData, i);

        FirebaseJson jsonPoint;
        jsonPoint.setJsonData(pointData.stringValue);

        FirebaseJsonData xData, yData;
        jsonPoint.get(xData, "x");
        jsonPoint.get(yData, "y");

        if (length < 20) {
          pathArray[length][0] = xData.intValue;  // x
          pathArray[length][1] = yData.intValue;  // y
          length++;

          Serial.print("Point ");
          Serial.print(i);
          Serial.print(": x = ");
          Serial.print(xData.intValue);
          Serial.print(", y = ");
          Serial.println(yData.intValue);
        } else {
          Serial.println("⚠️ Max path length reached.");
          break;
        }
      }

      Serial.print("✅ Total points: ");
      Serial.println(length);
    } else {
      Serial.println("❌ 'path' not found or not array");
    }
  } else {
    Serial.println("❌ Firebase GET failed: " + fbdo.errorReason());
  }

  return length;
}
