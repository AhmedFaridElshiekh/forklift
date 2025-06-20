#include "rfid.h"
#include "config.h"
MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
MFRC522::MIFARE_Key key;
byte block = 1;

void initRFID() {
  SPI.begin();
  rfid.PCD_Init();

  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  Serial.println("📢 RFID ready to scan...");
}

bool readXYFromCard(byte &xValue, byte &yValue) {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial())
    return false;

  MFRC522::StatusCode status;

  // التوثيق
  status = rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(rfid.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("❌ Auth failed: ");
    Serial.println(rfid.GetStatusCodeName(status));
    return false;
  }

  byte buffer[18];
  byte size = sizeof(buffer);

  // قراءة البيانات
  status = rfid.MIFARE_Read(block, buffer, &size);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("❌ Read failed: ");
    Serial.println(rfid.GetStatusCodeName(status));
    return false;
  }

  xValue = buffer[0];
  yValue = buffer[1];

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  return true;
}
