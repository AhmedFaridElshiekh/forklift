#ifndef RFID_H
#define RFID_H

#include "config.h"
#include <MFRC522.h>
#include <SPI.h>

extern MFRC522 rfid;
extern MFRC522::MIFARE_Key key;
extern byte block;

void initRFID();
bool readXYFromCard(byte &xValue, byte &yValue);

#endif
