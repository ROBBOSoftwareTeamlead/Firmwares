/*
 * Diagnostic firmware for Nano / Robot / Lab / OTTO (Nulllab).
 * Reads MODEL_ID from serial number in EEPROM; outputs "ROBBO-XXXXX." (5 digits).
 * Serial: 115200 baud. Host (RobboScratch flasher) switches to 115200 after flashing
 * and waits for this string; used as firmwares["diagnostics"] when flashing at 57600
 * (candidate nano/otto_nulllab in DIAGNOSTIC_CANDIDATES).
 */
#include <EEPROM.h>

#define SERIAL_SPEED 115200
#define SERIAL_ADDRESS 0
#define SERIAL_RAW_MAX 49

char chararrSerialRaw[50];
char chararrModel[21];
char chararrVersion[21];
char chararrPart[21];
char chararrSerial[21];
int MODEL_ID;



void parseSerialNumber(){
    EEPROM.get(SERIAL_ADDRESS, chararrSerialRaw);
    chararrSerialRaw[SERIAL_RAW_MAX] = '\0';

    int iPointer = 0;
    while (iPointer <= SERIAL_RAW_MAX && chararrSerialRaw[iPointer] != '-' && chararrSerialRaw[iPointer] != '\0') {
      iPointer++;
    }
    if (iPointer > SERIAL_RAW_MAX) { MODEL_ID = 9999; return; }
    iPointer++;

    int iModelOffset = 0;
    while (iPointer <= SERIAL_RAW_MAX && chararrSerialRaw[iPointer] != '-' && chararrSerialRaw[iPointer] != '\0' && iModelOffset < 20) {
      chararrModel[iModelOffset] = chararrSerialRaw[iPointer];
      iModelOffset++;
      iPointer++;
    }
    chararrModel[iModelOffset] = '\0';
    if (iPointer > SERIAL_RAW_MAX) { MODEL_ID = 9999; return; }
    iPointer++;

    int iVersionOffset = 0;
    while (iPointer <= SERIAL_RAW_MAX && chararrSerialRaw[iPointer] != '-' && chararrSerialRaw[iPointer] != '\0' && iVersionOffset < 20) {
      chararrVersion[iVersionOffset] = chararrSerialRaw[iPointer];
      iVersionOffset++;
      iPointer++;
    }
    chararrVersion[iVersionOffset] = '\0';
    if (iPointer > SERIAL_RAW_MAX) { MODEL_ID = 9999; return; }
    iPointer++;

    int iPartOffset = 0;
    while (iPointer <= SERIAL_RAW_MAX && chararrSerialRaw[iPointer] != '-' && chararrSerialRaw[iPointer] != '\0' && iPartOffset < 20) {
      chararrPart[iPartOffset] = chararrSerialRaw[iPointer];
      iPartOffset++;
      iPointer++;
    }
    chararrPart[iPartOffset] = '\0';
    if (iPointer > SERIAL_RAW_MAX) { MODEL_ID = 9999; return; }
    iPointer++;

    int iSerialOffset = 0;
    while (iPointer <= SERIAL_RAW_MAX && chararrSerialRaw[iPointer] != '\0' && iSerialOffset < 20) {
      chararrSerial[iSerialOffset] = chararrSerialRaw[iPointer];
      iSerialOffset++;
      iPointer++;
    }
    chararrSerial[iSerialOffset] = '\0';

    if(strcmp(chararrModel, "R") == 0
       && strcmp(chararrVersion, "1") == 0
       /* && (strcmp(chararrPart, "1") == 0 || strcmp(chararrPart, "2") == 0 || strcmp(chararrPart, "3") == 0 || strcmp(chararrPart, "4") == 0 || strcmp(chararrPart, "5") == 0) */ ){

       MODEL_ID=0;
    }
    else if(strcmp(chararrModel, "L") == 0
       && strcmp(chararrVersion, "1") == 0
       /* && strcmp(chararrPart, "1") == 0 */ ){

       MODEL_ID=1;
    }
    else if(strcmp(chararrModel, "L") == 0
       && strcmp(chararrVersion, "3") == 0
       /* && (strcmp(chararrPart, "1") == 0 || strcmp(chararrPart, "2") == 0 || strcmp(chararrPart, "3") == 0) */){

       MODEL_ID=2;
    }
     else if(strcmp(chararrModel, "O") == 0              // O= Otto
         && strcmp(chararrVersion, "1") == 0           // s zavoda
        /*&& (strcmp(chararrPart, "5") == 0 )*/){          // s zavoda

       MODEL_ID=5;                                     //MODEL_ID for OTTO
    }
    else{
       MODEL_ID=9999;
    }

}







void setup(){
    parseSerialNumber();

    Serial.begin(SERIAL_SPEED);
}



void loop(){
      Serial.print(F("ROBBO-"));
      if(MODEL_ID < 10000){
      Serial.write('0');
      }
      if(MODEL_ID < 1000){
      Serial.write('0');
      }
      if(MODEL_ID < 100){
      Serial.write('0');
      }
      if(MODEL_ID < 10){
      Serial.write('0');
      }
      Serial.print(MODEL_ID);
      Serial.write('.');

      static int loopCount = 0;
      if (loopCount < 5) {
        delay(100);
        loopCount++;
      } else {
        delay(1000);
      }
}

