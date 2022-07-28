#include <SPI.h>
#include <SD.h>

#define JUXTA_LOG_SIZE              17 // bytes (with header)
#define GAP_DEVICE_NAME_LEN         21 // bytes
//#define JUXTA_LOG_OFFSET_HEADER     0 // 2 bytes
// these are modified because the header is detected/removed
#define JUXTA_LOG_OFFSET_LOGCOUNT   0 // 4 bytes
#define JUXTA_LOG_OFFSET_SCANADDR   4 // 6 bytes
#define JUXTA_LOG_OFFSET_RSSI       10 // 1 bytes
#define JUXTA_LOG_OFFSET_TIME       11 // 4 bytes

const int JUXTA_RESET = 6;
const int JUXTA_DEBUG = 5;

const int FEATHER_SD_CS = 4;
const int FEATHER_LED_R = 13;
const int FEATHER_LED_G = 8;
const int FEATHER_BTN = 10;
const int CARD_DETECT = 7;

int fadeValue = 0;
bool doJuxta = false;
bool fadeDir = true;
bool sdIn = false;

//uint8_t nvsDataBuffer[JUXTA_LOG_SIZE];
const uint8_t flushByte = 0x99;
const uint8_t headerByte = 0xA2;

uint8_t macAddress[GAP_DEVICE_NAME_LEN];
uint8_t macCount = 0;

static String dataString;
static uint32_t logCount = 0;
static uint8_t addr[6];
static uint32_t localTime = 0;
static int8_t rssi = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(JUXTA_RESET, OUTPUT); // active LOW
  pinMode(JUXTA_DEBUG, OUTPUT); // active LOW
  pinMode(FEATHER_LED_R, OUTPUT);
  pinMode(FEATHER_LED_G, OUTPUT);
  pinMode(FEATHER_BTN, INPUT_PULLUP); // active LOW
  pinMode(CARD_DETECT, INPUT_PULLUP); // active LOW

  digitalWrite(JUXTA_RESET, HIGH);
  digitalWrite(JUXTA_DEBUG, HIGH);
  analogWrite(FEATHER_LED_R, 0);
  digitalWrite(FEATHER_LED_G, LOW);

  Serial.begin(115200);
  Serial1.begin(115200);

  Serial.print("Initializing SD card...");
  initSD();
}

bool initSD() {
  delay(100); // debounce
  while (digitalRead(CARD_DETECT) == LOW) {
    analogWrite(FEATHER_LED_R, 0);
    digitalWrite(FEATHER_LED_G, HIGH);
    delay(100);
    analogWrite(FEATHER_LED_R, 255);
    digitalWrite(FEATHER_LED_G, LOW);
    delay(100);
  }
  delay(100);
  if (!SD.begin(FEATHER_SD_CS)) {
    Serial.println("Card failed, or not present");
    return false;
  }
  Serial.println("card initialized.");
  return true;
}

void toggleReset() {
  digitalWrite(JUXTA_RESET, LOW);
  delay(100);
  digitalWrite(JUXTA_RESET, HIGH);
}

void readJuxta() {
  analogWrite(FEATHER_LED_R, 255);
  digitalWrite(FEATHER_LED_G, LOW); // keep LOW until incoming serial data
  Serial.println("Opening file...");
  String filename = "JUXTA.txt"; // !! rename to BLE MAC
  SD.remove(filename);
  File dataFile = SD.open(filename, FILE_WRITE);
  if (dataFile) {
    Serial.println("File open.");
    digitalWrite(JUXTA_DEBUG, LOW);
    delay(100);
    toggleReset();

    // Juxta should be in serial mode now
    unsigned long lastSerialTime = millis();
    unsigned long curTime;
    bool doLoop = true;
    bool isFlushing = true;
    bool isGettingAddr = true;
    bool isGettingHeader = true;
    int headerCount = 0;
    int dataPos = 0;

    while (doLoop) {
      if (Serial1.available()) {
        uint8_t data = Serial1.read();
        if (isFlushing) {
          if (data != flushByte) {
            isFlushing = false; // gets set once per dump
          }
        }

        if (!isFlushing && isGettingAddr) {
          if (macCount < GAP_DEVICE_NAME_LEN) {
            macAddress[macCount] = data;
          } else if (macCount == GAP_DEVICE_NAME_LEN) {
            isGettingAddr = false;
          }
          macCount++;
        }

        if (!isFlushing && !isGettingAddr) {
          if (isGettingHeader) {
            if (data == headerByte) {
              headerCount++;
            }
            if (headerCount == 2) {
              isGettingHeader = false;
            }
          } else { // header done, start building data
            Serial.print(dataPos);
            Serial.print("-");
            Serial.println(data, HEX);

            switch (dataPos) {
              case 0:
              case 1:
              case 2:
              case 3:
                memcpy(&logCount + (dataPos - JUXTA_LOG_OFFSET_LOGCOUNT), &data, sizeof(data));
                dataPos++;
                break;
              case 4:
              case 5:
              case 6:
              case 7:
              case 8:
              case 9:
                addr[dataPos - JUXTA_LOG_OFFSET_SCANADDR] = data;
                dataPos++;
                break;
              case 10:
                rssi = data;
                dataPos++;
                break;
              case 11:
              case 12:
              case 13:
              case 14:
                memcpy(&localTime + (dataPos - JUXTA_LOG_OFFSET_TIME), &data, sizeof(data)); // conflicting with logCount
                if (dataPos == 14) { // done with log
                  dataFile.print(logCount, DEC);

                  dataFile.print(",0x");
                  for (int k = 0; k < 6; k++) {
                    dataFile.print(addr[5 - k], HEX);
                    Serial.print(addr[5 - k], HEX);
                    Serial.println("<<");
                  }
                  dataFile.print(",");
                  dataFile.print(rssi, DEC);
                  dataFile.print(",");
                  dataFile.println(localTime, DEC);

                  isGettingHeader = true;
                  dataPos = 0;
                  headerCount = 0;
                } else {
                  dataPos++;
                }
                break;
            }
          }
        }
        digitalWrite(FEATHER_LED_G, !digitalRead(FEATHER_LED_G)); // toggle
        lastSerialTime = millis();
      }

      curTime = millis();
      if (curTime - lastSerialTime > 1000) {
        doLoop = false; // timeout condition
        Serial.println("Serial1 timeout.");
      }
    }

    dataFile.close();
    Serial.println("File closed.");
  } else {
    Serial.println("Error opening data file.");
  }
  digitalWrite(JUXTA_DEBUG, HIGH);
  analogWrite(FEATHER_LED_R, 0);
  digitalWrite(FEATHER_LED_G, LOW);
  delay(1000); // debounce quick transfers
  //  attachInterrupt(digitalPinToInterrupt(FEATHER_BTN), btnPressed, FALLING);
  Serial.println("Done, returning to loop.");
}

void loop() {
  while (sdIn == false || digitalRead(CARD_DETECT) == LOW) {
    sdIn = initSD();
  }

  // interrupt is being finnicky, just poll
  if (digitalRead(FEATHER_BTN) == LOW) {
    doJuxta = true;
  }

  if (doJuxta) {
    doJuxta = false;
    readJuxta();
  }

  analogWrite(FEATHER_LED_R, fadeValue);
  if (fadeValue == 0x2F) {
    fadeDir = false;
  }
  if (fadeValue == 0x00) {
    fadeDir = true;
  }
  if (fadeDir) {
    ++fadeValue;
  } else {
    --fadeValue;
  }

  delay(20);
}
