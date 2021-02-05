#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Adafruit_CCS811.h>
#include <ClosedCube_HDC1080.h>

#define ONBOARD_LED  2
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_CCS811 gasSensor;
ClosedCube_HDC1080 tempHumiditiSensor;

void initDisplay();
void initGasSensor();
void blinkOnBoardLed(int count);
void initTempHumiditiSensor();

void setup() {
  Serial.begin(9600);

  pinMode(ONBOARD_LED,OUTPUT);
  WiFi.mode(WIFI_STA);
  initGasSensor();
  initTempHumiditiSensor();
  initDisplay();
}
 
void loop() {
  blinkOnBoardLed(1);
  
  display.clearDisplay();
  display.setCursor(0, 0);

  if (gasSensor.available()) {
    float temp = gasSensor.calculateTemperature();
    if (gasSensor.readData()) {
      Serial.print("CO2: ");
      Serial.print(gasSensor.geteCO2());
      Serial.print("ppm, TVOC: ");
      Serial.print(gasSensor.getTVOC());
      Serial.print("ppb Temp:");
      Serial.println(temp);
    } else {
      Serial.println("ERROR!");
    }
  }

  //tempHumiditiSensor.heatUp(2);

  display.print("T=");
  display.print(tempHumiditiSensor.readTemperature());
  display.print("C, RH=");
  display.print(tempHumiditiSensor.readHumidity());
  display.print("%");
  display.display();

  int nNetworkCount = WiFi.scanNetworks();

  if(nNetworkCount == 0) 
    {
        // No networks found.
        Serial.println("0 networks found.");
    } 
    else 
    {
 
        char    chBuffer[128];
        char    chEncryption[64];
        char    chRSSI[64];
        char    chSSID[64];
 
        sprintf(chBuffer, "%d networks found:", nNetworkCount);
        Serial.println(chBuffer);
 
        for(int nNetwork = 0; nNetwork < nNetworkCount; nNetwork ++) 
        {
            // Obtain ssid for this network.
            WiFi.SSID(nNetwork).toCharArray(chSSID, 64);
            sprintf(chRSSI, "(%d)", WiFi.RSSI(nNetwork));
            sprintf(chEncryption, "%s", WiFi.encryptionType(nNetwork) == WIFI_AUTH_OPEN ? " ": "*");
            sprintf(chBuffer, "%d: %s %s %s", nNetwork + 1, chSSID, chRSSI, chEncryption);
            Serial.println(chBuffer);
        }
    }
 
  Serial.println('\n');
  
  delay(2000);
}

void initDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64, 0x3C for 128x32
    Serial.println(F("Display failed to start!"));
    blinkOnBoardLed(5);
  }

  display.display();
  display.setTextSize(1);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text

  // Clear the buffer
  display.clearDisplay();
}

void initGasSensor() {
  if (!gasSensor.begin()){
    Serial.println("Failed to start gas sensor!");
    blinkOnBoardLed(10);
  }

  while (!gasSensor.available());
  
  //float temp = gasSensor.calculateTemperature();
  //gasSensor.setTempOffset(temp - 25.0);
}

void blinkOnBoardLed(int count) {
  for (int x = count; x > 0; x--) {
    digitalWrite(ONBOARD_LED, HIGH);
    delay(500);
    digitalWrite(ONBOARD_LED, LOW);
    delay(500);
  }
}

void initTempHumiditiSensor() {
  tempHumiditiSensor.begin(0x40);
 
  Serial.print("Manufacturer ID=0x");
  Serial.println(tempHumiditiSensor.readManufacturerId(), HEX); // 0x5449 ID of Texas Instruments
  Serial.print("Device ID=0x");
  Serial.println(tempHumiditiSensor.readDeviceId(), HEX); // 0x1050 ID of the device
 
//  Serial.print("Device Serial Number=");
//  HDC1080_SerialNumber sernum = tempHumiditiSensor.readSerialNumber();
//  char format[12];
//  sprintf(format, "%02X-%04X-%04X", sernum.serialFirst, sernum.serialMid, sernum.serialLast);
//  Serial.println(format);
}