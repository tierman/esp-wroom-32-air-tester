#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_CCS811.h>
#include <Wire.h>
#include <ClosedCube_HDC1080.h>
#include "ccs811.h"
#include "PMserial.h"
#include <SoftwareSerial.h>

#define ONBOARD_LED  2
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//Adafruit_CCS811 gasSensor;
ClosedCube_HDC1080 tempHumiditiSensor;
//CCS811 ccs811(-1); // nWAKE on GPIO_NUM_26
//SoftwareSerial Serial11(GPIO_NUM_17, GPIO_NUM_16);
SerialPM pms(PMSA003, GPIO_NUM_17, GPIO_NUM_27);

void initDisplay();
void initGasSensor();
void blinkOnBoardLed(int count);
void initTempHumiditiSensor();
void printRegister(HDC1080_Registers reg);
void ccs811Read();
void pmsa003Read();
void printTandRH(HDC1080_MeasurementResolution humidity, HDC1080_MeasurementResolution temperature);

void setup() {
  Serial.begin(9600);

  pinMode(GPIO_NUM_16,INPUT);
  pinMode(GPIO_NUM_17,OUTPUT);

  pinMode(ONBOARD_LED,OUTPUT);
  pinMode(GPIO_NUM_26,OUTPUT);// nWAKE on GPIO_NUM_26
  digitalWrite(GPIO_NUM_26, LOW);
  pinMode(GPIO_NUM_32,OUTPUT);
  digitalWrite(GPIO_NUM_32, LOW);
  pinMode(GPIO_NUM_12,OUTPUT);
  digitalWrite(GPIO_NUM_12, LOW);

  WiFi.mode(WIFI_STA);
  //initGasSensor();
  initTempHumiditiSensor();
  initDisplay();
  pms.init();
}
 
void loop() {
  blinkOnBoardLed(1);
  
  display.clearDisplay();
  display.setCursor(0, 0);


/*  if (gasSensor.available()) {
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
*/

  display.print("T=");
  display.print(tempHumiditiSensor.readTemperature());
  display.print("C, RH=");
  display.print(tempHumiditiSensor.readHumidity());
  display.print("%\n");
  

  Serial.print("\n\ntemp: ");
  Serial.print(tempHumiditiSensor.readTemperature()); 
  Serial.print("C, RH=");
  Serial.print(tempHumiditiSensor.readHumidity());
  Serial.print("%");
  Serial.print("\n\n");
  
//  printRegister(tempHumiditiSensor.readRegister());

  ccs811Read();

  pmsa003Read();

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
           // Serial.println(chBuffer);
        }
    }
 
  Serial.println('\n');
  
  display.display();
  
  //digitalWrite(GPIO_NUM_26, HIGH);

  delay(2000);
}

void pmsa003Read() {
  pms.read();
  if (pms)
  { // successfull read
      // print formatted results
    display.printf("PM1.0 %2d, \nPM2.5 %2d, \nPM10 %2d [ug/m3]\n",
                  pms.pm01, pms.pm25, pms.pm10);

    Serial.printf("PM1.0 %2d, PM2.5 %2d, PM10 %2d [ug/m3]\n",
                  pms.pm01, pms.pm25, pms.pm10);

    if (pms.has_number_concentration()) {
      Serial.printf("N0.3 %4d, N0.5 %3d, N1.0 %2d, N2.5 %2d, N5.0 %2d, N10 %2d [#/100cc]\n",
                    pms.n0p3, pms.n0p5, pms.n1p0, pms.n2p5, pms.n5p0, pms.n10p0);
    }

    if (pms.has_temperature_humidity() || pms.has_formaldehyde()) {
      Serial.printf("%5.1f °C, %5.1f %%rh, %5.2f mg/m3 HCHO\n",
                    pms.temp, pms.rhum, pms.hcho);
    }
  } else { // something went wrong
    switch (pms.status)
    {
    case pms.OK: // should never come here
      break;     // included to compile without warnings
    case pms.ERROR_TIMEOUT:
      Serial.println(F(PMS_ERROR_TIMEOUT));
      break;
    case pms.ERROR_MSG_UNKNOWN:
      Serial.println(F(PMS_ERROR_MSG_UNKNOWN));
      break;
    case pms.ERROR_MSG_HEADER:
      Serial.println(F(PMS_ERROR_MSG_HEADER));
      break;
    case pms.ERROR_MSG_BODY:
      Serial.println(F(PMS_ERROR_MSG_BODY));
      break;
    case pms.ERROR_MSG_START:
      Serial.println(F(PMS_ERROR_MSG_START));
      break;
    case pms.ERROR_MSG_LENGTH:
      Serial.println(F(PMS_ERROR_MSG_LENGTH));
      break;
    case pms.ERROR_MSG_CKSUM:
      Serial.println(F(PMS_ERROR_MSG_CKSUM));
      break;
    case pms.ERROR_PMS_TYPE:
      Serial.println(F(PMS_ERROR_PMS_TYPE));
      break;
    }
  }
}

void ccs811Read() {
/*
  ccs811.begin();
    // Read
  uint16_t eco2, etvoc, errstat, raw;
  ccs811.read(&eco2,&etvoc,&errstat,&raw); 
  
  // Print measurement results based on status
  if( errstat==CCS811_ERRSTAT_OK ) { 
    Serial.print("CCS811: ");
    Serial.print("eco2=");  Serial.print(eco2);     Serial.print(" ppm  ");
    Serial.print("etvoc="); Serial.print(etvoc);    Serial.print(" ppb  ");
    display.print("eco2=");  display.print(eco2);     display.print(" ppm  ");
    display.print("etvoc="); display.print(etvoc);    display.print(" ppb  ");

    //Serial.print("raw6=");  Serial.print(raw/1024); Serial.print(" uA  "); 
    //Serial.print("raw10="); Serial.print(raw%1024); Serial.print(" ADC  ");
    //Serial.print("R="); Serial.print((1650*1000L/1023)*(raw%1024)/(raw/1024)); Serial.print(" ohm");
    Serial.println();
  } else if( errstat==CCS811_ERRSTAT_OK_NODATA ) {
    Serial.println("CCS811: waiting for (new) data");
  } else if( errstat & CCS811_ERRSTAT_I2CFAIL ) { 
    Serial.println("CCS811: I2C error");
  } else {
    Serial.print("CCS811: errstat="); Serial.print(errstat,HEX); 
    Serial.print("="); Serial.println( ccs811.errstat_str(errstat) ); 
  }*/
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
  Serial.println(F("Display initialized correctly."));
}

void initGasSensor() {
/*
  if(!gasSensor.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }

  // Wait for the sensor to be ready
  while(!gasSensor.available());
  */
  /*
  Serial.println(F("Enable CCS811 - initGasSensor()"));
  // Enable CCS811
  //ccs811.set_i2cdelay(50); // Needed for ESP8266 because it doesn't handle I2C clock stretch correctly
  bool ok= ccs811.begin();
  if ( !ok ) {
    Serial.println("setup: CCS811 begin FAILED");
    blinkOnBoardLed(10);
  }

  // Print CCS811 versions
  Serial.print("setup: hardware    version: "); Serial.println(ccs811.hardware_version(),HEX);
  Serial.print("setup: bootloader  version: "); Serial.println(ccs811.bootloader_version(),HEX);
  Serial.print("setup: application version: "); Serial.println(ccs811.application_version(),HEX);
  */
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
 
  HDC1080_Registers reg = tempHumiditiSensor.readRegister();
 
  reg.SoftwareReset = 0;
	reg.Heater = 0;
	reg.ModeOfAcquisition = 0;
	tempHumiditiSensor.writeRegister(reg);

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

void printRegister(HDC1080_Registers reg) {
	Serial.println("HDC1080 Configuration Register");
	Serial.println("------------------------------");
	
	Serial.print("Software reset bit: ");
	Serial.print(reg.SoftwareReset, BIN);
	Serial.println(" (0=Normal Operation, 1=Software Reset)");

	Serial.print("Heater: ");
	Serial.print(reg.Heater, BIN);
	Serial.println(" (0=Disabled, 1=Enabled)");

	Serial.print("Mode of Acquisition: ");
	Serial.print(reg.ModeOfAcquisition, BIN);
	Serial.println(" (0=T or RH is acquired, 1=T and RH are acquired in sequence, T first)");

	Serial.print("T Measurement Resolution: ");
	Serial.print(reg.TemperatureMeasurementResolution, BIN);
	Serial.println(" (0=14 bit, 1=11 bit)");

	Serial.print("RH Measurement Resolution: ");
	Serial.print(reg.HumidityMeasurementResolution, BIN);
	Serial.println(" (0=14 bit, 01=11 bit, 10=8 bit)");
}

void printTandRH(HDC1080_MeasurementResolution humidity, HDC1080_MeasurementResolution temperature) {
	tempHumiditiSensor.setResolution(humidity, temperature);

	HDC1080_Registers reg = tempHumiditiSensor.readRegister();
	printRegister(reg);

	Serial.print("T=");
	Serial.print(tempHumiditiSensor.readTemperature());
	Serial.print("C, RH=");
	Serial.print(tempHumiditiSensor.readHumidity());
	Serial.println("%");
}