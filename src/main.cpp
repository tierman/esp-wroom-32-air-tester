#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <ClosedCube_HDC1080.h>
#include <ccs811.h>
#include <PMserial.h>
#include <EEPROM.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define ONBOARD_LED  2
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID  "4ac8a682-9736-4e5d-932b-e9b31405049c"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ClosedCube_HDC1080 tempHumiditiSensor;
CCS811 gasSensor(GPIO_NUM_26); // nWAKE on GPIO_NUM_26
SerialPM pms(PMSA003, GPIO_NUM_17, GPIO_NUM_27);
bool deviceConnected = false;
std::string bleCallbackValue;
BLECharacteristic *pCharacteristic;

unsigned long diffTime = 0;
unsigned long savedTime = 0;
unsigned long actualTime = 0;
double temperature = 0.0;
double humidity = 0.0;

void initDisplay();
void initGasSensor();
void initPinout();
void initCommunication();
void initTempHumiditiSensor();
void blinkOnBoardLed(int count);
void readGasSensor();
void pmsa003Read();
void initPmsSensor();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  log_d("Total heap: %d", ESP.getHeapSize());
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Free PSRAM: %d", ESP.getFreePsram());
  log_i("\nSetup air-tester started.\n");
  initPinout();
  initCommunication();
  initGasSensor();
  initTempHumiditiSensor();
  initDisplay();
  initPmsSensor();
  log_i("\nSetup air-tester finished.\n");
}
 
void loop() {
  blinkOnBoardLed(1);

  actualTime = (esp_timer_get_time() / 1000000LL);
  diffTime = actualTime - savedTime;

  if (deviceConnected) {
    Serial.print("BLE connected!");

    pCharacteristic->setValue("aaaa");
    pCharacteristic->notify();
    
    Serial.print("BLE received: ");
    
    for (int i = 0; i < bleCallbackValue.length(); i++) {
      Serial.print(bleCallbackValue[i]);
    }
  }


  display.clearDisplay();
  display.setCursor(0, 0);

  if (diffTime >= 20) {
    temperature = tempHumiditiSensor.readTemperature();
    humidity = tempHumiditiSensor.readHumidity();

    Serial.printf("\nReading temp [savedTime: %2d, diffTime: %2d, actualTime: %2d]\n", savedTime, diffTime, actualTime);
    savedTime = actualTime;    
  }

  display.print("T=");
  display.print(temperature);
  display.print("C, RH=");
  display.print(humidity);
  display.print("%\n");
  

  Serial.print("\n\ntemp: ");
  Serial.print(temperature); 
  Serial.print("C, RH=");
  Serial.print(humidity);
  Serial.print("%");
  Serial.print(" diff time : "); Serial.print(diffTime); Serial.print("\n\n");
  
  readGasSensor();

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
}

void initPmsSensor() {
  pms.init();
}

class ServerCallbacks: public BLEServerCallbacks {
    
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    };
 
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      Serial.println("Test a");    
      bleCallbackValue = pCharacteristic->getValue();
    }

    void onRead(BLECharacteristic *pCharacteristic) {
      Serial.println("Test b");    
      bleCallbackValue = pCharacteristic->getValue();
    }

    void onNotify(BLECharacteristic *pCharacteristic) {
      Serial.println("Test c");    
      bleCallbackValue = pCharacteristic->getValue();
    }
    
};

void initCommunication() {
  Serial.println("Initialize wireless and bluetooth connection...");

  WiFi.mode(WIFI_STA);

  //BLE
  BLEDevice::init("Air-tester");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("Hello World");
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  //pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void initPinout() {
  pinMode(GPIO_NUM_16,INPUT);
  pinMode(GPIO_NUM_17,OUTPUT);

  pinMode(ONBOARD_LED,OUTPUT);

  pinMode(GPIO_NUM_32,OUTPUT);
  digitalWrite(GPIO_NUM_32, LOW);

  pinMode(GPIO_NUM_12,OUTPUT);
  digitalWrite(GPIO_NUM_12, LOW);
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

void readGasSensor() {
  uint16_t eco2, etvoc, errstat, raw;
  gasSensor.read(&eco2,&etvoc,&errstat,&raw); 
  
  // Print measurement results based on status
  if( errstat==CCS811_ERRSTAT_OK ) { 
    Serial.print("CCS811: ");
    Serial.print("eco2=");  Serial.print(eco2);     Serial.print(" ppm  ");
    Serial.print("etvoc="); Serial.print(etvoc);    Serial.print(" ppb  ");
    display.print("eco2= ");  display.print(eco2);     display.print(" ppm \n");
    display.print("etvoc= "); display.print(etvoc);    display.print(" ppb \n");

    Serial.print("raw6=");  Serial.print(raw/1024); Serial.print(" uA  "); 
    Serial.print("raw10="); Serial.print(raw%1024); Serial.print(" ADC  ");
    Serial.print("R="); Serial.print((1650*1000L/1023)*(raw%1024)/(raw/1024)); Serial.print(" ohm");
    Serial.println();
  } else if( errstat==CCS811_ERRSTAT_OK_NODATA ) {
    Serial.println("CCS811: waiting for (new) data");
  } else if( errstat & CCS811_ERRSTAT_I2CFAIL ) { 
    Serial.println("CCS811: I2C error");
  } else {
    Serial.print("CCS811: errstat="); Serial.print(errstat,HEX); 
    Serial.print("="); Serial.println( gasSensor.errstat_str(errstat) ); 
  }
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
  Serial.println("Enable CCS811 (gas sensor) - initGasSensor()");
  // Enable CCS811
  //gasSensor.set_i2cdelay(50); // Needed for ESP8266 because it doesn't handle I2C clock stretch correctly
  bool ok = gasSensor.begin();

  if ( !ok ) {
    Serial.println("setup: CCS811 begin FAILED");
    blinkOnBoardLed(10);
  }

  Serial.print("setup: CCS811 start ");
  ok = gasSensor.start(CCS811_MODE_1SEC);
  if( ok ) Serial.println("ok"); else Serial.println("FAILED");

  // Print CCS811 versions
  Serial.print("setup: hardware    version: "); Serial.println(gasSensor.hardware_version(),HEX);
  Serial.print("setup: bootloader  version: "); Serial.println(gasSensor.bootloader_version(),HEX);
  Serial.print("setup: application version: "); Serial.println(gasSensor.application_version(),HEX);
  
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
}
