#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <sstream>
#include <Crypto.h>
#include <base64.hpp>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "SPI.h"
#include "TFT_22_ILI9225.h"
#define BLOCK_SIZE 32
#define SERVICE_UUID (uint16_t)0x181D
#define CHARACTERISTIC_UUID_RX "78604f25-789e-432e-b949-6fb2306fd5d7"
#define CHARACTERISTIC_UUID_SSID "98a8d501-07ab-42a9-94e1-d590839bf71b"
#define CHARACTERISTIC_UUID_PASS "2b9310ca-d12c-4940-a436-c33e32be84c7"
#define CHARACTERISTIC_IV "ef70f86e-a5a4-499a-81e4-d8edffcd3a7e"
#define CHARACTERISTIC_DATA "a8e6a804-216b-4dd8-90ff-a230226b42c1"
#define TFT_RST 26  // IO 26
#define TFT_RS  25  // IO 25
#define TFT_CLK 14  // HSPI-SCK
#define TFT_SDI 13  // HSPI-MOSI
#define TFT_CS  15  // HSPI-SS0
#define TFT_LED 0   // 0 if wired to +5V directly
SPIClass hspi(HSPI);
#define TFT_BRIGHTNESS 200 // Initial brightness of TFT backlight (optional)
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED, TFT_BRIGHTNESS);

const char* ssid     = "-";
std::string eses;
const char* password = "-";
std::string pass;
const String server = "159.89.204.122";
const String port = "80";
const int LED = 32;
bool deviceConnected = false;
std::string value = "0";
int choose = 0;

const int HX711_dout = 21; //mcu > HX711 dout pin
const int HX711_sck = 18; //mcu > HX711 sck pin
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_calVal_eepromAdress = 0;
long t;
HTTPClient http;

typedef uint8_t byte;
uint32_t Ka;
uint8_t iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
char encoded_iv[64];
byte shaResult[SHA256_SIZE];
const uint32_t prime = 2147483647;
const uint32_t generator = 16807;
int count = 1;
uint32_t B = 0;
float weight;
uint32_t a;
uint32_t A;
BLECharacteristic *pWeightCharacteristic;
BLECharacteristic *pIVCharacteristic;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      digitalWrite(LED, HIGH); 
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class WriteCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pWriteCharacteristic) {
    std::string value = pWriteCharacteristic->getValue();
    if (value.length() > 0) {
      for (int i = 0; i < value.length(); i++) {
        Serial.print(value[i]);
      }
      Serial.println();
      if (value == "1") {
        choose = 1;
      } 
      else if (value == "2") {
        choose = 2;
      }
    }
  }
};

class SSIDCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pSSIDCharacteristic) {
    eses = pSSIDCharacteristic->getValue();
    ssid = eses.c_str();
    Serial.println(ssid);
  }
};


class PASSCallbaccks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pPassCharacteristic) {
    pass = pPassCharacteristic->getValue();
    password = pass.c_str();
    Serial.println(password);
  }
};


void ble_connect()
{
    BLEDevice::init("Weight Device");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pEnvironment = pServer->createService(SERVICE_UUID);
    pWeightCharacteristic = pEnvironment->createCharacteristic(
        CHARACTERISTIC_DATA,  
        BLECharacteristic::PROPERTY_NOTIFY
    );
    
    pWeightCharacteristic->addDescriptor(new BLE2902());
    pIVCharacteristic = pEnvironment->createCharacteristic(
      CHARACTERISTIC_IV,
      BLECharacteristic::PROPERTY_READ
    );

    pIVCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pWriteCharacteristic = pEnvironment->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE
    );//bikin characteristics

    BLECharacteristic *pSSIDCharacteristic = pEnvironment->createCharacteristic(
      CHARACTERISTIC_UUID_SSID,
      BLECharacteristic::PROPERTY_WRITE
    );

    BLECharacteristic *pPassCharacteristic = pEnvironment->createCharacteristic(
      CHARACTERISTIC_UUID_PASS,
      BLECharacteristic::PROPERTY_WRITE
    );

    pWriteCharacteristic->addDescriptor(new BLE2902());    
    pWriteCharacteristic->setCallbacks(new WriteCallbacks());

    pSSIDCharacteristic->addDescriptor(new BLE2902());
    pSSIDCharacteristic->setCallbacks(new SSIDCallbacks());

    pPassCharacteristic->addDescriptor(new BLE2902());
    pPassCharacteristic->setCallbacks(new PASSCallbaccks());

    pEnvironment->start();
    pServer->getAdvertising()->start();
}
void wifi_connect() 
{
    tft.setOrientation(3);
    tft.drawRectangle(0, 0, tft.maxX() - 1, tft.maxY() - 1, COLOR_WHITE);
    tft.setFont(Terminal12x16);
    tft.drawText(30, 80, "Connecting to" );
    tft.drawText(30, 100, String(ssid));
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
    tft.setOrientation(3);
    tft.drawRectangle(0, 0, tft.maxX() - 1, tft.maxY() - 1, COLOR_WHITE);
    tft.setFont(Terminal12x16);
    tft.drawText(38, 80, "WIFI Connected" );
    tft.drawText(48, 100, "IP address" );
    tft.drawText(48, 120, String (WiFi.localIP()));
    delay(500);
}

void makeIV() 
{
  RNG::fill(iv, 16);
  int iv_size = sizeof(iv);
  encode_base64(iv, iv_size, (unsigned char*)encoded_iv);
}

uint32_t keyGen() 
{
  return random(1, prime);
}
uint32_t mul_mod(uint32_t a, uint32_t b, uint32_t m)
{
  uint32_t result = 0; //variable to store the result
  uint32_t runningCount = b % m; //holds the value of b*2^i
  for (int i = 0 ; i < 32 ; i++) {
    if (i > 0) runningCount = (runningCount << 1) % m;
    if (bitRead(a, i)) {
      result = (result % m + runningCount % m) % m;
    }
  }
  return result;
}
uint32_t pow_mod(uint32_t b, uint32_t e, uint32_t m)
{
  uint32_t r;  // result of this function
  uint32_t pow;
  uint32_t e_i = e;
  uint8_t i;
  if ( b == 0 || m == 0 ) {
    return 0;
  }
  if ( e == 0 ) {
    return 1;
  }
  b = b % m;
  pow = b;
  r = 1;
  while ( e_i ) {
    if ( e_i & 1 ) {
      r = mul_mod(r, pow, m); //(r * pow) % m;
    }
    pow = mul_mod(pow, pow, m); //(pow * pow) % m;
    e_i = e_i >> 1;
    i++;
  }
  return r;
}

void bufferSize(char* text, int &length)
{
  int i = strlen(text);
  int buf = round(i / BLOCK_SIZE) * BLOCK_SIZE;
  length = (buf <= i) ? buf + BLOCK_SIZE : length = buf;
}

void encrypt(char* plain_text, char* output, int length)
{
  int size = strlen(plain_text);
  AES aesEncryptor(shaResult, iv, AES::AES_MODE_256, AES::CIPHER_ENCRYPT);
  int encryptPadSize = aesEncryptor.calcSizeAndPad(size);
  byte enciphered[encryptPadSize];
  aesEncryptor.process((uint8_t*)plain_text, enciphered, size);
  int encrypted_size = sizeof(enciphered);
  char encoded[encrypted_size];
  encode_base64(enciphered, encrypted_size, (unsigned char*)encoded);
  strcpy(output, encoded);
}

void decrypt(char* enciphered, char* output, int length)
{
  length = length + 1; //re-adjust
  char decoded[length];
  decode_base64((unsigned char*)enciphered, (unsigned char*)decoded);
  bufferSize(enciphered, length);
  byte deciphered[length];
  AES aesDecryptor(shaResult, iv, AES::AES_MODE_256, AES::CIPHER_DECRYPT);
  aesDecryptor.process((uint8_t*)decoded, deciphered, length);
  strcpy(output, (char*)deciphered);
}

void hashing(char* payload)
{
  SHA256 hasher;
  hasher.doUpdate(payload, strlen(payload));
  hasher.doFinal(shaResult);
}

void postData(char* data) 
{
  String url = "http://" + server + ":" + port + "/input/ambilberat"; //diserver namanya apa?
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  String weight = data;
  Serial.println(weight);
  int httpResponseCode = http.POST("{\"berat\":\"" + weight + "\"}");
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println(httpResponseCode);
    Serial.println(response);
  } else {
    Serial.print("Pengiriman metode POST error: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}

void sendParam(uint32_t pub) 
{
  String url = "http://" + server + ":" + port + "/postkey";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  String pub_key = String(pub);
  
  //int iv_size = sizeof(iv);
  //char encoded_iv[64];
  //encode_base64(iv, iv_size, (unsigned char*)encoded_iv);
  
  String iv = String(encoded_iv);
  Serial.println(iv);
  int httpResponseCode = http.POST("{\"pub_key\":\"" + pub_key + "\", \"iv\":\"" + iv + "\"}");
 
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println(httpResponseCode);
    Serial.println(response);
  } else {
    Serial.print("Pengiriman metode POST error: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}

void getParam() 
{
  String url = "http://" + server + ":" + port + "/pk";
  http.begin(url);
  int httpCode = http.GET();
  if (httpCode > 0) {
    String payload = http.getString();
    Serial.println(httpCode);
    Serial.println(payload);
    B = payload.toInt();
  } else {
    Serial.print("HTTP GET error ");
    Serial.println(httpCode);
  }
  http.end();
}

void intro() 
{
    tft.clear();
    Serial.println("");
    tft.setOrientation(3);
    tft.drawRectangle(0, 0, tft.maxX() - 1, tft.maxY() - 1, COLOR_WHITE);
    tft.setFont(Terminal12x16);
    tft.drawText(38, 80, "Telemedicine " );
    delay(5000);
}
void tampilWeight() 
{
    tft.clear();
    tft.setOrientation(3);
    tft.drawRectangle(0, 0, tft.maxX() - 1, tft.maxY() - 1, COLOR_WHITE);
    tft.setFont(Terminal12x16);
    tft.drawText(68, 80, String(weight));
    tft.drawText(150, 80, "Kg");
}

void makeParam() 
{
  
  makeIV();
  //RNG::fill(iv, 16);
  //memulai pertukaran kunci dengan diffie hellman
  a = keyGen();
  A = pow_mod(generator, a, prime);
  Serial.print("Shared index is: ");
  Serial.println(A);
  getParam();
  Serial.print("Received shared index is: ");
  Serial.println(B);
  Ka = pow_mod(B, a, prime);
  Serial.print("Shared Key A: ");
  Serial.println(Ka);
  char buffer[16];
  char *payload = buffer;
  sprintf(buffer, "%lu", Ka);
  hashing(payload);
  
  Serial.print("Hash: ");
  for (byte i = 0; i < 32; i++) {
      if (shaResult[i]<0x10) { Serial.print('0'); }
      Serial.print(shaResult[i], HEX);
  }
  Serial.print("\n");
  Serial.println("");
}

void setup() {
  hspi.begin();
  tft.begin(hspi);
  Serial.begin(115200);
  pinMode (LED, OUTPUT);
  pinMode (4, INPUT);

 intro();
  
  ble_connect();
  
  while (choose == 0) {
    delay(500);
    Serial.print(".");
  }
  if (deviceConnected) {
    if (choose == 1) {
    Serial.println("Lewat Bluetooth");
    tft.setOrientation(3);
    tft.drawRectangle(0, 0, tft.maxX() - 1, tft.maxY() - 1, COLOR_WHITE);
    tft.setFont(Terminal12x16);
    tft.drawText(38, 80, "Mode Bluetooth " );
    delay(2000);
    makeIV();
    pIVCharacteristic->setValue(encoded_iv);
    }      
    else if(choose == 2) {
      Serial.println("Lewat Wifi");
      //tft.clear();
      tft.setOrientation(3);
      tft.drawRectangle(0, 0, tft.maxX() - 1, tft.maxY() - 1, COLOR_WHITE);
      tft.setFont(Terminal12x16);
      tft.drawText(58, 80, "Mode Wi-Fi" );
      //tft.drawText(38, 100, "Masukkan SSID dan Pass" );
      //tft.clear();
      //delay(2000);
      while (ssid == "-" || password == "-") {
        Serial.println(ssid);
        Serial.println(password);
        delay(500);
      }
      Serial.println("waktu mulai");
      WiFi.mode(WIFI_STA);
      wifi_connect();
      makeParam();
      sendParam(A);

      Serial.println("wakut selesai wifi init ");
    } 
  }
  
float calibrationValue; 
calibrationValue = 22.50; 
#if defined(ESP8266) || defined(ESP32)
#endif

  LoadCell.begin();
  long stabilizingtime = 2000; 
  boolean _tare = true; 
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    LoadCell.setCalFactor(calibrationValue); 
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  Serial.print("Calibration value: ");
  Serial.println(LoadCell.getCalFactor());
  Serial.print("HX711 measured conversion time ms: ");
  Serial.println(LoadCell.getConversionTime());
  Serial.print("HX711 measured sampling rate HZ: ");
  Serial.println(LoadCell.getSPS());
  Serial.print("HX711 measured settlingtime ms: ");
  Serial.println(LoadCell.getSettlingTime());
  Serial.println("Note that the settling time may increase significantly if you use delay() in your sketch!");
  if (LoadCell.getSPS() < 7) {
    Serial.println("!!Sampling rate is lower than specification, check MCU>HX711 wiring and pin designations");
  }
  else if (LoadCell.getSPS() > 100) {
    Serial.println("!!Sampling rate is higher than specification, check MCU>HX711 wiring and pin designations");
  }
}

void loop() {
  char buf[16];
  
  static boolean newDataReady = 0;
  const int serialPrintInterval = 500;

  if (LoadCell.update()) newDataReady = true;
  if (newDataReady) {
      weight = LoadCell.getData()/1000;
    if (millis() > t + serialPrintInterval) {
      Serial.print("Load_cell output val:  ");
      Serial.print(weight);
      Serial.println(" Kg ");
      t = millis();
      newDataReady = 0;
    }

  char *plain_text = gcvt(weight, 4, buf);
  
  tampilWeight();

  Serial.print("Proses ke: ");
  Serial.println(count);
  
  // encrypt
  int length = 0;
  bufferSize(plain_text, length);
  char encrypted[length];
  encrypt(plain_text, encrypted, length);
  
  Serial.println("");
  Serial.print("Encrypted: ");
  Serial.println(encrypted);
  
  if (deviceConnected) {
      if (choose == 1) {
        char data[8];
        dtostrf(weight, 2, 2, data);
        pWeightCharacteristic->setValue(data);
        pWeightCharacteristic->notify(); 
      }
      else if(choose == 2) {
        postData(encrypted);
        
      }
  }
  
  length = strlen(encrypted);
  char decrypted[length];
  decrypt(encrypted, decrypted, length);
  Serial.print("Decrypted: ");
  Serial.println(decrypted);
  
  int i;
  Serial.print("key: ");
  for ( i = 0; i < BLOCK_SIZE; i++ ) {
    printf( "%02X", shaResult[i] );
  }
  printf( "\n" );
  Serial.print("iv: ");
  for ( i = 0; i < 16; i++ ) {
    printf( "%02X", iv[i] );
  }
  printf( "\n" );
  printf( "\n" );
  
  count++;
  //delay(500);
}
}
