#define _TIMERINTERRUPT_LOGLEVEL_ 0

#include <Artnet.h>

#include <ESP8266TimerInterrupt.h>
#include <ESP8266_ISR_Timer.h>
#include <ESP8266_ISR_Timer.hpp>

#include <ArduinoWiFiServer.h>
#include <BearSSLHelpers.h>
#include <CertStoreBearSSL.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiGratuitous.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiClientSecureBearSSL.h>
#include <WiFiServer.h>
#include <WiFiServerSecure.h>
#include <WiFiServerSecureBearSSL.h>
#include <WiFiUdp.h>

#ifndef UTIL_H
#define UTIL_H

#define htons(x) (((x) << 8 & 0xFF00) | ((x) >> 8 & 0x00FF))
#define ntohs(x) htons(x)

#define htonl(x)                                                               \
  (((x) << 24 & 0xFF000000UL) | ((x) << 8 & 0x00FF0000UL) |                    \
   ((x) >> 8 & 0x0000FF00UL) | ((x) >> 24 & 0x000000FFUL))
#define ntohl(x) htonl(x)

#endif

WiFiUDP Udp;
// Init ESP8266 timer 1
// ESP8266Timer ITimer;

// Init ESP8266_ISR_Timer
ESP8266_ISR_Timer ISR_Timer;
#define HW_TIMER_INTERVAL_US 10000L

// void IRAM_ATTR TimerHandler()
//{
// ISR_Timer.run();
//}

unsigned int localUdpPort = 6454;
char incomingPacket[572];
Artnet artnet;

#define DEFAULT_UNIVERSE 0
#define DEFAULT_ADDRESS 0
#define CONFIG_VERSION "ER0"
#define CONFIG_START 0

#define DIMMER 0
#define SMOKER 1

#pragma pack(push, 1)

struct StoreStruct {
  char version[4];
  uint16_t Universe;
  uint16_t Address;
  char nodeName[18], longName[64], wifiSSID[40], wifiPass[40];
  char runmode;
} deviceSettings = {
    CONFIG_VERSION,         DEFAULT_UNIVERSE, DEFAULT_ADDRESS, "espArtNetNode",
    "espArtNetNode by Eri", "RamodeART",      "45342523442",   0};

#pragma pack(pop)

void eepromSave() {
  for (uint16_t t = 0; t < sizeof(deviceSettings); t++)
    EEPROM.write(CONFIG_START + t, *((char *)&deviceSettings + t));

  EEPROM.commit();
}

void eepromLoad() {

  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2]) {

    for (uint16_t t = 0; t < sizeof(deviceSettings); t++)
      *((char *)&deviceSettings + t) = EEPROM.read(CONFIG_START + t);

    Serial.printf("Loadconf\n");
    Serial.println(deviceSettings.wifiSSID);
    Serial.println(deviceSettings.wifiPass);
  } else {
    eepromSave();
    Serial.printf("Noconf\n");
    delay(500);
    ESP.eraseConfig();
    delay(500);
    ESP.restart();
  }
}

#define PWM1_PIN 2
#define PWM2_PIN 0
#define NUM_CHANS 2
#define CAP_CHAN NUM_CHANS
#define STB_CHAN NUM_CHANS + 1
#define ARTNET_CUSTOM_CONFIG 0x6017

uint8_t target_dim[NUM_CHANS];
uint8_t current_dim[NUM_CHANS];
int step_dim = 255;
int stb_dim = 0;
uint8_t pins[] = {PWM1_PIN, PWM2_PIN};

void IRAM_ATTR beacon() {

  // Serial.println("dims:");
  // Serial.println(255-current_dim[0]);
  // Serial.println(255-current_dim[1]);
  // Serial.println(255-target_dim[0]);
  // Serial.println(255-target_dim[1]);
  // Serial.println(step_dim);
}

uint8_t strobe(uint8_t dim) {
  if (stb_dim == 0)
    return dim;
  if ((millis() / ((255 - stb_dim) * 8)) % 2 == 0)
    return 255;
  return dim;
}

void IRAM_ATTR fade() {
  int step;
  for (uint8_t i = 0; i < NUM_CHANS; i++) {
    step = strobe(target_dim[i]) - current_dim[i];
    if (step > 0) {
      step = min(step, step_dim);
    } else {
      step = max(step, -step_dim);
    }
    current_dim[i] += step;
    analogWrite(pins[i], current_dim[i]);
  }
}
void IRAM_ATTR relay() {
  for (uint8_t i = 0; i < NUM_CHANS; i++) {
    if (target_dim[i] > 205) {
      current_dim[i] = HIGH;
    } else {
      current_dim[i] = LOW;
    }
    digitalWrite(pins[i], current_dim[i]);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  EEPROM.begin(sizeof(deviceSettings));
  eepromLoad();

  for (uint8_t i = 0; i < NUM_CHANS; i++) {
    current_dim[i] = 255;
    target_dim[i] = 255;
    pinMode(pins[i], OUTPUT);
  }

  analogWriteRange(255);
  analogWriteFreq(1000);

  WiFi.begin(deviceSettings.wifiSSID, deviceSettings.wifiPass);

  artnet.begin();

  artnet.setBroadcastAuto(WiFi.localIP(), WiFi.subnetMask());
  artnet.setName(deviceSettings.nodeName, deviceSettings.longName);

  if (deviceSettings.runmode == DIMMER) {
    artnet.setArtDmxCallback(onDMX0);
    // if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_US, TimerHandler))
    // {} ISR_Timer.setInterval(1000, beacon);
    fade();
    ISR_Timer.setInterval(10, fade);

    //}
  } else if (deviceSettings.runmode == SMOKER) {
    artnet.setArtDmxCallback(onDMX1);
    relay();
    ISR_Timer.setInterval(10, relay);
  }
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
}

void onDMX1(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t *data,
            IPAddress remoteIP) {
  if (universe == deviceSettings.Universe) {

    for (uint8_t i = 0; i < NUM_CHANS; i++) {
      target_dim[i] = 255 - data[deviceSettings.Address + i];
    }
  }
}

void onDMX0(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t *data,
            IPAddress remoteIP) {
  if (universe == deviceSettings.Universe) {
    for (uint8_t i = 0; i < NUM_CHANS; i++) {
      target_dim[i] = 255 - data[deviceSettings.Address + i];
    }
    step_dim =
        ceil(255.0 *
             exp(-data[deviceSettings.Address + CAP_CHAN] /
                 46.0)); // 1 + 255 - data[deviceSettings.Address + CAP_CHAN];
    stb_dim = data[deviceSettings.Address + STB_CHAN];
  }
}

uint8_t artnetPacket[MAX_BUFFER_ARTNET];

void onCustomConfig() {
  Serial.println("onCFG");
  if (artnetPacket[12 + 0] == CONFIG_VERSION[0] &&
      artnetPacket[12 + 1] == CONFIG_VERSION[1] &&
      artnetPacket[12 + 2] == CONFIG_VERSION[2]) {
    Serial.println("CFG");
    memcpy(&deviceSettings, artnetPacket + 12, sizeof(deviceSettings));
    eepromSave();
    delay(500);
    ESP.restart();
  }
}

void loop() {
  uint16_t r = artnet.read();
  uint16_t l;
  if (r == ART_POLL) {
  }
  if (r == ART_DMX) {
  }
  if (r == ARTNET_CUSTOM_CONFIG) {
    l = artnet.getPacket(artnetPacket);
    onCustomConfig();
  }
  ISR_Timer.run();
}
