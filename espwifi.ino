#include <cstdint>
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

#include <ArduinoOTA.h>


#define TX_PIN 2 // the pin we are using to TX with
#define RX_PIN 2 // the pin we are using to RX with
#define EN_PIN 0 // the pin we are using to enable TX on the DMX transceiver
#define DMX_PORT = 1;

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
#define BRIDGE 2
#define DEFAULT_MODE 0

#pragma pack(push, 1)

struct StoreStruct {
  char version[4];
  uint16_t Universe;
  uint16_t Address;
  char nodeName[18], longName[64], wifiSSID[40], wifiPass[40];
  char runmode;
} deviceSettings = {
    CONFIG_VERSION,         DEFAULT_UNIVERSE, DEFAULT_ADDRESS, "espArtNetNode",
    "espArtNetNode by Eri", "RamodeART",      "45342523442",   DEFAULT_MODE};

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

    // Serial.printf("Loadconf\n");
    //  Serial.println(deviceSettings.wifiSSID);
    //  Serial.println(deviceSettings.wifiPass);
  } else {
    eepromSave();
    //  Serial.printf("Noconf\n");
    delay(500);
    ESP.eraseConfig();
    delay(500);
    ESP.restart();
  }
}

#define DMX_PIN 2
#define PWM1_PIN 2
#define PWM2_PIN 0
#define NUM_CHANS 3
#define CAP_CHAN NUM_CHANS
#define STB_CHAN NUM_CHANS + 1
#define ARTNET_CUSTOM_CONFIG 0x6017

uint8_t target_dim[NUM_CHANS];
uint8_t current_dim[NUM_CHANS];
uint8_t dmx_frame[512];
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

#define BLACKOUT 2000

unsigned long alive;

void blackout() {
  if (millis() - alive > BLACKOUT)
    target_dim[0] = 0;
}

uint8_t strobe(uint8_t dim) {
  if (stb_dim == 0)
    return dim;
  if ((millis() / ((255 - stb_dim) * 8)) % 2 == 0)
    return 0;
  return dim;
}

void IRAM_ATTR fade() {
  int step;
  uint16_t value;
  for (uint8_t i = 0; i < NUM_CHANS; i++) {
    step = strobe(target_dim[i]) - current_dim[i];
    if (step > 0) {
      step = min(step, step_dim);
    } else {
      step = max(step, -step_dim);
    }
    current_dim[i] += step;
  }
  for (uint8_t i = 0; i < NUM_CHANS - 1; i++) {
    uint16_t value = 65025 - (current_dim[i + 1] * current_dim[0]);
    analogWrite(pins[i], value);
  }
}

void IRAM_ATTR relay() {
  for (uint8_t i = 0; i < NUM_CHANS; i++) {
    if (target_dim[i] < 50) {
      current_dim[i] = HIGH;
    } else {
      current_dim[i] = LOW;
    }
  }
  for (uint8_t i = 1; i < NUM_CHANS; i++) {
    digitalWrite(pins[i - 1], current_dim[i] || current_dim[0]);
  }
}

#define DMXSPEED 250000
#define DMXFORMAT SERIAL_8N2
#define BREAKSPEED 83333
#define BREAKFORMAT SERIAL_8N1

void setup() {
  Serial.begin(115200);
  Serial.println();
  EEPROM.begin(sizeof(deviceSettings));
  eepromLoad();
  alive = millis();

  if (deviceSettings.runmode == BRIDGE) {
    pinMode(DMX_PIN, OUTPUT);
    dmx_frame[0] = 0;
  } else {

    for (uint8_t i = 0; i < NUM_CHANS; i++) {
      current_dim[i] = 0;
      target_dim[i] = 0;
    }
    for (uint8_t i = 1; i < NUM_CHANS; i++) {
      pinMode(pins[i - 1], OUTPUT);

    }

    analogWriteRange(65025);
    analogWriteFreq(2000);
  }
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
  } else if (deviceSettings.runmode == BRIDGE) {
    artnet.setArtDmxCallback(onDMX2);
  }
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname(deviceSettings.nodeName);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd OTA");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();


}

void dmxupdate() {

  digitalWrite(TX_PIN, HIGH);
  Serial1.begin(BREAKSPEED, BREAKFORMAT);
  Serial1.write(0);
  Serial1.flush();
  delay(1);
  Serial1.end();

  // send data
  Serial1.begin(DMXSPEED, DMXFORMAT);
  digitalWrite(TX_PIN, LOW);
  Serial1.write(dmx_frame, 512);
  Serial1.flush();

  delay(1);
  Serial1.end();
}

void onDMX2(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t *data,
            IPAddress remoteIP) {

  if (universe == deviceSettings.Universe) {
    memcpy(dmx_frame + 1, data, length);
    target_dim[0] = 1;
  }
}

void onDMX1(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t *data,
            IPAddress remoteIP) {
  if (universe == deviceSettings.Universe) {
    for (uint8_t i = 0; i < NUM_CHANS; i++) {
      target_dim[i] = data[deviceSettings.Address - 1 + i];
    }
  }
}

void onDMX0(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t *data,
            IPAddress remoteIP) {
  if (universe == deviceSettings.Universe) {
    for (uint8_t i = 0; i < NUM_CHANS; i++) {
      target_dim[i] = data[deviceSettings.Address - 1 + i];
    }
    step_dim =
        ceil(255.0 *
             exp(-data[deviceSettings.Address - 1 + CAP_CHAN] /
                 46.0));
    stb_dim = data[deviceSettings.Address - 1 + STB_CHAN];
  }
}

uint8_t artnetPacket[MAX_BUFFER_ARTNET];

void onCustomConfig() {
  // Serial.println("onCFG");
  if (artnetPacket[12 + 0] == CONFIG_VERSION[0] &&
      artnetPacket[12 + 1] == CONFIG_VERSION[1] &&
      artnetPacket[12 + 2] == CONFIG_VERSION[2]) {
    //  Serial.println("CFG");
    memcpy(&deviceSettings, artnetPacket + 12, sizeof(deviceSettings));
    eepromSave();
    delay(500);
    ESP.restart();
  }
}

void loop() {
  ArduinoOTA.handle();
  uint16_t r = artnet.read();
  uint16_t l;
  if (r == ART_POLL) {
  }
  if (r == ART_DMX) {
    alive = millis();
  }
  if (r == ARTNET_CUSTOM_CONFIG) {
    l = artnet.getPacket(artnetPacket);
    onCustomConfig();
  }
  if (deviceSettings.runmode == 2) {
    if (target_dim[0] == 1)
      dmxupdate();
  } else {
    ISR_Timer.run();
  };
  blackout();
}
