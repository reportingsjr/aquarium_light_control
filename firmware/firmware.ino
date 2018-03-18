/*
 * Copyright (c) 2015, Majenko Technologies
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * * Neither the name of Majenko Technologies nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ESP8266WiFi.h>
#include "wifiPassword.h"
// for MQTT
#include <PubSubClient.h>
// for OTA updates
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "Time.h"
#include "Timezone.h"

#include "updateTime.h"

#include <OneWire.h>
#include <DallasTemperature.h>

TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};  //UTC - 4 hours
TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};   //UTC - 5 hours
Timezone usEastern(usEDT, usEST);


const PROGMEM char*     MQTT_CLIENT_ID    = "aquarium_light";
const PROGMEM char*     MQTT_SERVER_IP    = "192.168.0.22";
const PROGMEM uint16_t  MQTT_SERVER_PORT  = 1883;

// MQTT: topics
// auto state
const PROGMEM char*     MQTT_AUTO_LIGHT_STATE_TOPIC                = "aquariumlight/auto";
const PROGMEM char*     MQTT_AUTO_LIGHT_COMMAND_TOPIC              = "aquariumlight/auto/switch";

// auto brightness
const PROGMEM char*     MQTT_AUTO_LIGHT_BRIGHTNESS_STATE_TOPIC     = "aquariumlight/auto/brightness";
const PROGMEM char*     MQTT_AUTO_LIGHT_BRIGHTNESS_COMMAND_TOPIC   = "aquariumlight/auto/brightness/set";

// manual state
const PROGMEM char*     MQTT_MANUAL_LIGHT_STATE_TOPIC                = "aquariumlight/manual";
const PROGMEM char*     MQTT_MANUAL_LIGHT_COMMAND_TOPIC              = "aquariumlight/manual/switch";

// manual brightness
const PROGMEM char*     MQTT_MANUAL_LIGHT_BRIGHTNESS_STATE_TOPIC     = "aquariumlight/manual/brightness";
const PROGMEM char*     MQTT_MANUAL_LIGHT_BRIGHTNESS_COMMAND_TOPIC   = "aquariumlight/manual/brightness/set";

// payloads by default (on/off)
const PROGMEM char*     LIGHT_ON          = "ON";
const PROGMEM char*     LIGHT_OFF         = "OFF";

enum current_state_enum {
  on_auto,
  off_auto,
  on_ha,
  off_ha
};

current_state_enum m_current_state = off_auto;
current_state_enum m_current_auto_state = off_auto;
current_state_enum m_current_manual_state = off_ha;

int m_light_brightness = 0;
int m_max_light_brightness = 1023;

int pwmIntervals = 180; // three hour fade on and off
float R = (pwmIntervals * log10(2))/(log10(m_max_light_brightness));

// pin used for the led (PWM)
const PROGMEM uint8_t LIGHT_PIN = D1;

const uint8_t MSG_BUFFER_SIZE = 20;
char m_msg_buffer[MSG_BUFFER_SIZE];

WiFiClient wifiClient;
PubSubClient client(wifiClient);

// temperature sensor on pin D2
OneWire oneWire(D2);
DallasTemperature DS18B20(&oneWire);

// function called to adapt the brightness and the state of the led
void setLightState() {
    analogWrite(LIGHT_PIN, m_light_brightness);
    Serial.print("INFO: Brightness: ");
    Serial.println(m_light_brightness);
}

// function called to publish the state of the led (on/off)
void publishLightState() {
    if(m_current_auto_state == on_auto) {
      client.publish(MQTT_AUTO_LIGHT_STATE_TOPIC, LIGHT_ON, true);
    } else {
      client.publish(MQTT_AUTO_LIGHT_STATE_TOPIC, LIGHT_OFF, true);
    }

    if(m_current_manual_state == on_ha) {
      client.publish(MQTT_MANUAL_LIGHT_STATE_TOPIC, LIGHT_ON, true);
    } else {
      client.publish(MQTT_MANUAL_LIGHT_STATE_TOPIC, LIGHT_OFF, true);
    }
}

// function called to publish the brightness of the led
void publishLightBrightness() {
  if(m_current_auto_state == on_auto) {
    snprintf(m_msg_buffer, MSG_BUFFER_SIZE, "%d", map(m_max_light_brightness, 0, 1023, 0, 255));
    client.publish(MQTT_AUTO_LIGHT_BRIGHTNESS_STATE_TOPIC, m_msg_buffer, true);
  } else if (m_current_manual_state == on_ha) {
    snprintf(m_msg_buffer, MSG_BUFFER_SIZE, "%d", map(m_light_brightness, 0, 1023, 0, 255));
    client.publish(MQTT_MANUAL_LIGHT_BRIGHTNESS_STATE_TOPIC, m_msg_buffer, true);
  }
}


// auto on called: set state to on_auto, set m_max_light_brightness to brightness value,
//                 set m_light_state to on, publish manual state to off
// auto off called: set state to off_auto, set m_light_state to off
// manual on called: set state to on_ha, set m_light_brightness to brightness value,
//                   set m_light_state to on, publish auto state to off
// manual off called: set state to off_ha, set m_light_state to off

// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }
  // handle message topic
  if (String(MQTT_AUTO_LIGHT_COMMAND_TOPIC).equals(p_topic)) {
    // test if the payload is equal to "ON" or "OFF"
    if (payload.equals(String(LIGHT_ON))) {
      m_current_auto_state = on_auto;
      m_current_manual_state = off_ha;
      setLightState();
      publishLightState();
    } else if (payload.equals(String(LIGHT_OFF))) {
      m_current_auto_state = off_auto;
      setLightState();
      publishLightState();
    }
  } else if (String(MQTT_AUTO_LIGHT_BRIGHTNESS_COMMAND_TOPIC).equals(p_topic)) {
    uint8_t brightness = payload.toInt();
    if (brightness < 0 || brightness > 1023) {
      // do nothing...
      return;
    } else {
      m_max_light_brightness = map(brightness, 0, 255, 0, 1023);
      setLightState();
      publishLightBrightness();
    }
  } else if (String(MQTT_MANUAL_LIGHT_COMMAND_TOPIC).equals(p_topic)) {
    // test if the payload is equal to "ON" or "OFF"
    if (payload.equals(String(LIGHT_ON))) {
      m_current_manual_state = on_ha;
      m_current_auto_state = off_auto;
      setLightState();
      publishLightState();
    } else if (payload.equals(String(LIGHT_OFF))) {
      m_current_manual_state = off_ha;
      setLightState();
      publishLightState();
    }
  } else if (String(MQTT_MANUAL_LIGHT_BRIGHTNESS_COMMAND_TOPIC).equals(p_topic)) {
    uint8_t brightness = payload.toInt();
    if (brightness < 0 || brightness > 1023) {
      // do nothing...
      return;
    } else {
      m_light_brightness = map(brightness, 0, 255, 0, 1023);
      setLightState();
      publishLightBrightness();
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("INFO: Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID)) {
      Serial.println("\nINFO: connected");

      // Once connected, publish an announcement...
      // publish the initial values
      publishLightState();
      publishLightBrightness();

      // ... and resubscribe
      client.subscribe(MQTT_AUTO_LIGHT_COMMAND_TOPIC);
      client.subscribe(MQTT_AUTO_LIGHT_BRIGHTNESS_COMMAND_TOPIC);
      client.subscribe(MQTT_MANUAL_LIGHT_COMMAND_TOPIC);
      client.subscribe(MQTT_MANUAL_LIGHT_BRIGHTNESS_COMMAND_TOPIC);
    } else {
      Serial.print("ERROR: failed, rc=");
      Serial.print(client.state());
      Serial.println("DEBUG: try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup ( void ) {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("INFO: WiFi connected");
  Serial.print("INFO: IP address: ");
  Serial.println(WiFi.localIP());

  startNTP();

  //ArduinoOTA.setPassword((const char *)"123");
  ArduinoOTA.setHostname("aquariumLights");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
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

  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  client.setCallback(callback);

  analogWriteFreq(500);
  analogWrite(D1, m_light_brightness);

  DS18B20.begin();
  DS18B20.setResolution(11);
}

void state_machine() {
  if(m_current_auto_state == on_auto) {
    R = (pwmIntervals * log10(2))/(log10(m_max_light_brightness));
    // artificially increase the time by 15 mins ever loop to simulate faster days.
    time_t utc = now();// + timeWarp*900;
    time_t eastern = usEastern.toLocal(utc);

    if(hour(eastern) >= 7 && hour(eastern) < 10) {
      Serial.println("Getting brighter!");
      float minutesSince7 = (hour(eastern) - 7)*60 + minute(eastern);
      m_light_brightness = pow(2, (minutesSince7 /  R)) - 1;
    }
    if(hour(eastern) >= 10 && hour(eastern) < 19) {
      Serial.println("Peak brightness!");
      m_light_brightness = 1023;
    }
    if(hour(eastern) >= 19 && hour(eastern) < 22) {
      // dim LEDs
      Serial.println("Getting dimmer!");
      float minutesUntil10 = (22 - hour(eastern))*60 - minute(eastern);
      m_light_brightness = pow(2, (minutesUntil10 / R)) - 1;
    }
    if(hour(eastern) >= 22 || hour(eastern) < 7) {
      Serial.println("Peak darkness!");
      m_light_brightness = 0;
    }
    publishLightBrightness();
  } else if(m_current_manual_state == on_ha) {
    // do nothing, because the brightness is set manually from home assistant
  } else if(m_current_manual_state == off_ha && m_current_auto_state == off_auto ) {
    // If both auto and manual are turned off then set the brightness to 0
    m_light_brightness = 0;
  }
  setLightState();
}

void loop ( void ) {
  ArduinoOTA.handle();

  if (!client.connected()) {
    reconnect();
  }
  state_machine();
  client.loop();
  DS18B20.requestTemperatures();
  float temp = DS18B20.getTempCByIndex(0);
  snprintf(m_msg_buffer, MSG_BUFFER_SIZE, "%d.%03d", (int)temp, (int)(temp*1000)%1000);
  client.publish("aquarium/temperature", m_msg_buffer, true);
  delay(500);
}

