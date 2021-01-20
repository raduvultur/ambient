/*
Used libraries:
	knolleary/PubSubClient@^2.8
	adafruit/Adafruit BME280 Library@^2.1.2
	olikraus/U8g2@^2.28.8
	wifwaf/MH-Z19@^1.5.3
	arduino-libraries/NTPClient@^3.1.0
	ricki-z/SDS011 sensor Library@^0.0.6

Modification of HardwareSerial.cpp (for the SDS011 serial pins)

#ifndef RX1
#define RX1 12
#endif

#ifndef TX1
#define TX1 14
#endif
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include "MHZ19.h" 
#include <NTPClient.h>
#include <WiFiUdp.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#include <SDS011.h>

#define WIFISSID "wifi name" // Put your WifiSSID here
#define PASSWORD "wifi pass" // Put your wifi password here
#define MQTT_CLIENT_NAME "ambient1" // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string; 
                                    // it should be a random and unique ascii string and different from all other devices

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Adafruit_BME280 bme;
MHZ19 myMHZ19;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);

float p10, p25;
int err;
SDS011 my_sds;
HardwareSerial port(1);

// Create variable to hold mqtt messages
#define MSG_BUFFER_SIZE	(100)
char msg[MSG_BUFFER_SIZE];

char mqttBroker[]  = "192.168.0.220";
const char* topic = "ambient"; // CHANGE SensorID here!

U8G2LOG u8g2log;

// setup the terminal (U8G2LOG) and connect to u8g2 for automatic refresh of the display
// The size (width * height) depends on the selected font and the display

// assume 4x6 font
#define U8LOG_WIDTH 32
#define U8LOG_HEIGHT 10
uint8_t u8log_buffer[U8LOG_WIDTH*U8LOG_HEIGHT];

WiFiClient mywifi;
PubSubClient client(mywifi);

void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  Serial.write(payload, length);
  Serial.println(topic);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    
    // Attemp to connect
    if (client.connect(MQTT_CLIENT_NAME)) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  digitalWrite(10, 0);
  digitalWrite(9, 0);

  Serial2.begin(9600);
  myMHZ19.begin(Serial2);
  my_sds.begin(&port);

  u8g2.begin();
  
  //https://github.com/olikraus/u8g2/wiki/fntlistmono#8-pixel-height
  u8g2.setFont(u8g2_font_pxplusibmvga8_mr);	// set the font for the terminal window
  u8g2log.begin(u8g2, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
  u8g2log.setLineHeightOffset(0);	// set extra space between lines in pixel, this can be negative
  u8g2log.setRedrawMode(0);		// 0: Update screen with newline, 1: Update screen for every char
  u8g2log.print("\f");

  WiFi.begin(WIFISSID, PASSWORD);
  bme.begin(0x76);

  u8g2log.print("Wait for WiFi...");u8g2log.print("\n");
  Serial.println();
  Serial.print("Wait for WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    u8g2log.print(".");
    delay(500);
  }
  
  timeClient.begin();
  timeClient.setTimeOffset(1);

  Serial.println("");
  u8g2log.print("\f");u8g2log.print("WiFi Connected");u8g2log.print("\n");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  u8g2log.print("IP address: ");u8g2log.print("\n");
  u8g2log.print(WiFi.localIP());u8g2log.print("\n");
  Serial.println(WiFi.localIP());
  client.setServer(mqttBroker, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  my_sds.read(&p25, &p10);

  String message = "location=office temperature=" + String(bme.readTemperature()) + " humidity=" + String(bme.readHumidity()) + " pressure=" + String(bme.readPressure()) + " co2=" + String(myMHZ19.getBackgroundCO2());
  message.toCharArray(msg, message.length());
  
  Serial.println(message); // Data Format 
  client.publish(topic, msg);

  timeClient.update();

  u8g2log.print("\f");
  u8g2log.println(timeClient.getFormattedTime());

  u8g2log.print("Temp  ");u8g2log.println(String(bme.readTemperature()));
  u8g2log.print("Hum   ");u8g2log.println(String(bme.readHumidity()));
  //u8g2log.print("Press ");u8g2log.println(String(bme.readPressure()));
  u8g2log.print("CO2   ");u8g2log.println(myMHZ19.getBackgroundCO2());
  u8g2log.print("PM ");u8g2log.print(String(p25));u8g2log.print("-");u8g2log.println(String(p10));
  u8g2.sendBuffer();

  delay(10000); // Publish data every x seconds to the Broker
}
