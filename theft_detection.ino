#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
  #include <Arduino_ConnectionHandler.h>

#define PIR_PIN1 19 // Connect PIR sensor 1 to GPIO 2
#define PIR_PIN2 18 // Connect PIR sensor 2 to GPIO 4
#define PIR_PIN3 5 // Connect PIR sensor 3 to GPIO 5
#define RELAY_PIN 17 // Connect relay module to GPIO 12
#define BUZZER_PIN 2 // Connect buzzer to GPIO 13
#define NIGHT_MODE_START_HOUR 14 // Night mode start hour (24-hour format)
#define NIGHT_MODE_END_HOUR 14 // Night mode end hour (24-hour format)

const char* ssid = "SCRC_LAB_IOT";
const char* password = "Scrciiith@123";
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800; // GMT offset in seconds (1 hour)
const int daylightOffset_sec = 3600; // Daylight offset in seconds (1 hour)
  WiFiConnectionHandler conn(ssid,password);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec, daylightOffset_sec);


void setup() {
  pinMode(PIR_PIN1, INPUT);
  pinMode(PIR_PIN2, INPUT);
  pinMode(PIR_PIN3, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize the NTP client
  timeClient.begin();
  timeClient.setTimeOffset(gmtOffset_sec);
}

void loop() {
  conn.check(); // reconnecting the wifi after getting disconnected
  timeClient.update();
  int currentHour = timeClient.getHours();
 Serial.println(currentHour);
  // Check if it's nighttime
  if (currentHour >= NIGHT_MODE_START_HOUR || currentHour < NIGHT_MODE_END_HOUR) {
    // Check for motion from any of the PIR sensors
    if (digitalRead(PIR_PIN1) || digitalRead(PIR_PIN2) || digitalRead(PIR_PIN3)) {
      // Motion detected during nighttime, trigger the relay and buzzer
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(BUZZER_PIN, HIGH);
      Serial.print("Motion Detected");
      delay(5000); // Delay for 1 second (adjust as needed)
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
delay(2000);
}
