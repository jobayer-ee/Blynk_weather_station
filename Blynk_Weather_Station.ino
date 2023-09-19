
//Updatet the lines according to your blynk account

#define BLYNK_TEMPLATE_ID "########"
#define BLYNK_TEMPLATE_NAME "#########"
#define BLYNK_AUTH_TOKEN "###################"

/////////////////////////////////////////

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <DHT.h>
#include <NTPClient.h>
#include <Wire.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TimeLib.h> // Include the TimeLib library
#include <SPI.h>
#include <Adafruit_BMP280.h>


BlynkTimer timer;


#define FAN_PIN 33      // Fan pin
#define TH 5
#define LDR 4
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

// Global variable to keep track of LDR state
bool isLightOn = false;

char auth[] = BLYNK_AUTH_TOKEN;


// Define your Wi-Fi credentials
const char* ssid = "#############"; // Update Wifi info
const char* password = "##########";


// Define the NTP server
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 6 * 3600;  // GMT offset for Dhaka (in seconds) - 6 hours ahead of GMT
const int daylightOffset_sec = 0;  // No daylight offset for Dhaka

DHT dht(TH, DHT22);
Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

// Create an NTP client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec, daylightOffset_sec);

// Create an OLED display instance
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  // Initialize OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Blynk.begin(auth, ssid, password);
   timer.setInterval(100L, sendSensor);
  Serial.begin(9600);
while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  dht.begin();
  pinMode(FAN_PIN, OUTPUT);
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */



  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Initialize NTP client
  timeClient.begin();

  // Clear the display
  display.clearDisplay();
  display.display();
}

//Get the LDR sensor values
void LDRsensor() {
  bool value = digitalRead(LDR);
   isLightOn = (value == 1);
  if (value == 1) {
    WidgetLED LED(V4);
    LED.on();
  } else {
    WidgetLED LED(V4);
    LED.off();
  }
}
void sendSensor()
{
    int temp =bmp.readTemperature();
    float pressure= bmp.readPressure();
    float firstFourDigits = (int)(pressure / 100.0) + 1.0;
    Serial.println(firstFourDigits);
    float altitude=bmp.readAltitude(firstFourDigits);
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(firstFourDigits));/* Adjusted to local forecast! */
    Serial.println(" m");                 //If you don't know it, modify it until you get your current altitude
    Serial.println();

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
    Blynk.virtualWrite(V0, t);
    Blynk.virtualWrite(V1, h);
    Blynk.virtualWrite(V5, temp);
    Blynk.virtualWrite(V3, pressure);
    Blynk.virtualWrite(V9, altitude);  

    if (!isnan(temp) && temp >= 35.0) {
    digitalWrite(FAN_PIN, HIGH);  // Turn on the fan
  } else {
    digitalWrite(FAN_PIN, LOW);   // Turn off the fan
  }
    delay(2000);
}


void loop() {
  // Update the NTP client
  timeClient.update();

  // Get the current time
  time_t rawTime = timeClient.getEpochTime();
  struct tm *timeInfo;
  timeInfo = localtime(&rawTime);

  // Convert to 12-hour format
  int hour = (hourFormat12(rawTime) == 0) ? 12 : hourFormat12(rawTime);
  int minute = timeInfo->tm_min;
  String amPm = (hour < 12) ? "AM" : "PM";

  // Format the time
  String formattedTime = String(hour) + ":" + (minute < 10 ? "0" : "") + String(minute) + " " + amPm;

  // Get BMP280 temperature
  float bmpTemperature = bmp.readTemperature();

  // Display the time, BMP280 temperature, DHT22 temperature, humidity, pressure, and altitude
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

 display.setCursor(0, 0);
  display.print("IP: ");
  display.println(WiFi.localIP());

  display.setCursor(0, 10);
  display.print("Time: ");
  display.print(formattedTime);
 
  // Display BMP280 temperature
  display.setCursor(0, 20);
  display.print("T-IN:");
  int temp =bmp.readTemperature();
  display.print(temp);
  display.print(" OUT:");
  float t = dht.readTemperature();
  display.print(t);
  display.print(" C");

  // Display humidity
  display.setCursor(0, 30);
  display.print("Humidity: ");
  float h = dht.readHumidity();
  display.print(h);
  display.print("%");

  // Display pressure
  display.setCursor(0, 40);
  display.print("Pressure: ");
    float pressure= bmp.readPressure();
    float firstFourDigits = (int)(pressure / 100.0) + 1.0;
  display.print(firstFourDigits);
  display.print(" Pa");

  // Display altitude
  display.setCursor(0, 50);
  display.print("Altitude: ");
  float altitude=bmp.readAltitude(firstFourDigits);
  display.print(altitude); // Modify the sea level pressure accordingly
  display.print(" M");

  display.display();
  // Update every minute (60 seconds delay)
  delay(1000);

  Blynk.run();
  LDRsensor();
  timer.run();
}
