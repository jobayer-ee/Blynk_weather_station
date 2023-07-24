#define BLYNK_TEMPLATE_ID "TMPL6ugaSfvko"
#define BLYNK_TEMPLATE_NAME "SKYNET"
#define BLYNK_AUTH_TOKEN "3eQGUVCwdANLou4LdF9b-t7E-B4RkkVH"
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <DHT.h>
#define FAN_PIN 18      // Fan pin
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
char auth[] = BLYNK_AUTH_TOKEN;

char ssid[] = "FUNNY BEAR";  // type your wifi name
char pass[] = "deyajabena";  // type your wifi password

BlynkTimer timer;
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#define TH 5
#define LDR 4
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

DHT dht(TH, DHT22);
Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

void setup() {
  Blynk.begin(auth, ssid, pass);
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
}
//Get the DHT11 sensor values
void DHT11sensor() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);
}

//Get the LDR sensor values
void LDRsensor() {
  bool value = digitalRead(LDR);
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
    Blynk.virtualWrite(V5, temp);
    Blynk.virtualWrite(V3, pressure);
    Blynk.virtualWrite(V9, altitude);  //The "1019.66" is the pressure(hPa) at sea level in day in your region
    if (!isnan(temp) && temp >= 35.0) {
    digitalWrite(FAN_PIN, HIGH);  // Turn on the fan
  } else {
    digitalWrite(FAN_PIN, LOW);   // Turn off the fan
  }
    delay(2000);
}
void loop()
{
  
Blynk.run();
LDRsensor();
DHT11sensor();
  timer.run();
  }
