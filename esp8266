/***************************************************************************
  Sample sketch for using a bme680 air quality sensor with a ESP8266 and sending the result over MQTT once a minute. 

  Written by Erik Lemcke, combined out of the following samples:

  https://learn.adafruit.com/adafruit-bme680-humidity-temperature-barometic-pressure-voc-gas/arduino-wiring-test BME680 code & library by adafruit 
  https://www.home-assistant.io/blog/2015/10/11/measure-temperature-with-esp8266-and-report-to-mqtt/, home assistant mqqt by Paulus Schoutsen

***************************************************************************/
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <BlynkSimpleEsp8266.h>
#include "DFRobot_BME680_I2C.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

//Blynk Auth
char auth[] = "449f3ffab7624b21af98b2abbc7360ba";

#define wifi_ssid "RobotUnicorn"
#define wifi_password "finlayfreya"

#define mqtt_server "192.168.1.33" //pi!
#define mqtt_user ""
#define mqtt_password ""

#define temp_topic "bme680/temp"
#define hum_topic "bme680/hum"
#define gas_topic "bme680/gas"
#define pressure_topic "bme680/pressure"
#define alt_topic "bme680/alt"
#define plantMoistureTopic  "plant/simonsPlant"

#define SEALEVELPRESSURE_HPA (1013.25)

//NODEMCU connected to BME680- 
// SDA - D2
// SCL - D1

int MoistureValue, MoisturePercent ;
int sensor_pin = A0;
char stringValue[16];

float seaLevel; 
BlynkTimer timer;

//Adafruit_BME680 bme; // I2C
DFRobot_BME680_I2C bme(0x76);  //0x76 I2C address 

WiFiClient espClient;
PubSubClient client(espClient);


void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

 
  
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // If you do not want to use a username and password, change next line to
     if (client.connect("ESP8266Client")) {
    //if (client.connect("ESP8266Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


// This function sends Arduino's up time every second to Virtual Pin (5).
// In the app, Widget's reading frequency should be set to PUSH. This means
// that you define how often to send data to Blynk App.
void sendSensor()
{
  float h = bme.readHumidity() /1000;
  float t = bme.readTemperature() / 100;

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, h);
  Blynk.virtualWrite(V6, t);
}


void setup() {
  uint8_t rslt = 1;
  Serial.begin(115200);
  Serial.println("Startup");

  setup_wifi();
  client.setServer(mqtt_server, 1883); // 192.168.1.33:1883 (live) --- 192.168.1.105:1880
  
  unsigned long previousMillis = millis();

 while(!Serial);
  delay(1000);
  Serial.println();
  while(rslt != 0) {
    rslt = bme.begin();
    if(rslt != 0) {
      Serial.println("bme begin failure");
      delay(2000);
    }
  }
  Serial.println("bme begin successful");
  //#ifdef CALIBRATE_PRESSURE
  bme.startConvert();
  delay(1000);
  bme.update();

 seaLevel = bme.readSeaLevel(134.51); //DOWNDERRY
  Serial.print("seaLevel :");
  Serial.println(seaLevel);
 // #endif


//Blynk cloud
Blynk.begin(auth, wifi_ssid, wifi_password);

  //Blynk local 
  //Blynk.begin(auth, wifi_ssid, wifi_password, IPAddress(192,168,1,33), 8080); 
  // Setup a function to be called every second
  timer.setInterval(10000L, sendSensor);  // 3000 = 3 seconds
}

long lastMsg = 0;

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  //client.loop();

  long now = millis();
  //send a meaage every minute
  if (now - lastMsg > 10 * 1000) {
    lastMsg = now;

   
  bme.startConvert();
  //delay(1000);
  bme.update();
  Serial.println();
  Serial.print("temperature(C) :");
  
  Serial.println(bme.readTemperature() / 100, 2);
  Serial.print("pressure(Pa) :");
  Serial.println(bme.readPressure());
  Serial.print("humidity(%rh) :");
  Serial.println(bme.readHumidity() / 1000, 2);
  Serial.print("gas resistance(ohm) :");
  Serial.println(bme.readGasResistance());
  Serial.print("altitude(m) :");
  Serial.println(bme.readAltitude());
  //#ifdef CALIBRATE_PRESSURE
  Serial.print("calibrated altitude(m) :");
  Serial.println(bme.readCalibratedAltitude(seaLevel));

//soil sensor
  MoistureValue = analogRead(sensor_pin);
  Serial.print("moist value: ");
  Serial.print(MoistureValue);Serial.print(" ");
  MoisturePercent = map(MoistureValue,834,440,0,100); //Dry,Wet,lowest,highest
  if (MoisturePercent >100) {
    MoisturePercent = 100;
  }
  if (MoisturePercent <0) {
    MoisturePercent = 0;
  }
  Serial.print("Plant Mositure: ");
  Serial.print(MoisturePercent);
  Serial.println("%");
  
//JSON
StaticJsonBuffer<300> JSONbuffer;
JsonObject& JSONencoder = JSONbuffer.createObject();
 
  JSONencoder["idx"] = "0001";
  JSONencoder["temp"] = bme.readTemperature() / 100, 0;
  JSONencoder["hum"] = bme.readHumidity() / 1000, 0;
  JSONencoder["pressure"] = bme.readPressure();
  JSONencoder["moisture"] = MoisturePercent;

  char JSONmessageBuffer[100];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);
 
  if (client.publish("esp/test", JSONmessageBuffer) == true) {
    Serial.println("Success sending message");
  } else {
    Serial.println("Error sending message");
  }

  client.loop();
  Serial.println("-------------");
   
   //MQTT
   /*
   client.publish(temp_topic, String(bme.readTemperature() / 100, 0).c_str(), true);
   client.publish(hum_topic, String(bme.readHumidity() / 1000, 0).c_str(), true);
   client.publish(pressure_topic, String(bme.readPressure() /1000, 0).c_str(), true);
   client.publish(gas_topic, String(bme.readGasResistance() / 1000, 0).c_str(), true);
   client.publish(alt_topic, String(bme.readAltitude()).c_str(), true);

   client.publish(plantMoistureTopic, itoa(value, stringValue, 10));
*/
   }
   
 Blynk.run();
  timer.run();  
    
  }
