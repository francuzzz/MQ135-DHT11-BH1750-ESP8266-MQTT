//WiFi
#include <ESP8266WiFi.h>
const char* ssid = "VA";
const char* password = "qwerty123";
WiFiClient espClient;

//MQTT
#include <PubSubClient.h>
#define mqtt_server "m21.cloudmqtt.com"
#define mqtt_user "sejaohtm"
#define mqtt_password "uiYM7HIoLWqo"
#define mqtt_clientId "ESP0081C778"


const long sampleDelay = 30000;
PubSubClient client(espClient);
// MQTT Topics
#define humidity_topic "sensor/humidity"
#define temperature_topic "sensor/temperature"
#define co2_topic "sensor/co2"

#include <DHT.h>
#include <DHT_U.h>
#include <MQ135.h>
#define DHTTYPE DHT11 // Тип датчика
#define DHTPIN 5 //К какому пину подключен датчик DHT
DHT dht (DHTPIN, DHTTYPE);

MQ135 gasSensor = MQ135 (A0); //Датчик газа подключен к аналоговому пину


#include <TimeLib.h>
#include <SimpleTimer.h>
SimpleTimer timer;

unsigned long lastSampleTime = 0;

void setup() {
  Serial.begin(115200);
  dht.begin();
  setupWifi();
  client.setServer(mqtt_server, 11918);
  timer.setInterval(5000L,sendTemps);

}

void sendTemps (){

float h = dht.readHumidity(); 
float t = dht.readTemperature(); 

 if (isnan(h) || isnan(t)){
     return;
     }

float rzero = gasSensor.getRZero();
float ppm = gasSensor.getPPM();
float rzeroc = gasSensor.getCorrectedRZero(t, h);
float ppmc = gasSensor.getCorrectedPPM(t, h);


Serial.print("Humidity: ");
  Serial.println(h);
  Serial.print("Temperature: ");
  Serial.println(t);
  Serial.print("A0: ");
  Serial.println(analogRead (A0));
  Serial.print("Rzero: ");
  Serial.println(rzero);
  Serial.print("Rzeroc: ");
  Serial.println(rzeroc);
  Serial.print("PPM: ");
  Serial.println(ppm);
  Serial.print("PPMC: ");
  Serial.println(ppmc);
  Serial.println();

}
void setupWifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

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
    if (client.connect(mqtt_clientId, mqtt_user, mqtt_password)) {
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

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
  float h = dht.readHumidity(); 
  float t = dht.readTemperature(); 

 if (isnan(h) || isnan(t)){
     return;
     }

  float rzero = gasSensor.getRZero();
  float ppm = gasSensor.getPPM();
  float rzeroc = gasSensor.getCorrectedRZero(t, h);
  float ppmc = gasSensor.getCorrectedPPM(t, h);
  
  if (currentMillis - lastSampleTime >= sampleDelay) {
    lastSampleTime = currentMillis;

    if (isnan(t)) {
      Serial.println("Error reading temperature!");
    }
    else {
      
      client.publish(temperature_topic, String(t).c_str(), true);

      Serial.print("Temperature: ");
      Serial.print(t);
      Serial.println(" *C");
    }
    // Get humidity event and print its value.
    if (isnan(h)) {
      Serial.println("Error reading humidity!");
    }
    else {
      
      client.publish(humidity_topic, String(h).c_str(), true);

      Serial.print("Humidity: ");
      Serial.print(h);
      Serial.println("%");
    }
    client.publish(co2_topic, String(ppmc).c_str(), true);
  }
}
