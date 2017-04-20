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

void setup() {
  Serial.begin(115200);
  dht.begin();
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
void loop() {
  timer.run();

}


