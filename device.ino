/****************************************
   INKLUDERER BIBLIOTEKER
 ****************************************/
#include "UbidotsEsp32Mqtt.h"
#include "vector"

#include <HardwareSerial.h>  //GPS-en bruker seriell kommunikasjon på de fysiske UART-portene
#include <Adafruit_GPS.h>  //GPS-biblioteket

/****************************************
   DEFINERER VARIABLER
 ****************************************/

//ubidots
const char *UBIDOTS_TOKEN = "BBFF-8eYG4XFUFDUWjtzgxbu7WMXwj2EavJ"; //ubidots-token
const char *WIFI_SSID = "AndroidAP";      //wifi ssid og passord for å komme på nett
const char *WIFI_PASS = "1q2w3e4r";

const char *DEVICE_LABEL = "test";   //put here your Device label to which data  will be published
const char *VARIABLE_LABEL = "gps"; //put here your Variable label to which data  will be published
const char *SUBSCRIBE_DEVICE_LABEL = "maptrace";
const char *SUBSCRIBE_VARIABLE_LABEL_LAT = "latitudes";
const char *SUBSCRIBE_VARIABLE_LABEL_LONG = "longitudes";
const char *PUBLISH_VARIABLE_LABEL_GETCOORDINATES = "getcoordinates";

const int PUBLISH_FREQUENCY = 3500; // oppdateringsraten i millisekunder
const int RECONNECT_FREQUENCY = 5000; // Attempt new reconnect
const int TIMEOUT = 10000;

//timere
unsigned long timer, timer2, timerGPS;
uint8_t analogPin = 34; //pin brukt til å lese data fra GPIO34, eller ADC_CH6

// tid til sensor
unsigned long time_now = 0;
unsigned long time_before = 0;
float periode = 0;

// buzzer
const int freq = 500;      // frekvens på lyd til sensor
const int buzzerpin = 26;  // pin i bruk for buzzer
const int channel = 0;
const int freqoff = 300;  // frekvens på lyd for slå av
const int freqon = 700;   // frekvens på lyd for slå på
const int freqgps = 800;   // frekvens på lyd gps avstand

// sensor
const int trigPin = 5;    // pin i bruk for trigPin
const int echoPin = 18;   // pin i bruk for echoPin
float startwarning = 20;  // avstand den begynner å varsle på (i cm)
long duration;
float distanceCm;
#define SOUND_SPEED 0.034

// knapp
#define button 33
int state = 1;

// touch sensititittivivivty
#define Threshold 40

//touch knapper
const int TouchReset = 15;  //touch knapp on
const int TouchOff = 4;     //touch knapp off

double latitudeGPS = 63.25164;
double longitudeGPS = 10.28426;

bool getLat, getLong;
bool switchh = 1;

int longitudesLength;

//std::vector<double> latitudes = {45.7519089,45.7561908,45.7553674,45.7507560,45.7481656,45.7466084,45.7490491};
//std::vector<double> longitudes = {4.8369455,4.8403573,4.8427391,4.8395634,4.8436189,4.8427606,4.8361087};
std::vector<double> latitudes = {};
std::vector<double> longitudes = {};
int intervals; //hvor mange intervaller det er, altså hvor mange koordinater.
int interval = 0; //hvilket intervall man er i nå


Ubidots ubidots(UBIDOTS_TOKEN); //ubidots-objekt som inneholder token

#define GPSSerial Serial2 // RX - IO17; TX - IO16
Adafruit_GPS GPS(&GPSSerial); //GPS-objektet inneholder adressen til seriellport 2
#define GPSECHO false

/****************************************
   HJELPEFUNKSJONER
 ****************************************/
double btof(byte *payload, unsigned int length) //konverterer array med chars til en double
{
  char *demo_ = (char *)malloc(sizeof(char) * 10);
  for (int i = 0; i < length; i++)
  {
    demo_[i] = payload[i];
  }
  return atof(demo_);
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);

  }
  double value = btof(payload, length);
  Serial.println();
  if (getLong == 1) {
    Serial.print("long");
    /*Serial.println(getLong);
      Serial.print("Lat");
      Serial.println(getLat);
      Serial.print("hello");*/
    if (longitudes.size() == 0) {
      longitudes.push_back(value);
    }
    else if (value != longitudes[longitudes.size() - 1]) {
      longitudes.push_back(value);
    }
  }

  if (getLat == 1) {
    /*Serial.print("long");
      Serial.println(getLong);
    */Serial.print("Lat");
    //Serial.println(getLat);
    if (latitudes.size() == 0 ) {
      latitudes.push_back(value);
    }
    else if (value != latitudes[latitudes.size() - 1]) {
      latitudes.push_back(value);
    }

  }
  getLong = 0;
  getLat = 0;
}

/****************************************
   KART OG GPS-FUNKSJONER
 ****************************************/

double findDistance(double x0, double y0, double x1, double y1, double x2, double y2) {
  //bruker double for å få med alle desimalpunktene til koordinatene
  //denne funksjonen finner distansen til ett punkt(x0,y0) fra en linje definert av x1 y1 og x2 y2
  double t = abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)); //teller
  double n = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2)); //nevner
  return (t / n);
}

void getLatitude() { //henter nye variabler hvis verdi fra ubidots er annerledes fra verdi på esp32
  getLat = 1;
  getLong = 0;
  ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL_LAT);
}

void getLongitude() {
  getLong = 1;
  getLat = 0;
  ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL_LONG);
}

std::vector<double> cleanVector(std::vector<double> x) { //I visse tilfeller havner det en ekstra verdi i feil vector, fjerner disse med denne funksjonen
  double var = trunc(x.at(0)); // Denne funksjonen finner outliers i vector ved å se på hvilke verdier som er typiske i den og fjerner da de som ikke er det
  int count = 1;
  for (int i = 0; i < x.size(); i++) {
    if (var == trunc(x.at(i))) {
      count++;
    }
    Serial.println(x.at(i), 7);
  }
  if (count > x.size() / 2) {
    int a = 0;
    while (a < x.size()) {
      if (trunc(x.at(a)) != var) {
        x.erase(x.begin() + a);
        a = 0;
      } else {
        a++;
      }
    }
  }
  if (count <= x.size() / 2) {
    int a = 0;
    while (a < x.size()) {
      if (trunc(x.at(a) == var)) {
        x.erase(x.begin() + a);
        a = 0;
      } else {
        a++;
      }
    }
  }

  for (int i = 0; i < x.size(); i++) {
    Serial.println(x.at(i), 7);
  }
  return x;
}

void getCoordinatesUbidots() { //henter koordinater fra ubidots, må kjøres synkront med et python skript som sender de valgte "waypoints" til ubidots.
  bool getCoordinates = true;
  ubidots.add(PUBLISH_VARIABLE_LABEL_GETCOORDINATES, 1);
  ubidots.publish(DEVICE_LABEL);
  sleep(3);

  while (getCoordinates == true) {
    if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
    {
      timer = millis();

      if ( switchh == 1) {
        getLatitude();
        switchh = 0;
      } else {
        getLongitude();
        switchh = 1;
      }

      for (int i = 0; i < latitudes.size(); i++) {
        Serial.println(latitudes.at(i), 7);
      }
      Serial.println("LONG");
      for (int i = 0; i < longitudes.size(); i++) {
        Serial.println(longitudes.at(i), 7);
      }


      if (abs(millis() - timer2) > 5 * PUBLISH_FREQUENCY) {
        Serial.println("her");
        if (longitudes.size() == longitudesLength && longitudesLength != 0) {
          Serial.println("fikser");
          longitudes = cleanVector(longitudes);
          latitudes = cleanVector(latitudes);
        }
        timer2 = millis();
        longitudesLength = longitudes.size();
        ubidots.add(PUBLISH_VARIABLE_LABEL_GETCOORDINATES, 0);
        ubidots.publish(DEVICE_LABEL);
        getCoordinates = false;
      }
    }
  }
}

void checkInterval(double x0, double y0, double x1, double y1) { //denne funksjonen sjekker om man skal til neste intervall eller ikke. Tar 2 punkter
  double criticalDistance = 0.0001; //11 meter hvor langt unna man skal være en koordinat før man begynner på "neste" intervall
  double distance = sqrt(pow((x0 - x1), 2) + pow((y0 - y1), 2)); //formel for distanse mellom 2 punkter
  if (distance < criticalDistance) {
    interval++;
  }
}

double dmm2ddLatitude(double latitude) {
  //oppdeling av breddegrad-koordinatene på ddmm.mmmmm-formatet for så å gjøre det om til decimal degrees

  double latitude100 = latitude / 100;
  double ddLat = int(latitude100);
  double m2m5Lat = (latitude100 - ddLat) * 100;

  double ddLatitude = ddLat + m2m5Lat / 60;
  return ddLatitude;

}

double dmm2ddLongitude(double longitude) {
  //oppdeling av lengdegrad-koordinatene på dddmm.mmmmm-formatet for så å gjøre det om til decimal degrees

  double longitude100 = longitude / 100;
  double dddLong = int(longitude100);
  double m2m5Long = (longitude100 - dddLong) * 100;

  double ddLongitude = dddLong + m2m5Long / 60;
  return ddLongitude;

}

/****************************************
   AVSTANDSSENSOR OG BUZZERLYDER
 ****************************************/

void sensorgobeep() {
  time_now = millis();   // setter time_now til tiden nå

  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance
  distanceCm = duration * SOUND_SPEED / 2;
  Serial.println(distanceCm);

  periode = distanceCm * 15;

  if (time_now >  time_before + periode && distanceCm < startwarning)  // om det har gått lengre tid enn avstand multiplisert med 15 og mindre enn "x"
  {
    if (distanceCm < 4)
    {
      ledcWriteTone(channel, freq);
    }
    else
    {
      //buzzer goess peepepeeepeeeep
      ledcWriteTone (channel, freq);
      delay(50);
      ledcWriteTone(channel, 0);
      time_before = millis();
    }
  }
  else if (distanceCm > startwarning)
  {
    ledcWriteTone(channel, 0);
  }
}
void TurnOnSound()
{
  for (int i = 0; i < 2; i++) // kjører to trege peep
  {
    ledcWriteTone(channel, freqon);
    delay (250);
    ledcWriteTone(channel, 0);
    delay (250);
  }
}

void turnoffsound() //kjører 3 beep
{
  for (int i = 0; i < 3; i++)
  {
    ledcWriteTone(channel, freqoff);
    delay (150);
    ledcWriteTone(channel, 0);
    delay (150);
  }
}


void GPSSound()
{
  ledcWriteTone(channel, freqgps);
  delay (150);
  ledcWriteTone(channel, 0);
  delay (150);

}

/****************************************
   SLEEP MODE-FUNKSJONER
 ****************************************/

void reset()
{
  ESP.restart();
}

void deepsleep()
{
  delay (1000);
  esp_deep_sleep_start();  //starter deep sleep
}


/****************************************
   SETUP-FUNKSJONEN
 ****************************************/

void setup() {
  Serial.begin(115200); //starter seriell monitor med baudrate på 115200 symbol/sek

  pinMode(trigPin, OUTPUT); // setter  trigPin som en output
  pinMode(echoPin, INPUT);  // setter  echoPin som en input
  pinMode(button, INPUT_PULLUP);
  ledcAttachPin(buzzerpin, channel); // buzzer

  //setup for GPS-en
  GPS.begin(9600); // 9600 NMEA is the default baud rate for MTK - some use 4800
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //setter formatet på NMEA-meldingen
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  //ubidots
  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();

  //starter timere
  timer = millis();
  timer2 = millis();
  timerGPS = millis();

  longitudesLength = longitudes.size();

  getCoordinatesUbidots(); //henter maptrace koordinater fra ubidots
  //Setup interrupt on(GPIO15) wakeup + reset
  touchAttachInterrupt(TouchReset, reset, Threshold);
  esp_sleep_enable_touchpad_wakeup();  //setter oppvåkningskilden på ESP32-en til touchpaden

  TurnOnSound(); //kjørt igjennom all setup (lag lyd)
}


/****************************************
   HOVEDLOOPEN
 ****************************************/

void loop() {

  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }

  char c = GPS.read(); // henter GPS-data

  if (abs(millis() - timerGPS) > PUBLISH_FREQUENCY) //triggers the routine every 5 seconds
  {
    if (GPS.newNMEAreceived()) { // parser GPS-data
      Serial.print("Last NMEA: "); Serial.print(GPS.lastNMEA());
      if (!GPS.parse(GPS.lastNMEA()))
        return;
    }

    //reserverer minne til å lagre context-verdier, legg til mer ved behov
    char* str_lat = (char*)malloc(sizeof(char) * 10);
    char* str_lng = (char*)malloc(sizeof(char) * 10);
    //char str_lat,str_lng,context;

    if (GPS.fix) { //hvis det ikke er noe GPS-signal (koordinater)

      //setter koordinatene fra GPS-en direkte i funksjonene som gjør om fra dmm til dd-format
      longitudeGPS = dmm2ddLongitude(GPS.longitude);
      latitudeGPS = dmm2ddLatitude(GPS.latitude);

      Serial.println(" "); Serial.print("Longitude: "); Serial.print(longitudeGPS, 6);
      Serial.print("   Latitude: "); Serial.print(latitudeGPS, 6); Serial.println(" ");
    }
    else { //dersom man ikke får inn posisjonsdata, sendes default-verdiene til ubidots

      Serial.println("No fix");

    }
    //Serial.println("0");

    //lagrer koordinatene som chars
    sprintf(str_lat, "%f", latitudeGPS);
    sprintf(str_lng, "%f", longitudeGPS);
    //Serial.println("1");

    //setter av minne til å lagre context array
    char* context = (char*)malloc(sizeof(char) * 30);
    //Serial.println("2");

    // Adds context key-value pairs
    ubidots.addContext("lat", str_lat);
    ubidots.addContext("lng", str_lng);
    //Serial.println("3");

    //Builds the context with the coordinates to send to Ubidots
    ubidots.getContext(context);
    //Serial.println("4");
    ubidots.add(VARIABLE_LABEL, 1, context);
    ubidots.publish(DEVICE_LABEL);
    //Serial.println("5");

    intervals = longitudes.size(); //setter intervals til lengden longitudes vector.
    if (interval < intervals) { //sjekker om man er ferdig med løypa slik at en ikke går out of bounds
      if (findDistance(latitudeGPS, longitudeGPS, latitudes[interval], longitudes[interval], latitudes[interval + 1], longitudes[interval + 1]) > 0.001) { //sjekker distansen fra en linje mellom 2 punkter er større en 0.001 koordinater
        GPSSound();

      }
      //Serial.println("6");
      checkInterval(latitudeGPS, longitudeGPS, latitudes[interval + 1], longitudes[interval + 1]); //sjekker om man skal gå til neste intervall
    }
    timerGPS = millis();
    Serial.println("7");
  }

  if (digitalRead(button) == 0)  // en enkel "touchknapp for test"
  {
    delay (500);
    if (state == state)
    {
      state = !state;
    }
    else if (state = !state)
    {
      state = state;
    }

  }

  if (state == 0)
  {
    sensorgobeep();
    delay(100);
  }
  if (touchRead(TouchOff) < Threshold)
  {
    turnoffsound();
    deepsleep();
  }


  ubidots.loop();

}
