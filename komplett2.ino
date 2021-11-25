#include "UbidotsEsp32Mqtt.h"
#include "vector"

#include <HardwareSerial.h>
#include <Adafruit_GPS.h>


#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>

WiFiManager wifiManager;
WiFiManager wm;


/****************************************
 * Define Constants
 ****************************************/
const char *UBIDOTS_TOKEN = "BBFF-nKvPfwxPTDAW1eoBkh6Nxpi4hQbaed";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "Shady_nettverk";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "heiheihei";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "test";   // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL = "gps"; // Put here your Variable label to which data  will be published
const char *SUBSCRIBE_DEVICE_LABEL = "maptrace";
const char *SUBSCRIBE_VARIABLE_LABEL_LAT = "latitudes";
const char *SUBSCRIBE_VARIABLE_LABEL_LONG = "longitudes";
const int PUBLISH_FREQUENCY = 3500; // Update rate in milliseconds
const int RECONNECT_FREQUENCY = 5000; // Attempt new reconnect
const int TIMEOUT = 10000;

unsigned long timer,timer2,timerGPS;
uint8_t analogPin = 34; // Pin used to read data from GPIO34 ADC_CH6.

double latitudeGPS = 1.25164;
double longitudeGPS = -77.28426;

bool getLat,getLong;
bool switchh = 1;

int longitudesLength;

//std::vector<double> latitudes = {45.7519089,45.7561908,4.8427391,45.7553674,45.7507560,45.7481656};
//std::vector<double> longitudes = {4.8369455,4.8403573};
std::vector<double> latitudes = {};
std::vector<double> longitudes = {};


Ubidots ubidots(UBIDOTS_TOKEN);

#define GPSSerial Serial2 // RX - IO17; TX - IO16
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

/****************************************
 * Auxiliar Functions
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
  double value = btof(payload,length);
  Serial.println();  
  if (getLong == 1){
    Serial.print("long");
    /*Serial.println(getLong);
    Serial.print("Lat");
    Serial.println(getLat);
    Serial.print("hello");*/
    if(longitudes.size() == 0){
      longitudes.push_back(value);
    }
    else if(value != longitudes[longitudes.size()-1]){
        longitudes.push_back(value);
      }
    }
    
  if (getLat == 1){
    /*Serial.print("long");
    Serial.println(getLong);
    */Serial.print("Lat");
    //Serial.println(getLat); 
    if(latitudes.size() == 0 ){
      latitudes.push_back(value);
    }
    else if(value != latitudes[latitudes.size()-1]){
      latitudes.push_back(value);
    }
    
  }
  getLong = 0;
  getLat = 0;
}

/****************************************
 * Main Functions
 ****************************************/
double findDistance(double x0, double y0, double x1, double y1,double x2, double y2){ 
  //bruker double for å få med alle desimalpunktene til koordinatene
  //denne funksjonen finner distansen til ett punkt(x0,y0) fra en linje definert av x1 y1 og x2 y2
  float t = abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1)); //teller
  float n = sqrt(pow((x2-x1),2) + pow((y2-y1),2)); //nevner
  return(t/n);
}

void findInterval(){ 
  
}
void getLatitude(){ /*henter nye variabler hvis verdi fra ubidots er annerledes fra verdi på esp32*/
    getLat = 1;
    getLong = 0;
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL_LAT);
}
void getLongitude(){
    getLong = 1;
    getLat = 0;
    ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL_LONG);
}
std::vector<double> cleanVector(std::vector<double> x){ //I visse tilfeller havner det en ekstra verdi i feil vector, fjerner disse med denne funksjonen
    double var = trunc(x.at(0)); // Denne funksjonen finner outliers i vector ved å se på hvilke verdier som er typiske i den og fjerner da de som ikke er det
    int count = 1;
    for (int i = 0; i < x.size(); i++) {
      if(var == trunc(x.at(i))){
        count++;
      }
      Serial.println(x.at(i),7);
    }
    if (count > x.size()/2){
      int a = 0;
      while(a < x.size()){
        if(trunc(x.at(a)) != var){
          x.erase(x.begin()+a);
          a = 0;
        }else{
          a++;
        }
      }
    }
    if(count <= x.size()/2){
      int a = 0;
      while(a < x.size()){
        if(trunc(x.at(a) == var)){
          x.erase(x.begin()+a);
          a = 0;
        }else{
          a++;
        }
      }
    }

    for (int i = 0; i < x.size(); i++) {
      Serial.println(x.at(i),7);
    }
  return x;
}
void getCoordinatesUbidots(){
  bool getCoordinates = true;

  while(getCoordinates == true){
    if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {/* Reserves memory to store context key values, add as much as you need */
    //char* str_lat = (char*)malloc(sizeof(char) * 10);
   //char* str_lng = (char*)malloc(sizeof(char) * 10);
    //char str_lat,str_lng,context;

    /* Saves the coordinates as char */
    //sprintf(str_lat, "%f", latitudeGPS);
    //sprintf(str_lng, "%f", longitudeGPS);

    /* Reserves memory to store context array */
    //char* context = (char*)malloc(sizeof(char) * 30);

    /* Adds context key-value pairs */
    //ubidots.addContext("lat", str_lat);
    //ubidots.addContext("lng", str_lng);

    /* Builds the context with the coordinates to send to Ubidots */
    //ubidots.getContext(context);

    //ubidots.add(VARIABLE_LABEL, 1, context); 
    //ubidots.publish(DEVICE_LABEL);
    timer = millis();
    
    if( switchh == 1){
      getLatitude();
      switchh = 0;
    }else{
      getLongitude();
      switchh = 1;
    }

    for (int i = 0; i < latitudes.size(); i++) {
      Serial.println(latitudes.at(i),7);
    }
    Serial.println("LONG");
      for (int i = 0; i < longitudes.size(); i++) {
        Serial.println(longitudes.at(i),7);
    }
  

     if (abs(millis() - timer2) > 5*PUBLISH_FREQUENCY){
      Serial.println("her");
      if(longitudes.size() == longitudesLength && longitudesLength != 0){
          Serial.println("fikser");
          longitudes = cleanVector(longitudes);
          latitudes = cleanVector(latitudes);
     }
        timer2 = millis();
        longitudesLength = longitudes.size();
        getCoordinates = false;        
      }
    }    
  }
}
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  
  // 9600 NMEA is the default baud rate for MTK - some use 4800

  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // 
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);


  wm.setConfigPortalBlocking(false);
  if(wm.autoConnect(WIFI_SSID, WIFI_PASS)){
      Serial.println("Connected to WiFi on setup");
      ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
      ubidots.reconnect();
      delay(100);
      if (ubidots.connected()){
          Serial.println("Connected to Ubidots on setup");
      }
  }
  else {
      Serial.println("Configportal running");
  }
    
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();

  timer = millis();
  timer2 = millis();
  timerGPS = millis();

  longitudesLength = longitudes.size();

  //getCoordinatesUbidots(); //henter maptrace koordinater fra ubidots
}

void loop()
{
  // put your main code here, to run repeatedly:
  if ((!ubidots.connected()) && (WiFi.status()== WL_CONNECTED)){
    ubidots.reconnect();
  }

  char c = GPS.read(); // Henter GPS-data

  if (abs(millis() - timerGPS) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {/* Reserves memory to store context key values, add as much as you need */
    if (GPS.newNMEAreceived()) { // Parser GPS-data
      Serial.print("Last NMEA: "); Serial.print(GPS.lastNMEA());
      if (!GPS.parse(GPS.lastNMEA()))
        return;
    }
    
    char* str_lat = (char*)malloc(sizeof(char) * 10);
    char* str_lng = (char*)malloc(sizeof(char) * 10);
    //char str_lat,str_lng,context;

    if (GPS.fix) { // Hvis det er GPS-signal
      longitudeGPS = GPS.longitude;
      latitudeGPS = GPS.latitude;
      Serial.println(" "); Serial.print("Longitude: "); Serial.print(longitudeGPS);
      Serial.print("   Latitude: "); Serial.print(latitudeGPS); Serial.println(" ");
    }
    else { // Hvis det ikke er GPS-signal
      Serial.println("No fix");
    }

    /* Saves the coordinates as char */
    sprintf(str_lat, "%f", latitudeGPS);
    sprintf(str_lng, "%f", longitudeGPS);

    /* Reserves memory to store context array */
    char* context = (char*)malloc(sizeof(char) * 30);

    /* Adds context key-value pairs */
    ubidots.addContext("lat", str_lat);
    ubidots.addContext("lng", str_lng);

    /* Builds the context with the coordinates to send to Ubidots */
    ubidots.getContext(context);

    ubidots.add(VARIABLE_LABEL, 1, context); 
    ubidots.publish(DEVICE_LABEL);
    timerGPS = millis();
  }


  /*if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {/* Reserves memory to store context key values, add as much as you need */
    //char* str_lat = (char*)malloc(sizeof(char) * 10);
   //char* str_lng = (char*)malloc(sizeof(char) * 10);
    //char str_lat,str_lng,context;

    /* Saves the coordinates as char */
    //sprintf(str_lat, "%f", latitudeGPS);
    //sprintf(str_lng, "%f", longitudeGPS);

    /* Reserves memory to store context array */
    //char* context = (char*)malloc(sizeof(char) * 30);

    /* Adds context key-value pairs */
    //ubidots.addContext("lat", str_lat);
    //ubidots.addContext("lng", str_lng);

    /* Builds the context with the coordinates to send to Ubidots */
    //ubidots.getContext(context);

    //ubidots.add(VARIABLE_LABEL, 1, context); 
    //ubidots.publish(DEVICE_LABEL);
    /*timer = millis();
    
    if( switchh == 1){
      getLatitude();
      switchh = 0;
    }else{
      getLongitude();
      switchh = 1;
    }
    for (int i = 0; i < latitudes.size(); i++) {
    Serial.println(latitudes.at(i),7);
    }
    Serial.println("LONG");
    for (int i = 0; i < longitudes.size(); i++) {
      Serial.println(longitudes.at(i),7);
    }/*
   // getLong = 0;
    //getLat = 0;
   // Serial.println(longitudes.size());
    //Serial.println(longitudes[longitudes.size()-1],7); 
    //Serial.println(longitudes[0],7);       
    
    /*while(0 == 1){ While løkke for å hente variablene og legge de inn i en vector
      latitudes.push_back(getLatitude(0));
      longitudes.push_back(getLongitude(0));
      Serial.println(longitudes[0],7);
      sleep(10);
    } 
    
    /*Serial.println(findDistance(45.7539089,4.8399455,45.7519089,4.8369455,45.7561908,4.8403573),7);*/
    /* free memory 
    //free(str_lat);
    //free(str_lng);
    //free(context);
  
    /*float value = analogRead(analogPin);
    ubidots.add(VARIABLE_LABEL, value); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    /*timer = millis();*/
  //}
    //Serial.print(longitudesLength);
/*   if (abs(millis() - timer2) > 5*PUBLISH_FREQUENCY){
     Serial.println("her");
     if(longitudes.size() == longitudesLength && longitudesLength != 0){
        Serial.println("fikser");
        longitudes = cleanVector(longitudes);
        latitudes = cleanVector(latitudes);
     }
     timer2 = millis();
      longitudesLength = longitudes.size();
    }*/

  ubidots.loop();



}