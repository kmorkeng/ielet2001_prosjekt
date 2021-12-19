//for å få mest mulig nøyaktig output, trenger man flest mulig desimaler som med GPS.latitude/longitude
double longitude = 1026.7897;
double latitude = 6324.8810;

double latitude100 = latitude / 100;
double longitude100 = longitude / 100;

void setup() {
  Serial.begin(115200);
}

void loop() {
  double dddLong = int(longitude100);
  double m2m5Long = (longitude100 - dddLong) * 100;

  double ddLat = int(latitude100);
  double m2m5Lat = (latitude100 - ddLat) * 100;

  /*
    Serial.println("Start");
    Serial.println(dddLong);
    Serial.println(m2m5Long, 5);  //viser flere desimaler ved å gjøre sånn her

    Serial.println(ddLat);
    Serial.println(m2m5Lat, 5);
  */


  double ddLatitude = ddLat + m2m5Lat / 60;
  double ddLongitude = dddLong + m2m5Long / 60;

  /*
    Serial.println(m2m5Lat/60, 6);
    Serial.println(m2m5Long/60, 6);
  */

  Serial.print("Latitude: ");
  Serial.println(ddLatitude, 6);
  Serial.print("Longitude: ");
  Serial.println(ddLongitude, 6);
}
