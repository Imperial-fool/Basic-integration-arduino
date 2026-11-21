#include <Adafruit_GPS.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_HTU31D.h>
#include <Adafruit_TSL2591.h>
#include <Adafruit_Sensor.h>


Adafruit_GPS *GPS = nullptr;
Adafruit_HTU31D *Temp_Humidity_sensor = nullptr;
Adafruit_TSL2591 *Lux_sensor = nullptr;
Adafruit_SH1107 *Display = nullptr;

/*
Global Vars
*/
float humidity, temp = 0.0f;
int32_t lat,lon = 0;

int32_t illuminance = 0;

uint32_t tGPS = 0, tTHS = 0, tLux = 0, tDisp = 0, tStatus = 0;


void halt(const char *msg) {
  Serial.println(msg);
  while (true) delay(10);
}
bool due(uint32_t &last, uint32_t period) {
  uint32_t now = millis();
  if (now - last >= period) { last = now; return true; }
  return false;
}
/*
GPS init and functions
*/
bool initGPS() {
  GPS = new Adafruit_GPS(&Wire);
  if (GPS->begin(0x10)) {
    GPS->sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS->sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    return true;
  }
  return false;
}

bool readGPS() {
  bool gotNMEAResponse = false;

  GPS->read();
    if (GPS->newNMEAreceived()) {
      gotNMEAResponse = true;
      char *s = GPS->lastNMEA(); 

      Serial.print(s);
      bool ok = GPS->parse(s);
      Serial.print(F("parse ")); Serial.print(ok ? F("ok") : F("FAIL"));
      Serial.print(F(" fix=")); Serial.print(GPS->fix);
      Serial.print(F(" q="));   Serial.print(GPS->fixquality);
      Serial.print(F(" sats=")); Serial.println(GPS->satellites);
      if (!ok) {
        Serial.print("Failed to parse.");
        return false;
      }
      if(GPS->fix){  
        lat = GPS->latitude_fixed;   
        lon = GPS->longitude_fixed;
        return true;
      }else{
        Serial.println("no fix");
        return false;
      }
    }
  return false; 
}

void printGPS(){
  Serial.print(F("Location: "));
  printFixedPoint(lat, 10000000L);
  Serial.print(F(", "));
  printFixedPoint(lon, 10000000L);
}

void printGPS(Print &out) {

  out.print(F("Lat: "));
  printFixedPoint(out, lat, 10000000L);
  out.print(F("Lon: ")); 
  printFixedPoint(out, lon, 10000000L);
}
/*
end GPS init and functions
*/

//M0 doesn't support floating point math in hardware, ideally we do everything with ints except for display
void printFixedPoint(int32_t v, long fixedPoint){
  if(v < 0){ Serial.print('-'); v = -v;}
  Serial.print(v / fixedPoint);
  Serial.print('.');
  int32_t frac = v %  fixedPoint;
  for (int32_t p =  fixedPoint / 10; p > 1 && frac < p; p /= 10) Serial.print('0'); 
  Serial.print(frac);

}
void printFixedPoint(Print &out, int32_t v, long fixedPoint) {
  if (v < 0) { out.print('-'); v = -v; }
  out.print(v / fixedPoint);
  out.print('.');
  int32_t frac = v % fixedPoint;
  for (int32_t p = fixedPoint / 10; p > 1 && frac < p; p /= 10) out.print('0'); 
  out.println(frac);
}
/*
Temp_Humidity_sensor init and functions
*/
bool initTHsensor() {
  Temp_Humidity_sensor = new Adafruit_HTU31D();
  return Temp_Humidity_sensor->begin();
}
bool readTHS(){
  sensors_event_t hum, tem;
  Temp_Humidity_sensor->getEvent(&hum, &tem);
  float h = hum.relative_humidity;
  float t = tem.temperature;

  if (isnan(h) || isnan(t)) {
    return false;
  }
  if (t < -40.0f || t > 125.0f || h < 0.0f || h > 100.0f) {
    return false;
  }
  humidity = h;
  temp = t;

  return true;
}
void printTHS(){
  Serial.print(F("Huminity: "));
  Serial.println(humidity, 2);
  Serial.print(F("Temperature: "));
  Serial.println(temp, 2);
}

void printTHS(Print &out) {
  out.print(F("Humidity: ")); 
  out.println(humidity, 2);
  out.print(F("Temp: "));    
  out.println(temp, 2);
}
/*
end Temp_Humidity_sensor init and functions
*/

/*
Lux_sensor init and functions
*/
bool initLux() {
  Lux_sensor = new Adafruit_TSL2591();
  if(Lux_sensor->begin(&Wire)){
    Lux_sensor->setGain(TSL2591_GAIN_MED);
    Lux_sensor->setTiming(TSL2591_INTEGRATIONTIME_300MS);
    return true;
  };
  return false;
}

bool readLux(){

  uint32_t full  = Lux_sensor->getFullLuminosity(); //full luminosity returns a 32 bit uint with half representing ir and the other representing visible light
  /*
  full = [ upper 16 bits | lower 16 bits ]
         (channel 1 ir)  (channel 0, visible)
  */
  uint16_t ir = full >> 16; //we shift over the upper half and assign it to ir
  uint16_t visible = full & 0xFFFF; //we 'and' the full with 0xFFFF which zeros out the top assigning the lower channel to visible 
   if (visible == 0xFFFF || ir == 0xFFFF) {
    return false;
  }
  float lux = Lux_sensor->calculateLux(visible, ir);
  if (lux < 0) {
    return false;
  }
  illuminance = (int32_t)lroundf(lux); //calculate and return an int that is between 0 and 88000 representing the rough range of the sensor

  return true;
}

void printLux(){
    Serial.println(F("Current Lux: "));
    Serial.print(illuminance);
}

void printLux(Print &out) {
  out.print(F("Lux: ")); out.println(illuminance);
}
/*
end Lux_sensor init and functions
*/

/* display init and functions*/
bool initDisplay() {
  Display = new Adafruit_SH1107(64, 128, &Wire);

  if (!Display->begin(0x3C, true)) return false;

  Display->setRotation(1);
  Display->setTextSize(1);
  Display->setTextColor(SH110X_WHITE);
  Display->clearDisplay();
  yield();
  Display->display();

  return true;
}

void refreshDisplay() {
  Display->clearDisplay();
  Display->setCursor(0, 0);
  printGPS(*Display);
  printTHS(*Display);
  printLux(*Display);
  yield();
  Display->display();
}
/*end display init and functions*/
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {};

  if (!initDisplay()) halt("Display init failed");

  bool failed = false;

  if (initGPS()) {
    Serial.println(F("GPS ok"));
    if(readGPS()){
      printGPS();
    }

  }
  else { Serial.println(F("GPS failed")); failed = true; }

  if (initTHsensor()) {
    Serial.println(F("HTU31D ok"));
    if(readTHS()) printTHS();
  }
  else { Serial.println(F("HTU31D failed")); failed = true; }

  if (initLux()) {
    Serial.println(F("Lux ok"));
    if(readLux()) printLux();
  }
  else { Serial.println(F("TSL2591 failed")); failed = true; }

  if (failed) halt("One or more modules failed");
  tGPS = millis();
  while(!GPS->fix){
    Display->println("Waiting on GPS fix.");
    Display->print(">time waiting: ");
    Display->println((millis() - tGPS)/1000);
    Display->display();
    delay(200);
    Display->clearDisplay();
  }
  tGPS = 0;
}

void loop() {
  bool THS_OK, LUX_OK, GPS_OK = true;


  if (due(tGPS, 100))   GPS_OK = readGPS();                
  if (due(tTHS, 1000))  THS_OK = readTHS();
  if (due(tLux, 1000))  LUX_OK = readLux(); 

  if (due(tDisp, 500))  refreshDisplay();

  if (due(tStatus, 1000)) {
    Serial.print(F("THS:")); Serial.print(THS_OK ? F("ok") : F("FAIL"));
    Serial.print(F(" LUX:")); Serial.print(LUX_OK ? F("ok") : F("FAIL"));
    Serial.print(F(" GPS:")); Serial.println(GPS_OK ? F("ok") : GPS->fix ? F("fix") : F("nofix"));
  }

  refreshDisplay();
  delay(500);
}
















































