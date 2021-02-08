/////////************* **********/////////
//          Blynk Assignments           //
/////////************* **********/////////
/*
V0   -Blynk.virtualWrite(V0, Time.format(Time.local(), "%r - %a %D"));
      Blynk.virtualWrite(V1, temp);
      Blynk.virtualWrite(V2, humidity);
      Blynk.virtualWrite(V3, pressure);
      Blynk.virtualWrite(V4, qual);
V5   - Terminal Widget
V10  - LowAlertSetPoint   EEPROM.put(1
V11  - HighAlertSetPoint  EEPROM.put(50
V12  - LowAlertSetPoint returned to Blynk app (Settings)
V13  - HighAlertSetPoint returned to Blynk app (Settings)
*/
SYSTEM_THREAD(ENABLED);
#define Location LS  //location of device
#define db2p TRUE  //debug2particle   >>if(db2p) {Particle.publish("in db2p");}
//////******Still publishing to particle in createEventPayload!!!!<<boron data usage
#define SENSOR_READING_INTERVAL 30000  //<<< in loop()  sendAlert() relies on this
#define FirstRunDelay 8000
bool firstrunlockout = 1;
long sendinfo2blynk = 60000;
long sendalert2blynk = 5000;
long resendalert2blynk = 720000;  //720000 12min.
bool sendtoParticle = 0;

/////////************* **********/////////
//                Variables             //
/////////************* **********/////////
#include <blynk.h>
WidgetTerminal terminal(V5);
//char auth[] = "Mi23GHp-MVumIsjfunxSQT_WxnutVvs1";  //Red Barn Greenhouse in Blynk
//#if ("LS")
char auth[] = "PqViD5lZykvxcKA35ifjfI2MoPEKoBSF"; //HomeAir in Blynk
//#endif
//char auth[] = "ETkv5bSL15G5a_mhlcpQCPog4MqGm_R8";  //IDGreenhouse in Blynk
//char auth[] = "L3TJ_dNNlXRNDT-CcQWRhRIsRxicAyIc";
#define BlynkAlertLED D7    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool LEDstatus = 0;

BlynkTimer timer;
int timerNA = 99;
int resetAlertTimer = timerNA;
bool havealerted = 0;
bool alertenable = 1;
int lowalertsetpoint = 60;  //RedBarn 60   IDGreenhouse 50
int highalertsetpoint = 105;  //RedBarn 105  IDGreenhouse 90

#include <JsonParserGeneratorRK.h>

#include <OLED_Display_128X64.h>
void updateDisplay(int temp, int humidity, int pressure, String airQuality);

//#include <Grove_scd30_co2_sensor.h>  //I2C >> 0x61 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <SparkFun_SCD30_Arduino_Library.h>
SCD30 airSensor;

#include <Adafruit_BME280.h>   //I2C >> 0x76(default) or 0x77  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
Adafruit_BME280 bme;
int temp, pressure, humidity;
int getBMEValues(int &temp, int &humidity, int &pressure);

#include <Grove_Air_quality_Sensor.h>
#define AQS_PIN A2            //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
String getAirQuality();
AirQualitySensor aqSensor(AQS_PIN);
String qual;

#define DUST_SENSOR_PIN D4    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void getDustSensorReadings();
unsigned long lastInterval;
unsigned long lowpulseoccupancy = 0;
unsigned long last_lpo = 0;
unsigned long duration;
float ratio = 0;
float concentration = 0;

void createEventPayload(int temp, int humidity, int pressure, String airQuality);
////////**********Time***********/////////
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
unsigned long lastSync = millis();
time_t t;
/**** BlynkButtons *************************************************************************************************************/
BLYNK_WRITE(V10) {
  lowalertsetpoint = param.asInt(); // assigning incoming value from pin to a variable
  EEPROM.put(1, lowalertsetpoint);
}
BLYNK_WRITE(V11) {
  highalertsetpoint = param.asInt(); // assigning incoming value from pin to a variable
  EEPROM.put(50, highalertsetpoint);
}
BLYNK_WRITE(V20) {
  alertenable = param.asInt(); // assigning incoming value from pin to a variable
  digitalWrite(BlynkAlertLED, alertenable);
}
  
void setup() {
  delay(50);
  Time.zone(-6);  //-5for old time
  Blynk.begin(auth);
  timer.setInterval(sendinfo2blynk, sendinfo);
  timer.setInterval(sendalert2blynk, sendAlert);
  
  EEPROM.get(1, lowalertsetpoint);
  EEPROM.get(50, highalertsetpoint);
  Blynk.virtualWrite(V20, alertenable);
  Blynk.virtualWrite(V10, lowalertsetpoint);
  Blynk.virtualWrite(V11, highalertsetpoint);
  Blynk.notify(String("Device Rebooting... Allow 1 min for sensor readings to settle")); // +  myStr + ("°F"));
  pinMode(BlynkAlertLED, OUTPUT);

  // Configure the dust sensor pin as an input
  pinMode(DUST_SENSOR_PIN, INPUT);
  if (aqSensor.init()) {
    if(db2p && Particle.connected()) Particle.publish("DustSen", "Ready", PRIVATE | WITH_ACK);
  }
  else {
    if(db2p) Particle.publish("DustSen", "Error"); delay(1000);
  }

  Wire.begin();
  SeeedOled.init();

  SeeedOled.clearDisplay();
  SeeedOled.setNormalDisplay();
  SeeedOled.setPageMode();

  SeeedOled.setTextXY(2, 0);
  SeeedOled.putString("Particle");
  SeeedOled.setTextXY(3, 0);
  SeeedOled.putString("Air Quality");
  SeeedOled.setTextXY(4, 0);
  SeeedOled.putString("Monitor");

  if (bme.begin()) {
    if(db2p) Particle.publish("BME280", "Ready", PRIVATE | WITH_ACK); delay(1000);
  }
  else {
    if(db2p) Particle.publish("BME280", "Error"); delay(1000);
  }
  
  if (airSensor.begin()) {
    if(db2p) Particle.publish("SCD30", "Ready"); delay(1000);
  }
  else {
    if(db2p) Particle.publish("SCD30", "Error"); delay(1000);
  }
  airSensor.setMeasurementInterval(8); //Change number of seconds between measurements: 2 to 1800 (30 minutes)
  //My desk is ~1600m above sealevel
  //airSensor.setAltitudeCompensation(1600); //Set altitude of the sensor in m
  //Pressure in Boulder, CO is 24.65inHg or 834.74mBar
  //airSensor.setAmbientPressure(835); //Current ambient pressure in mBar: 700 to 1200

  lastInterval = millis();
}

void loop()
{
    Blynk.run();
    timer.run();
    firstrun();

  duration = pulseIn(DUST_SENSOR_PIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;

  if ((millis() - lastInterval) > SENSOR_READING_INTERVAL) {
    String quality = getAirQuality();
    //Serial.printlnf("Air Quality: %s", quality.c_str());
    getBMEValues(temp, pressure, humidity);
    //Serial.printlnf("Temp: %d", temp);
    //Serial.printlnf("Pressure: %d", pressure);
    //Serial.printlnf("Humidity: %d", humidity);
    getDustSensorReadings();
    updateDisplay(temp, humidity, pressure, quality);
    createEventPayload(temp, humidity, pressure, quality);
    lowpulseoccupancy = 0;
    lastInterval = millis();
    
    if (airSensor.dataAvailable()) { //SCD30 data
    String SCD_CO2;
    String SCD_Temp;
    String SCD_Hum;
    SCD_CO2 = String(airSensor.getCO2());
    SCD_Temp = String(airSensor.getTemperature(), 1);
    SCD_Hum = String(airSensor.getHumidity(), 1);
    Particle.publish("SCD30/readings", String("CO2:") + SCD_CO2 + (" Temp:") + SCD_Temp + (" Hum:") + SCD_Hum);
    
    //Particle.publish("co2(ppm):");
    //Particle.publish(" temp(C):");
    //Particle.publish(airSensor.getTemperature(), 1);

    //Particle.publish(" humidity(%):");
    //Particle.publish(airSensor.getHumidity(), 1);
  }
  else Particle.publish("SCD30", "Error in loop()");
  }
  
  if (millis() - lastSync > ONE_DAY_MILLIS) {
        Particle.syncTime();
        lastSync = millis();
    }
} //end loop
  
void firstrun() {
    if (millis() > FirstRunDelay && firstrunlockout == 0) {
        getBMEValues(temp, pressure, humidity);
        sendinfo();
        firstrunlockout = 1;
    }
}

void sendinfo() {
      getBMEValues(temp, pressure, humidity); //get current readings
      Blynk.virtualWrite(V0, Time.format("%r - %a %D"));
      Blynk.virtualWrite(V1, temp);
      Blynk.virtualWrite(V2, humidity);
      Blynk.virtualWrite(V3, pressure);
      Blynk.virtualWrite(V4, qual);
}

void sendAlert() {
    if(!havealerted && alertenable) {
        if(temp < lowalertsetpoint) {
            String myStr;
            myStr = String(temp);
            Blynk.notify(String("Low Temp Alert-") + /*("Location") +*/  myStr + ("°F"));
            Blynk.virtualWrite(V0, Time.format("%r - %a %D"));
            Blynk.virtualWrite(V1, temp);
            havealerted = 1;
            if (!timer.isEnabled(resetAlertTimer)) { resetAlertTimer = timer.setTimeout(resendalert2blynk, [] () {resetAlert(); resetAlertTimer = timerNA;} ); }
        }
        if(temp > highalertsetpoint) {
            String myStr;
            myStr = String(temp);
            Blynk.notify(String("High Temp Alert! ") +  myStr + ("°F"));
            //Blynk.virtualWrite(V0, Time.timeStr());
            Blynk.virtualWrite(V0, Time.format("%r - %a %D"));
            Blynk.virtualWrite(V1, temp);
            havealerted = 1;
            if (!timer.isEnabled(resetAlertTimer)) { resetAlertTimer = timer.setTimeout(resendalert2blynk, [] () {resetAlert(); resetAlertTimer = timerNA;} ); }
        }
    }
}
void resetAlert() {
    havealerted = 0;
}
/////////************* **********/////////
//           Sensor Functions           //
/////////************* **********/////////
String getAirQuality() {
  int quality = aqSensor.slope();
  qual = "None";

  if (quality == AirQualitySensor::FORCE_SIGNAL)
  {
    qual = "Danger";
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION)
  {
    qual = "High Pollution";
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION)
  {
    qual = "Low Pollution";
  }
  else if (quality == AirQualitySensor::FRESH_AIR)
  {
    qual = "Fresh Air";
  }

  return qual;
}

int getBMEValues(int &temp, int &pressure, int &humidity) {
  temp = (int)(bme.readTemperature() * 1.8F + 32); //convert temp to fahrenheit
  pressure = (int)(bme.readPressure() / 100.0F);
  humidity = (int)bme.readHumidity();

  return 1;
}

void getDustSensorReadings() {
  // This particular dust sensor returns 0s often, so let's filter them out by making sure we only
  // capture and use non-zero LPO values for our calculations once we get a good reading.
  if (lowpulseoccupancy == 0) {
    lowpulseoccupancy = last_lpo;
  }
  else {
    last_lpo = lowpulseoccupancy;
  }

  ratio = lowpulseoccupancy / (SENSOR_READING_INTERVAL * 10.0);                   // Integer percentage 0=>100
  concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve

  Serial.printlnf("LPO: %d", lowpulseoccupancy);
  Serial.printlnf("Ratio: %f%%", ratio);
  Serial.printlnf("Concentration: %f pcs/L", concentration);
}

void createEventPayload(int temp, int humidity, int pressure, String airQuality) {
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);

    jw.insertKeyValue("temp", temp);
    jw.insertKeyValue("humidity", humidity);
    jw.insertKeyValue("pressure", pressure);
    jw.insertKeyValue("air-quality", airQuality);

    if (lowpulseoccupancy > 0)
    {
      jw.insertKeyValue("dust-lpo", lowpulseoccupancy);
      jw.insertKeyValue("dust-ratio", ratio);
      jw.insertKeyValue("dust-concentration", concentration);
    }
  }
  if(sendtoParticle == 1)
  {
      Particle.publish("env-vals", jw.getBuffer(), PRIVATE);
  }

}

void updateDisplay(int temp, int humidity, int pressure, String airQuality) {
  SeeedOled.clearDisplay();

  SeeedOled.setTextXY(0, 3);
  SeeedOled.putString(airQuality);

  SeeedOled.setTextXY(2, 0);
  SeeedOled.putString("Temp: ");
  SeeedOled.putNumber(temp);
  SeeedOled.putString("F");

  SeeedOled.setTextXY(3, 0);
  SeeedOled.putString("Humidity: ");
  SeeedOled.putNumber(humidity);
  SeeedOled.putString("%");

  SeeedOled.setTextXY(4, 0);
  SeeedOled.putString("Press: ");
  SeeedOled.putNumber(pressure);
  SeeedOled.putString(" hPa");

  if (concentration > 1)
  {
    SeeedOled.setTextXY(5, 0);
    SeeedOled.putString("Dust: ");
    SeeedOled.putNumber(concentration); // Will cast our float to an int to make it more compact
    SeeedOled.putString(" pcs/L");
  }
}
