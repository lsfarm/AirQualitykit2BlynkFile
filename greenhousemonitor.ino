// This #include statement was automatically added by the Particle IDE.
//#include <PublishQueueAsyncRK.h>  //https://github.com/rickkas7/PublishQueueAsyncRK
//retained uint8_t publishQueueRetainedBuffer[2048];
//PublishQueueAsync publishQueue(publishQueueRetainedBuffer, sizeof(publishQueueRetainedBuffer));

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
#define SENSOR_READING_INTERVAL 30  //<<< in loop()  sendAlert() relies on this 30 = 3 sec.
#define FirstRunDelay 8000
bool firstrunlockout = 1;
long sendinfo2blynk = 60000;
long sendalert2blynk = 5000;
long resendalert2blynk = 720000;  //720000 12min.
bool sendtoParticle = 0;

/////////************* **********/////////
//                Variables             //
/////////************* **********/////////
bool BME280Status;
bool SCD30Status;
//bool BME280Status;
#include <blynk.h>
bool ReCnctFlag = 0;
int ReCnctCount;
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

void sensorInit() {
  // Configure the dust sensor pin as an input
  pinMode(DUST_SENSOR_PIN, INPUT);
  if (aqSensor.init()) {
      //publishQueue.publish("DustSen", "Ready", PRIVATE | WITH_ACK);
    if(db2p) Particle.publish("DustSen", "Ready", PRIVATE | WITH_ACK); delay(1000); //if(db2p && Particle.connected()) {bool result = Particle.publish("DustSen", "Ready", PRIVATE | WITH_ACK); delay(4000);}
  }
  else {
    if(db2p) Particle.publish("DustSen", "Error"); delay(1000);
  }
  if (bme.begin()) { //BME280
      //publishQueue.publish("BME280", "Ready", PRIVATE | WITH_ACK);
    if(db2p) Particle.publish("BME280", "Ready", PRIVATE | WITH_ACK); delay(1000);
    BME280Status = 1;
  }
  else {
    if(db2p) Particle.publish("BME280", "Error"); delay(1000);
    BME280Status = 0;
  }
  
  if (airSensor.begin()) { //SCD30
      //publishQueue.publish("SCD30", "Ready", PRIVATE | WITH_ACK);
    if(db2p) Particle.publish("SCD30", "Ready"); //delay(5000);
    SCD30Status = 1;
  }
  else {
    if(db2p) Particle.publish("SCD30", "Error"); delay(1000);
    SCD30Status = 0;
  }
  airSensor.setMeasurementInterval(8); //Change number of seconds between measurements: 2 to 1800 (30 minutes)
  //My desk is ~1600m above sealevel
  //airSensor.setAltitudeCompensation(1600); //Set altitude of the sensor in m
  //Pressure in Boulder, CO is 24.65inHg or 834.74mBar
  //airSensor.setAmbientPressure(835); //Current ambient pressure in mBar: 700 to 1200


}
  
void setup() {
    //delay(5000);
    //sensorInit();
    delay(50);
    Time.zone(-6);  //-5for old time
    Blynk.begin(auth);
    //sensorInit();
    timer.setInterval(30000L, updateAllSensors);
    timer.setInterval(sendinfo2blynk, sendinfo);
    timer.setInterval(sendalert2blynk, sendAlert);
  
    EEPROM.get(1, lowalertsetpoint);
    EEPROM.get(50, highalertsetpoint);
    Blynk.virtualWrite(V20, alertenable);
    Blynk.virtualWrite(V10, lowalertsetpoint);
    Blynk.virtualWrite(V11, highalertsetpoint);
    Blynk.notify(String("Device Rebooting... Allow 1 min for sensor readings to settle")); // +  myStr + ("°F"));
    pinMode(BlynkAlertLED, OUTPUT);
    
    Particle.variable("BME280Ready", BME280Status);
    Particle.variable("SCD30Ready", SCD30Status);
    
    Wire.begin();
    sensorInit();
    
    //Wire.begin();
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
  
    lastInterval = millis();
}

void loop() {
    if (Blynk.connected()) {  // If connected run as normal
        Blynk.run();
    } else if (ReCnctFlag == 0) {
        ReCnctFlag = 1;  // Set reconnection Flag  
        //if(debugEnable) { terminal.println("Starting reconnection timer in 30 seconds..."); terminal.flush(); }
        timer.setTimeout(35000L, []() {  // takes about 30 sec for particle.connected to return false -- blynk.connect will block if particle.connected isn't false yet(setting this lower)
            ReCnctFlag = 0;   ReCnctCount++;  // Increment reconnection Counter  // Reset reconnection Flag
            //if (debugEnable) { terminal.print("Attempting reconnection #"); terminal.println(ReCnctCount); }
            if (Particle.connected()) Blynk.connect();  // Try to reconnect to the server
        });  // END Timer Function
    }
    //Blynk.run();
    timer.run();
    firstrun();

    duration = pulseIn(DUST_SENSOR_PIN, LOW);
    lowpulseoccupancy = lowpulseoccupancy + duration;

    /*if ((millis() - lastInterval) > SENSOR_READING_INTERVAL) {
        digitalWrite(BlynkAlertLED, alertenable);
        if(alertenable) alertenable = 0;
        else alertenable = 1;
        terminal.println(Time.format("%r - %a %D: InLoop()")); terminal.flush();
        lastInterval = millis();
    }*/
  
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
void updateAllSensors() {
    String quality = getAirQuality();
    //createEventPayload(temp, humidity, pressure, quality);
    getBMEValues(temp, pressure, humidity);
    getSCD30values();
    getDustSensorReadings(); 
    lowpulseoccupancy = 0;
}

void sendinfo() {
      getBMEValues(temp, pressure, humidity); //get current readings
      String quality = getAirQuality(); //<<< move this out of BME280 stuff
      if(db2p) createEventPayload(temp, humidity, pressure, quality);
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
void getSCD30values() {
    if (airSensor.dataAvailable()) { //SCD30 data
        String SCD_CO2;
        String SCD_Temp;
        String SCD_Hum;
        SCD_CO2 = String(airSensor.getCO2());
        int val = (airSensor.getTemperature(), 1);
        val = val * 1.8F + 32;  //<<<<<<<<<<<<<this isn't right
        SCD_Temp = String(val);
        SCD_Hum = String(airSensor.getHumidity(), 1);
        Particle.publish("SCD30/readings", String(Time.format("%r - %D ")) + ("CO2: ") + SCD_CO2 + (" Temp: ") + SCD_Temp + (" Hum: ") + SCD_Hum, PRIVATE | WITH_ACK);
    }
    else Particle.publish("SCD30/readings", "Error in updateAllSensors()");
}
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
  /*if(db2p)*/ Particle.publish("BME280/readings", jw.getBuffer(), PRIVATE);

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
