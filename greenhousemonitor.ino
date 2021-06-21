/////////************* **********/////////
//          Blynk Assignments           //
/////////************* **********/////////
/*
V0    Blynk.virtualWrite(V0, Time.format(Time.local(), "%r - %a %D"));
      Blynk.virtualWrite(V1, temp);
      Blynk.virtualWrite(V2, humidity);
      Blynk.virtualWrite(V3, pressure);
      Blynk.virtualWrite(V4, qual);
V5    Blynk.virtualWrite(V5, wifiSTR;
V8   - Terminal Widget
V9   - pwLED Widget //if power at USB
V10  - LowAlertSetPoint   EEPROM.put(1
V11  - HighAlertSetPoint  EEPROM.put(50
V12  - LowAlertSetPoint returned to Blynk app (Settings)
V13  - HighAlertSetPoint returned to Blynk app (Settings)
V14  = HumAlert  Rising Hum Alert
V20  - Temp alert disable button bool alertenable 
V21  - hum alert enabled?? not used yet*/
/////////************* **********/////////
//         Global Variables             //
/////////************* **********/////////
SYSTEM_THREAD(ENABLED); //resaon for this??
#define Amistad  //location of device
/**** Device Settings****************************************************************************************************************/
//#define SENSOR_READING_INTERVAL 30  //<<< in loop()   30 = 3 sec. to discontinue this  using BlynkTimer now
//#define FirstRunDelay 12000
long updateSensors      = 30000L;
// moved to ifdef  long sendInfo2Blynk     = 60000L;
long sendAlert2Blynk    = 4000L; //if less than udpateAllSensors() blynk will alert on reboot << fixing this -Fixed??
long resendAlert2Blynk  = 720000L;  //720000 12min.
bool firstrunlockout    = 0;
int  timeUpdateDay      = 0;
int  runDayCounter      = 0;
int  sensorReInit       = 0;
/**** Particle *********************************************************************************************************************/
//moved to if def bool db2p = TRUE;  //debug2particle   >>if(db2p) {Particle.publish("in db2p");} set this to false when statified with performance to disable sending if rebooted
int switchdb2p(String command); //particle function
int SIG_STR = 99; //strength
int SIG_QUA = 99; //quality
/**** Blynk ************************************************************************************************************************/
#include <blynk.h>
bool ReCnctFlag = 0;
int ReCnctCount;
WidgetTerminal terminal(V8);
//char auth[] = "Mi23GHp-MVumIsjfunxSQT_WxnutVvs1";  //Red Barn Greenhouse in Blynk
#ifdef HomeAir
//char auth[] = "LH9dWY6U3gaULoRu-x9l9pHwDJFueGWa";//PqViD5lZykvxcKA35ifjfI2MoPEKoBSF"; 
String Location = "HomeAirTest";
long sendInfo2Blynk     = 60000L;
bool db2p = TRUE;
#endif
#ifdef RedBarnBeth
//WiFi.setCredentials("Dustin Nikkel", "joyousunit942");
char auth[] = "_9XmQH9iMNDH5-MTZ-uVEcLB1nblzj4U"; //new app 
String Location = "Red Barn Beth";
long sendInfo2Blynk     = 60000L;
bool db2p = FALSE;
#endif
#ifdef RedBarnNorth
char auth[] = "IQZpd5NiLIa9V0Sg2ZR_A04gRQpFJpUi"; //new app
//char auth[] = "CcXWiH1qYe6ACneKvrXitZ_KC56Fbj5W"; //old app
String Location = "Red Barn North";
long sendInfo2Blynk     = 60000L;
bool db2p = FALSE;
#endif
#ifdef RedBarnMain
char auth[] = "LH9dWY6U3gaULoRu-x9l9pHwDJFueGWa"; //new app
String Location = "Red Barn Main";
long sendInfo2Blynk     = 60000L;
bool db2p = FALSE;
#endif
#ifdef RedBarnTexline
char auth[] = "ljq3ApKCHBamvyBRkO-YZADwVgoWJxhT";
String Location = "Red Barn Texline";
long sendInfo2Blynk     = 600000L;
bool db2p = FALSE;
#endif
#ifdef Amistad
FuelGauge fuel;
char auth[] = "jWWNO9Ldl6PshSwBQ2EcxmHuOfrfHAlq";
String Location = "Amistad";
long sendInfo2Blynk     = 600000L;
bool db2p = FALSE;
#endif
#define BlynkAlertLED D7    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool LEDstatus = 0;
BlynkTimer timer;
int     timerNA         = 99;
int     resetAlertTimer = timerNA;
bool    havealerted     = 0;
int     humhavealerted  = 0;
bool    alertenable     = 1;
int     lowalertsetpoint    = 60;  //RedBarn 60   IDGreenhouse 50
int     highalertsetpoint   = 105;  //RedBarn 105  IDGreenhouse 90
int     humalert            = 42;
/**** PowerMonitoring *************************************************************************************************************/
bool hasVUSB(); 
bool hadVUSB = false;
WidgetLED pwLED(V9); //in Blynk
String batVolt;

#include <JsonParserGeneratorRK.h>
/**** Sensors *********************************************************************************************************************/
#include <OLED_Display_128X64.h>
bool SeeedOledStatus = 0; //set manually
void updateDisplay(int temp, int humidity, int pressure, String airQuality);

//#include <Grove_scd30_co2_sensor.h>  //I2C >> 0x61 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <SparkFun_SCD30_Arduino_Library.h>
bool SCD30Status;  //set automaticlly
SCD30 airSensor;
int tempSCD30, humiditySCD30, CO2SCD30;
int getSCDValues(int &tempSCD30, int &humiditySCD30, int &CO2SCD30);
void createSCD30Payload(int tempSCD30, int humiditySCD30, int CO2SCD30);

#include <Adafruit_BME280.h>   //I2C >> 0x76(default) or 0x77  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool BME280Status; //set automaticlly
bool testBME; //in readbme280() is this locking out the reading?? or is the reading actully going bad.
Adafruit_BME280 bme;
int temp, pressure, humidity;
int getBMEValues(int &temp, int &humidity, int &pressure);
void createEventPayload(int temp, int humidity, int pressure, String airQuality);

#include <Grove_Air_quality_Sensor.h>
bool AirQUStatus = 0;  //set automaticlly not working
#define AQS_PIN A2            //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
String getAirQuality();
AirQualitySensor aqSensor(AQS_PIN);
String qual; 
String airQuality;

#define DUST_SENSOR_PIN D4    //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool DustSenStatus = 0;  //set manually !!fix sensor reading interval before re-enabling this
void getDustSensorReadings();
unsigned long lastInterval;
unsigned long lowpulseoccupancy = 0;
unsigned long last_lpo = 0;
unsigned long duration;
float ratio = 0;
float concentration = 0;
/****Time***********************************************************************************************************************/
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
BLYNK_WRITE(V14) {
  humalert = param.asInt(); // assigning incoming value from pin to a variable
  EEPROM.put(100, humalert);
}
BLYNK_WRITE(V20) {
  alertenable = param.asInt(); // assigning incoming value from pin to a variable
  digitalWrite(BlynkAlertLED, alertenable); //turn on D7 blue LED if alert On
}


  
void setup() {
    //sensorInit();
    delay(50);
    setZone(); //Time.zone(-6);  //-5for old time
    Blynk.begin(auth);
    pwLED.off();
    //sensorInit();
    timer.setInterval(updateSensors, updateAllSensors); //sendAlert() relies on this
    delay(500);
    timer.setInterval(sendInfo2Blynk, sendinfo);
    // try this in firstRun() to get alert if in alert on reboot timer.setInterval(sendAlert2Blynk, sendAlert);
  
    EEPROM.get(1, lowalertsetpoint); //since firmware 2.0 this pops to -1 on fresh device instead of set from above
    EEPROM.get(50, highalertsetpoint);
    EEPROM.get(100, humalert);
    Blynk.virtualWrite(V20, alertenable);
    Blynk.virtualWrite(V10, lowalertsetpoint);
    Blynk.virtualWrite(V11, highalertsetpoint);
    Blynk.virtualWrite(V14, humalert);
    Blynk.notify(String(Location + " Starting... Allow 1 min for sensor readings to settle")); // +  myStr + ("°F"));
    pinMode(BlynkAlertLED, OUTPUT);
    #if Wiring_WiFi
    pinMode(PWR, INPUT); //PWR: 0=no USB power, 1=USB powered
	pinMode(CHG, INPUT); //CHG: 0=charging, 1=not charging
	#endif
    
    Particle.function("Debug2Particle", switchdb2p);
    Particle.function("RestartSensors", restartSensors);
    Particle.variable("1-Debug2Particle", db2p);
    Particle.variable("2-SIGStrength", SIG_STR);
    Particle.variable("3-SIGQuaility", SIG_QUA);
    Particle.variable("4-DayCounter", runDayCounter);
    Particle.variable("5-USBPower", hasVUSB);
    Particle.variable("6-BatteryVolt", batVolt);
    Particle.variable("7-BME280Ready", BME280Status);
    Particle.variable("8-SCD30Ready", SCD30Status);
    Particle.variable("9-AirQUReady", AirQUStatus);
    Particle.variable("Alert Enable", alertenable);
    Particle.variable("91-Low Alert", lowalertsetpoint);
    Particle.variable("92-High Alert", highalertsetpoint);
    Particle.variable("SensorReStart", sensorReInit);
    Particle.variable("testBME", testBME); //used for debuging >>will delete
    //Particle.variable("firstrunlockout", firstrunlockout);

    Wire.begin();
    sensorInit();
    
    lastInterval = millis();
    updateAllSensors();
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
    } //Blynk.run();
    timer.run();
    bool curVUSB = hasVUSB();
    if (curVUSB != hadVUSB) { hadVUSB = curVUSB;  if(curVUSB) {pwLED.on(); powerRegain();}   else{pwLED.off(); powerFail();}   }
    if (Blynk.connected() && !firstrunlockout) firstrun(); //get good sensor readings out to Blynk ASAP
    if (Time.hour() == 3 && Time.day() != timeUpdateDay) runOnceADay();  // update time and daylight savings
    if(DustSenStatus) { duration = pulseIn(DUST_SENSOR_PIN, LOW); lowpulseoccupancy = lowpulseoccupancy + duration; }
    if (millis() - lastSync > ONE_DAY_MILLIS) { Particle.syncTime(); lastSync = millis(); }
} //end loop
void powerRegain() {
    sensorInit(); //to try eliminating BME280 comunication failures
    if(alertenable) {
        Blynk.notify(String(Location + " Power Restored")); // +  myStr + ("°F"));
    }
}
void powerFail() {
    if(alertenable) {
        Blynk.notify(String(Location + " Power Failure")); // +  myStr + ("°F"));
    }
}
void firstrun() {
    //if (firstrunlockout == 0) { //if (millis() > FirstRunDelay && firstrunlockout == 0) {// moved into firstrun()  timer.setInterval(sendalert2blynk, sendAlert);
    getBMEValues(temp, pressure, humidity); //going to try putting this at the bottom of setup()
    sendinfo();
    timer.setInterval(sendAlert2Blynk, sendAlert);
    firstrunlockout = 1;
}
void updateAllSensors() {
    getSignalStrength();
    getSignalQuality();
    //getPowerInfo();
    if(BME280Status) getBMEValues(temp, pressure, humidity);
    if(SCD30Status) getSCDValues(tempSCD30, humiditySCD30, CO2SCD30);
    if(AirQUStatus) airQuality = getAirQuality(); //<<< move this out of BME280 stuff
    if(DustSenStatus) getDustSensorReadings(); lowpulseoccupancy = 0;
    if(SeeedOledStatus) updateDisplay(temp, humidity, pressure, airQuality);  // updateDisplay(temp, humidity, pressure, quality);
}

void sendinfo() {
      // this nesicary? getBMEValues(temp, pressure, humidity); //get current readings
      if(db2p) createEventPayload(temp, humidity, pressure, airQuality); // with BME280 sensor
      if(db2p && SCD30Status) createSCD30Payload(tempSCD30, humiditySCD30, CO2SCD30);
      Blynk.virtualWrite(V0, Time.format("%r %a %m/%d")); //"%r - %a %D"
      Blynk.virtualWrite(V1, temp);
      Blynk.virtualWrite(V2, humidity);
      Blynk.virtualWrite(V3, pressure);
      if(AirQUStatus) Blynk.virtualWrite(V4, qual);
      Blynk.virtualWrite(V5, SIG_STR);
      getPowerInfo(); //this gets readings and sends to particle
}

void sendAlert() {
    if(!havealerted && alertenable) {
        if(temp < lowalertsetpoint) {
            String myStr = String(temp);
            Blynk.notify("Low Temp Alert ~" + Location + "~ " + myStr + "°F");
            Blynk.virtualWrite(V0, Time.format("Low Temp %r"));
            Blynk.virtualWrite(V1, temp);
            havealerted = 1;
            if (!timer.isEnabled(resetAlertTimer)) { resetAlertTimer = timer.setTimeout(resendAlert2Blynk, [] () {havealerted = 0; resetAlertTimer = timerNA;} ); }
        }
        if(temp > highalertsetpoint) {
            String myStr = String(temp);
            Blynk.notify("High Temp Alert ~" + Location + "~ " + myStr + "°F");
            Blynk.virtualWrite(V0, Time.format("High Temp %r"));
            Blynk.virtualWrite(V1, temp);
            havealerted = 1;
            if (!timer.isEnabled(resetAlertTimer)) { resetAlertTimer = timer.setTimeout(resendAlert2Blynk, [] () {havealerted = 0; resetAlertTimer = timerNA;} ); }
        }
    }
    if(alertenable) {
        if(humidity > humalert && humhavealerted == 0) {
            String myStr = String(humidity);
            Blynk.notify("1st Humidity Alert ~" + Location + "~ " + myStr + "%");
            Blynk.virtualWrite(V0, Time.format("Humidity Alert %r"));
            Blynk.virtualWrite(V2, humidity);
            humhavealerted = 1;
            timer.setTimeout(5000L, [] () { terminal.print(Time.format("%D %I:%M%p - ")); terminal.print("1st Humidity Alert: "); terminal.println(humidity); terminal.flush();  } );
        }
        int nexthumalert = humalert + 5;
        if(humidity > nexthumalert && humhavealerted == 1) {
            String myStr = String(humidity);
            Blynk.notify("2nd Humidity Alert ~" + Location + "~ " + myStr + "%");
            Blynk.virtualWrite(V0, Time.format("Humidity Alert %r"));
            Blynk.virtualWrite(V2, humidity);
            humhavealerted = 2; 
            timer.setTimeout(5000L, [] () { terminal.print(Time.format("%D %I:%M%p - ")); terminal.print("2nd Humidity Alert: "); terminal.println(humidity); terminal.flush();  } );
        }
    }
    int humresetalerts = humalert - 5;
    if(humhavealerted != 0 && humidity < humresetalerts) {
        humhavealerted = 0;
        terminal.print(Time.format("%D %I:%M%p - ")); terminal.print("LowHumididty-AlertReset"); terminal.flush();
    }
}
/////////************* **********/////////
//           Sensor Functions           //
/////////************* **********/////////
void sensorInit() {
    sensorReInit++;
  // Configure the dust sensor pin as an input
  pinMode(DUST_SENSOR_PIN, INPUT);
  if (aqSensor.init()) {
      //publishQueue.publish("DustSen", "Ready", PRIVATE | WITH_ACK);
    if(db2p) Particle.publish("AirQUSen", "Ready", PRIVATE | WITH_ACK); delay(1000); //if(db2p && Particle.connected()) {bool result = Particle.publish("DustSen", "Ready", PRIVATE | WITH_ACK); delay(4000);}
    //AirQUStatus = 1; setting manually
  }
  else {
    if(db2p) Particle.publish("AirQUSen", "Error"); delay(1000);
    //AirQUStatus = 0; setting manually
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
    if(SeeedOledStatus) { // this needs to be set above this isn't working >> (SeeedOled.init()) {
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
        if(db2p) Particle.publish("SeeedOled", "Ready", PRIVATE | WITH_ACK); delay(1000);
        SeeedOledStatus = 1;
    }
    else {
        if(db2p) Particle.publish("SeeedOled", "Error"); delay(1000);
        SeeedOledStatus = 0; 
    }
}
int getSCDValues(int &tempSCD30, int &humiditySCD30, int &CO2SCD30) {
    if (airSensor.dataAvailable()) { //SCD30 data
        tempSCD30 = (airSensor.getTemperature() * 1.8F + 32); //, 1);
        humiditySCD30 = airSensor.getHumidity(); //, 1);
        CO2SCD30 = airSensor.getCO2();
        return 1;
    }
    else { Particle.publish("SCD30/debug", "Error in getSCDValues()"); return -1; }
}
int getBMEValues(int &temp, int &pressure, int &humidity) {
    if(db2p) {  //if(temp < 0) Particle.publish("BME280/debug", "Error in getBMEValues()");
        int reading = bme.readTemperature();
        if(reading == 0) { testBME = 0; /*BME280Status = 0;*/ String mySTR = String(testBME); Particle.publish("BME280/debug/unplugged", mySTR, PRIVATE); } //not working
        if(reading != 0) testBME = 1; /*BME280Status = 1;*/
    }
    temp = (int)(bme.readTemperature() * 1.8F + 32); //convert temp to fahrenheit
    pressure = (int)(bme.readPressure() / 100.0F);
    humidity = (int)bme.readHumidity();

  return 1;
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
void getDustSensorReadings() { //fix SENSOR_READING_INTERVAL 
  // This particular dust sensor returns 0s often, so let's filter them out by making sure we only
  // capture and use non-zero LPO values for our calculations once we get a good reading.
  if (lowpulseoccupancy == 0) {
    lowpulseoccupancy = last_lpo;
  }
  else {
    last_lpo = lowpulseoccupancy;
  }

  //ratio = lowpulseoccupancy / (SENSOR_READING_INTERVAL * 10.0);                   // Integer percentage 0=>100
  concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve

  Serial.printlnf("LPO: %d", lowpulseoccupancy);
  Serial.printlnf("Ratio: %f%%", ratio);
  Serial.printlnf("Concentration: %f pcs/L", concentration);
}

void createSCD30Payload(int tempSCD30, int humiditySCD30, int CO2SCD30){
    JsonWriterStatic<256> jw; {
        JsonWriterAutoObject obj(&jw);
        
        jw.insertKeyValue("Temp", tempSCD30);
        jw.insertKeyValue("Humidity", humiditySCD30);
        jw.insertKeyValue("CO2", CO2SCD30);
    }
    Particle.publish("SCD30/readings", jw.getBuffer(), PRIVATE);
}
void createEventPayload(int temp, int humidity, int pressure, String airQuality) {
    JsonWriterStatic<256> jw; {
        JsonWriterAutoObject obj(&jw);

        jw.insertKeyValue("temp", temp);
        jw.insertKeyValue("humidity", humidity);
        jw.insertKeyValue("pressure", pressure);
        jw.insertKeyValue("air-quality", airQuality);

        if (lowpulseoccupancy > 0) {
            jw.insertKeyValue("dust-lpo", lowpulseoccupancy);
            jw.insertKeyValue("dust-ratio", ratio);
            jw.insertKeyValue("dust-concentration", concentration);
        }
  }
  Particle.publish("BME280/readings", jw.getBuffer(), PRIVATE);

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

int switchdb2p(String command) {
    if(command == "1") {
        db2p = 1;
        return 1;
    }
    else if (command == "0") {
        db2p = 0;
        return 0;
    }
    else return -1;
} 
int restartSensors(String command) {
    if(command == "1") {
        sensorInit();
        return 1;
    }
    else return -1;
}
/*void getSignalStrength() {
    wifiSTR = WiFi.RSSI().getStrength();
    wifiQUA = WiFi.RSSI().getQualityValue();
}*/
int getSignalStrength() {
    #if Wiring_WiFi
        SIG_STR = WiFi.RSSI().getStrength();
    #endif
    #if Wiring_Cellular
        SIG_STR = Cellular.RSSI().getStrength();
    #endif
    return SIG_STR;
}
int getSignalQuality() {
    #if Wiring_WiFi
        SIG_QUA = WiFi.RSSI().getQuality();
    #endif
    #if Wiring_Cellular
        SIG_QUA = Cellular.RSSI().getQuality();
    #endif
    return SIG_QUA;
}
bool hasVUSB() { //checks if power supplied at USB this runs in loop() - bool curVUSB = hasVUSB(); 
    uint32_t *pReg = (uint32_t *)0x40000438; // USBREGSTATUS

    return (*pReg & 1) != 0;
}

void getPowerInfo() {
    #if Wiring_WiFi
    float voltage = analogRead(BATT) * 0.0011224;
    batVolt = String(voltage, 3); //sprintf("Voltage=%.1f", voltage);
	char buf[128];
	snprintf(buf, sizeof(buf), "voltage=%.1f PWR=%d CHG=%d", voltage, digitalRead(PWR), digitalRead(CHG));
    if(db2p) Particle.publish("PowerInfo", buf, PRIVATE);
	/*if (strcmp(buf, lastMsg) != 0 && millis() - lastPublish > 2000) {
		Particle.publish("battery", buf, PRIVATE);
		Log.info(buf);
		strcpy(lastMsg, buf);
		lastPublish = millis();
	}*/
	#endif
	#if Wiring_Cellular
	//batVolt = fuel.getVCell();
	#endif
}

void runOnceADay() {
    timeUpdateDay = Time.day();
    runDayCounter++;
    Particle.syncTime();
    setZone();
}
void setZone() {
	int month = Time.month();
	int day = Time.day();
	int weekday = Time.weekday();
	int previousSunday = day - weekday + 1;

	if (month < 3 || month > 11) {
		Time.zone(-6);
	}else if (month > 3 && month < 11) {
		Time.zone(-5);
	}else if (month == 3) {
		int offset = (previousSunday >= 8)? -5 : -6;
		Time.zone(offset);
	} else{
		int offset = (previousSunday <= 0)? -5 : -6;
		Time.zone(offset);
	}
}
