// Library Imports
#include <DHT.h>
#include <SdsDustSensor.h>
#include <ESP8266WiFi.h>

//**********************************************************************
// Pin Definitions
#define SDS_rxPin 5
#define SDS_txPin 4
#define DHTPIN 2
#define zInput 17                       // Analog Pin
const int selectPins[3] = {14, 12, 13}; // S0~14, S1~12, S2~13
const int calibrationLed = 15;

//**********************************************************************
// Constant Definitions

// ---WiFi Related---
String apiKey = "NZLT8WRC6OKPA3WT";
const char *ssid = "tkmc";
const char *pass = "wifi1234";
const char *server = "api.thingspeak.com";

// ---Dust Sensor Related---
float pm25 = 0.0;
float pm10 = 0.0;

// ---Temperature Readings---
float temp_ = 0.0;
float humid_ = 0.0;
float heatIndex = 0.0;
bool asFahrenheit = false;

// ---Gas Sensors related---
const int numSensors = 2;
int sensorReadings[numSensors] = {0.0};
int RL_VALUE = 5;
float Ro =  10;
#define CALIBARAION_SAMPLE_TIMES 20  // counts
#define CALIBRATION_SAMPLE_INTERVAL 200  // milliseconds
#define RO_CLEAN_AIR_FACTOR 9.83

// ---Time Delays---
int timeBwTmsn = 20000;

//**********************************************************************
// Object Initializations
SdsDustSensor sds(SDS_rxPin, SDS_txPin);
WiFiClient client;
DHT dht(DHTPIN, DHT22);

//**********************************************************************
// General Functions
void connectToWIfi()
{
    Serial.print("Connecting to Wifi with SSID ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWifi Successfully Connected");
}

void sendData()
{
        String postStr = apiKey;
        postStr += "&field1=";
        postStr += String(temp_);
        postStr += "&field2=";
        postStr += String(humid_);
        postStr += "&field3=";
        postStr += String(pm25);
        postStr += "&field4=";
        postStr += String(pm10);
        postStr += "&field5=";
        postStr += String(sensorReadings[0]);
        postStr += "&field6=";
        postStr += String(sensorReadings[1]);
        postStr += "\r\n\r\n";

    if (client.connect(server, 80))
    {
        client.print("POST /update HTTP/1.1\n");
        client.print("Host: api.thingspeak.com\n");
        client.print("Connection: close\n");
        client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n");
        client.print("Content-Type: application/x-www-form-urlencoded\n");
        client.print("Content-Length: ");
        client.print(postStr.length());
        client.print("\n\n");
        client.print(postStr);
    }
    else
    {
        Serial.println("Could not reach ThingsSpeak.com!!");
    }

    client.stop();
}
 
void printSensorValuestoSerial()
{
    while(Serial.available()){   
        Serial.print("Temperature: ");
        Serial.print(temp_);
        Serial.print(" degrees Celsius, Humidity: ");
        Serial.print(String(humid_) + "\n");
        Serial.print("PM2.5: ");
        Serial.print(pm25);
        Serial.print(" PM10: ");
        Serial.println(pm10);
        Serial.print("Analog Gas Sensor values : ");
        for (byte i = 0; i < numSensors; i++)
        {
            Serial.print(String(sensorReadings[i]) + "  ");
        }
    }
}

void selectMuxPin(byte pin)
{
    for (int i = 0; i < 3; i++)
    {
        if (pin & (1 << i))
            digitalWrite(selectPins[i], HIGH);
        else
            digitalWrite(selectPins[i], LOW);
    }
}

// Gas Sensor Calibration Functions
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

float MQCalibration(int mq_pin){
  int i;
  float val=0;
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(zInput));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro                                        
  return val;                                                      //according to the chart in the datasheet 

}

// Sensor Functions
float getTemperature()
{
    // if asFahrenheit is False, then function returns in deg C.
    return dht.readTemperature(asFahrenheit);
}

float getHumidity()
{
    return dht.readHumidity();
}

float getHeatIndex()
{
    return dht.computeHeatIndex(temp_, humid_, asFahrenheit);
}

void getSensorReadings()
{
    sds.wakeup();
    delay(2000);
    // Gas Sensor Values
    delay(200);
    for (byte i = 0; i < numSensors; i++)
    {
        selectMuxPin(i);
        delay(100);
        sensorReadings[i] = analogRead(zInput);
    }

    // DHT22 temperature and Humidity and Heat Index
    temp_ = getTemperature();
    humid_ = getHumidity();
    heatIndex = getHeatIndex();
    if (isnan(temp_) || isnan(humid_))
    {
        Serial.println("Failed to read from DHT Sensor!!");
    }

    // Dust Sensor
    PmResult pm = sds.queryPm();
    if (pm.isOk())
    {
        pm25 = pm.pm25;
        pm10 = pm.pm10;
    }
    else
    {
        Serial.println("Could not read values from Dust sensor, reason: " + pm.statusToString());
    }
    sleepDustSensor();
}

void sleepDustSensor()
{

    WorkingStateResult state = sds.sleep();
    if (state.isWorking())
    {
        Serial.println("Problem with sleeping the sensor.");
    }
    else
    {
        Serial.println("Sensor is sleeping");
    }
}

//**********************************************************************
// setup
void setup()
{
    Serial.begin(115200);
    sds.begin();
    dht.begin();
    pinMode(calibrationLed,OUTPUT);

    digitalWrite(calibrationLed,HIGH);
    Ro = MQCalibration(zInput);  //Calibrating,make sure the sensor is in clean air         
    digitalWrite(calibrationLed,LOW);  

    // Printing Dust Sensor Information
    Serial.println(sds.queryFirmwareVersion().toString());  // prints firmware version
    Serial.println(sds.setQueryReportingMode().toString()); // ensures sensor is in 'query' reporting mode

    // Connecting to wifi
    connectToWIfi();

    // Initializing the states of Multiplexer Select selectPins
    for (int i = 0; i < 3; i++)
    {
        pinMode(selectPins[i], OUTPUT);
        digitalWrite(selectPins[i], HIGH);
    }
}

//**********************************************************************
// loop
void loop()
{
    // Getting Sensor values
    getSensorReadings();

    // Printing Sensor values
    printSensorValuestoSerial();

    // Send data to thingspeak
    sendData();

    // Delay between each transmission - Sensor delay
    delay(timeBwTmsn);
}
