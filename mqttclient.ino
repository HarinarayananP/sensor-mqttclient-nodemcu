#define DHTPIN   D2
#define STEPSIZE 1.22E-03
#define LED D0
#define DHTTYPE DHT11

#include "FS.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include "PMS.h"
#include "SoftwareSerial.h"
#include "ADC_MAX147.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include "CO2Sensor.h"

float humidity, temperature;
enum channels {oz_ch = 0, temp_ch, no2_ch, met_ch, co_ch, so2_ch, h2s_ch, amm_ch};
float getEnvCorrectRatio(float,float);


// WiFi Credentials
const char* ssid = "********"; // WiFi SSID
const char* password = "**************"; // WiFi Password

// IBM Watson IOT - Device Credentials
#define ORG "******" 
#define DEVICE_TYPE "ESP32" 
#define DEVICE_ID "**********" 
#define TOKEN "**********" 


char server[] = ORG ".messaging.internetofthings.ibmcloud.com";
char pubTopic1[] = "iot/sensor_node/";
char authMethod[] = "use-token-auth";
char token[] = TOKEN;
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;


long lastMsg = 0;
char msg[150];
int loop_count = 0;

WiFiClient espClient;

SoftwareSerial Serial2(D3, D4); // RX, TX

DHT_Unified dht(DHTPIN, DHTTYPE);
ADCmax147 zadcs147(SS);

PMS pms(Serial2);
PMS::DATA data;

CO2Sensor co2Sensor(A0, 0.99, 100);



void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

PubSubClient client(AWS_endpoint, 1883, callback, espClient); //set MQTT port number to 8883 as per //standard


// Connect to WIFI.
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  espClient.setBufferSizes(512, 512);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  // Wait until WIFI Connection is established.
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected! IP address: ");
  Serial.println(WiFi.localIP());
}

// Connect to MQTT and Loop Until Connection establishes. 
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(clientId, authMethod, token)) {
      Serial.println("Connected!");
      // Device Connected log MQTT Publish goes here..
    } else {
      Serial.print("Connection Failed :x | RC:");
      Serial.print(client.state());
      Serial.println("Retrying in 5 seconds...");

      char buf[256];
      espClient.getLastSSLError(buf, 256);
      Serial.print("WiFiClientSecure SSL error: ");
      Serial.println(buf);
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.println("Initialising...");
  Serial.begin(9600);
  Serial.setDebugOutput(true);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  setup_wifi();
  Serial.print("Heap: "); Serial.println(ESP.getFreeHeap());
  Serial2.begin(9600);
  Serial.println("Final steps...");
  SPI.begin();
  Serial.print(" 1");
  SPI.beginTransaction(SPISettings(3000000, MSBFIRST, SPI_MODE0));
  Serial.print(" 2");
  zadcs147.begin();
  Serial.print(" 3");
  co2Sensor.calibrate();
  Serial.println("Initialisation Completed.");
}

void loop() {

  // Verify MQTT Connection
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  

  Serial.println("Waiting to read data from PM Sensors...");
  while(!pms.read(data));

  /*=========================================
  1. Temperature Sense TC1047
  Sensitivity 10mV/*C  and 500mV at 0*C  
  ==========================================*/
  unsigned int adc_result = zadcs147.ADC_readch(temp_ch);
  float temp = (float)(adc_result * (STEPSIZE-(212E-06)));   // Calibration Factor
  temp = (temp - 0.5) / 0.01;
  Serial.print(adc_result);
  Serial.print("\t");
  Serial.print("Temp: ");
  Serial.println(temp,2);
  publish_data(temp, "temp");

  /*=========================================
  2. NO2 Sense SPEC-3SP-NO2
  Sense Range 20ppm, VBias : -200mV ; Sensitivity: 22.18nA/ppm
  Gas Concentration Cx(ppm) = (Vgas - Vgas0)/M
  Vgas= V out at gas level; Vgas0 = V out @ 0 gas level; Sensor Calibration Factor M (V/ppm)
  M = Sensitivity_code(nA/ppm) x TIA Gain(kV/A) x 10E-9(A/nA) x 10E03(V/KV)
  M =  22.18x499x10e-09x10e03 = 0.0110678  
  ==========================================*/
  adc_result = zadcs147.ADC_readch(no2_ch);
  Serial.print(adc_result);
  float Vgas = (float)(adc_result * STEPSIZE);
  Serial.print("\t Vgas=");
  Serial.print(Vgas,4);
  float Vgas0 = 0.9685;     // Need to get from 0 ppm Calibration Environment; now given as Vref+Voff
  float no2 = (Vgas-Vgas0)*(90.352)*0.05*0.333;
   no2 = no2*1000;
  Serial.print("\t");
  Serial.print("NO2 ppb: ");
  if(no2 < 0.0)
    no2 *= -1.0;
  Serial.println(no2,3);
  publish_data(no2, "no2");  

  /*=========================================
  3. H2S Sense SPEC-3SP-H2S
  Sense Range 50ppm, VBias : 0mV ; Sensitivity: 284.63nA/ppm
  Gas Concentration Cx(ppm) = (Vgas - Vgas0)/M
  Vgas= V out at gas level; Vgas0 = V out @ 0 gas level; Sensor Calibration Factor M (V/ppm)
  M = Sensitivity_code(nA/ppm) x TIA Gain(kV/A) x 10E-9(A/nA) x 10E03(V/KV)
  M =  284.63 x 49.9 x 10e-09 x 10e03 = 0.014203  ;
  ==========================================*/
  adc_result = zadcs147.ADC_readch(h2s_ch);
  Serial.print(adc_result);
  Vgas = (float)(adc_result * STEPSIZE);
  Serial.print("\t Vgas=");
  Serial.print(Vgas,4);
  Vgas0 = 0.97;     // Need to get from 0 ppm Calibration Environment; now given as Vref+Voff
  float h2s = (Vgas-Vgas0)*(70.4075);
  Serial.print("\t");
  Serial.print("H2S ppm: ");
  if(h2s < 0.0)
    h2s *= -1.0;
  Serial.println(h2s,3);
  publish_data(h2s, "h2s");

  /*=========================================
  4. CO Sense  TGS-5042
  Sensitivity : 1.59nA/ppm
  Is = (Vout - 1.00) / 3.13
  ==========================================*/
  adc_result = zadcs147.ADC_readch(co_ch);
  Serial.print(adc_result);
  Serial.print("\t");
  float co = (float)(adc_result * STEPSIZE);
  Serial.print("Vgas:");
  Serial.print(co,4);
  co = (co - 0.00)/3.13;   // Is
  co = co/1.59;  // ppm            // scaling may require from nA to uA 
  Serial.print("\t CO ppm: ");
  if(co < 0.0)
    co *= -1.0;
  Serial.println(co,3);
  publish_data(co, "co");  
  
  /*=========================================
  5. SO2 Sense FECS43-20
  Is= ((VOUT[Gas] - VOUT[Air]) / I-V conversion amp. factor) x 106
  Sensitivity : 500nA(+/-100nA) per ppm ;  0-20ppm range  
  ==========================================*/
  adc_result = zadcs147.ADC_readch(so2_ch);
  Serial.print(adc_result);
  Vgas = (float)(adc_result * STEPSIZE);
  Serial.print("\t Vgas: ");
  Serial.print(Vgas,4);
  Vgas0 = 0.001;     // Calibration Reference @ 0ppm 
  float so2 = ((Vgas - Vgas0)/200000)*10E6;   // in uA
  so2 = (so2 / 0.5)*0.01;   // 
  so2 = so2*1000;
  Serial.print("\t");
  Serial.print("SO2 ppb: ");
  if(so2 < 0.0)
    so2 *= -1.0;
  Serial.println(so2,3);
  publish_data(so2, "so2");  

  /*=========================================
  6. Ozone Sense MQ131
  valueRL = 1Mohm
  ==========================================*/
  adc_result = zadcs147.ADC_readch(oz_ch);
  Serial.print(adc_result);
  float vRL = (float)(adc_result * STEPSIZE);
  float rs = (5.0/vRL - 1.0)* 1000000;   // RL = 1Mohm
  Serial.print("\t Rs: ");
  Serial.print(rs,4);
  // Compute the ratio Rs/R0 and apply the environmental correction
  float valueR0 = 498000;    // Rs value for calibration value at 0 ppm
  float ratio = rs / valueR0 * getEnvCorrectRatio(humidity,temperature);
  // R^2 = 0.9985
  // Use this if you are monitoring low concentration of O3 (air quality project)
  //float ozone = (8.1399 * pow(ratio, 2.3297));
  float ozone = (8.1399 * pow(ratio, 2.3297));
  Serial.print("\t");
  Serial.print("Ozone ppb: ");
  if(ozone < 0.0)
    ozone *= -1.0;
  Serial.println(ozone,3);
  publish_data(ozone, "ozone");
  
  //7. Methane Sense
  adc_result = zadcs147.ADC_readch(met_ch);
  Serial.print(adc_result);
  vRL = (float)(adc_result * STEPSIZE);
  rs = (5.0/vRL - 1.0)* 1000;   // RL = 1Kohm
  Serial.print("\t Rs:");
  Serial.print(rs,4);
  Serial.print("\t");
  valueR0 = 18062.14;                    // Calibration Value at 0 methane
  float methane = rs/valueR0;
  Serial.print("Methane ppm: ");
  if(methane < 0.0)
    methane *= -1.0;
  Serial.println(methane,3);
  publish_data(methane, "methane"); 
  
  //8. Ammonia Sense
  adc_result = zadcs147.ADC_readch(amm_ch);
  Serial.print(adc_result);
  vRL = (float)(adc_result * STEPSIZE);
   rs = (5.0/vRL - 1.0)* 1000;   // RL = 1Kohm
  Serial.print("\t Rs:");
  Serial.print(rs,4);
  Serial.print("\t"); 
  valueR0 = 46107.59;               // Calibration Value at 0 ammonium
  float amm = rs/valueR0;
  Serial.print("Ammonia ppm: ");
  if(amm < 0.0)
    amm *= -1.0;
  Serial.println(amm,3);
  publish_data(amm, "ammonia");
  
  //9. DHT11 Humidity & Temp
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    Serial.print("DHT Temperature: ");
    temperature = event.temperature;
    Serial.print(temperature);
    Serial.println(" *C");
  }
  
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println("Error reading humidity!");
  }
  else {
    Serial.print("Humidity: ");
    humidity = event.relative_humidity;
    Serial.print(humidity);
    Serial.println("%");
  }
  Serial.println("\n");
  publish_data(humidity);
  
  //10. CO2 Sensor
  int val = co2Sensor.read();
  Serial.print("CO2 value: ");
  Serial.println(val);
  publish_data(val, "humidity");

  
  // DELAY FOR 10 MINUTES
//  ESP.deepSleep(60*1000*1000);  // ESP Hardware Deepsleep for power saving.
delay(600e3);
}



/**
 * Get correction to apply on Rs depending on environmental
 * conditions
 */
float getEnvCorrectRatio(float humidityPercent,float temperatureCelsuis) {
   // Select the right equation based on humidity
  // If default value, ignore correction ratio
  if(humidityPercent == 60 && temperatureCelsuis == 20) {
    return 1.0;
  }
  // For humidity > 75%, use the 85% curve
  if(humidityPercent > 75) {
    // R^2 = 0.9986
    return -0.0141 * temperatureCelsuis + 1.5623;
  }
  // For humidity > 50%, use the 60% curve
  if(humidityPercent > 50) {
    // R^2 = 0.9976
    return -0.0119 * temperatureCelsuis + 1.3261;
  }

  // Humidity < 50%, use the 30% curve
  // R^2 = 0.996
  return -0.0103 * temperatureCelsuis + 1.1507;
 }

void publish_data(float value, char* name){
  String payload = "{\"value\":\"";
  payload += value;
  payload += "\"}";

  if (client.publish(pubTopic1 + name, (char*) payload.c_str())) {
      Serial.println("Published");
  } else {
      Serial.println("Publish Failed");
  }
  Serial.print("Heap: "); Serial.println(ESP.getFreeHeap()); //Low heap can cause problems
}