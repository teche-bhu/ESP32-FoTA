/*
  Packages Required : ArduinoJSON Library 
                    : DHTT Sensor Library
                    : Arduino Http Client Library
                    : Thingsboard IoT Platform Library
  Developed & Maintained by : Teche-Solutions
  Version : 0.12.2020
  Business Requirements : Publish Temperature and Humidity Microclimatic telemetry, CO, LPG, SMOKE PPM, Door/Windows
  status, Patient Detection to IoT Platform ThingsBoard Using Https protcol 
*/

#include <WiFi.h>           //Wifi-Library for ESP32
#include <HTTPClient.h>     //HTTP-Client Library for GET, PUT, POST, PATCH and DELETE Resources
/*------Mdash-Application https://mdash.net/------- */
#define MDASH_APP_NAME "teche-device1"
#include <mDash.h>
#define DEVICE_PASSWORD "nXQK7gjeHDQ99iBMlkemr8Q"
/*------Mdash Application---------*/

#include "DHT.h"            //DHTxx Sensor Library Header File
#include <ArduinoJson.h>    //Arduino JSON Library
#define DHTPIN 4            //GPIO PIN -4 (D4) of ESP32 Model : WROOM32
#define DHTTYPE DHT22       // DHT Sensor Type
DHT dht(DHTPIN, DHTTYPE);   //Initilize DHT sensor with GPIO and Sensor Type
/*-----PIR Sensor-----*/
#define PIR_MOTION_SENSOR 27 //PIN-27 to (data pin) for PIR - HCSR505 
#define LED  26//LED is connected to PIN 26 of ESP32
String patientStatus;
String doorStatus;
/* PIR sensor Ends here -----*/

/* Door sensor --- */
const int switchPin = 23;

/* ---- Door sensor ends here  --- */


/*
MQ2 Sensor code section
*/
const int calibrationLed = 13;                      //when the calibration start , LED pin 13 will light up , off when finish calibrating
const int MQ_PIN=34;                                //define which analog input channel you are going to use
int RL_VALUE=5;                                     //define the load resistance on the board, in kilo ohms
float RO_CLEAN_AIR_FACTOR=9.83;                     //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                    //which is derived from the chart in datasheet
 
/***********************Software Related Macros************************************/
int CALIBARAION_SAMPLE_TIMES=50;                    //define how many samples you are going to take in the calibration phase
int CALIBRATION_SAMPLE_INTERVAL=500;                //define the time interal(in milisecond) between each samples in the
                                                    //cablibration phase
int READ_SAMPLE_INTERVAL=50;                        //define how many samples you are going to take in normal operation
int READ_SAMPLE_TIMES=5;                            //define the time interal(in milisecond) between each samples in 
                                                    //normal operation
 
/**********************Application Related Macros**********************************/
#define         GAS_LPG             0   
#define         GAS_CO              1   
#define         GAS_SMOKE           2    
 
/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms


/*  End Here MQ2 --------------*/


const char* ssid = "forest";  //SSID Name to which ESP32 (Teche mote is about to connect to )
const char* password = "sidver1234";  //SSID Password
//String ThingsBoardAcccessToken = "techedevicemdash";
//Define Path for IoT Platform/Teche Platform for Posting the data
const char* serverName = "https://demo.thingsboard.io/api/v1/techepatient01/telemetry";
//const char* serverName = "http://192.168.0.101:9090/api/v1/techepatient01/telemetry";
/*
  The following variables are unsigned longs because the time, measured in
 milliseconds, will quickly become a bigger number than can be stored in an int.
*/
unsigned long lastTime = 0;

// Set timer to 30 seconds (30000) to publish telemetry
unsigned long timerDelay = 10000;

//Prepare JSON Document with a buffer size of 1024 bytes

DynamicJsonDocument sensor_data(2048);

void setup() 
{
  Serial.begin(115200); //Serial Port debug message baud rate
 
  
  dht.begin();   //Start the DHT11 sensor

   /* ------ Door Sensor -----*/

   pinMode(switchPin, INPUT_PULLUP);

   pinMode(LED_BUILTIN, OUTPUT);

   digitalWrite(switchPin, HIGH);

  /* ---- Door Sensor Ends Here---------*/
   
   
   /*-------PIR Sensor---------*/
    
    pinMode(PIR_MOTION_SENSOR, INPUT);
    pinMode(LED,OUTPUT);
   
   /* PIR Sensor Ends here  */
   /* -------------MQ2----  */

  Ro = MQCalibration(MQ_PIN);                         //Calibrating the sensor. Please make sure the sensor is in clean air         
  digitalWrite(calibrationLed,LOW);              
  Serial.print("Ro= ");
  Serial.print(Ro);
  Serial.println("kohm");
  delay(1000);

 /* MQ2 Ends Here ------*/
  
  
  WiFi.begin(ssid, password);   // Start the Wi-Fi
   /*------------Mdash------*/
   mDashBegin(DEVICE_PASSWORD);
  /*-----------Mdash--------*/
  Serial.println("Connecting");
  /*
   * 
   * Check Wifi is connected to Router/Access Point is connected ?
   * 
   */
  
  while(WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.println("Not Connected with Access Point ....");
  }

  //If connected print the IP of Teche Mote assigned by Router
  
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Timer Delay : 30 Second for sending second unit");
}

void loop() 
{
   //delay(2000);
  
  /* PIR sensor ----*/
  int sensorValue = digitalRead(PIR_MOTION_SENSOR);
 
  if(sensorValue == HIGH)
  {
    patientStatus = "Detected";
    digitalWrite(LED,HIGH);
    Serial.println(patientStatus);
  }
  else
  {
    patientStatus = "NOT Detected";
    digitalWrite(LED,LOW);
    Serial.println(patientStatus);
  }
  /*-PIR Sensor- Ends Here*/

  /* ---------- Door Sensor Starts Here ------ */
int doorsensorValue = digitalRead(switchPin);

if(doorsensorValue == LOW)
{
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Door is closed");
  doorStatus = "Closed";
}

else
{
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Door is Open");
  doorStatus = "Open";
}

/* ----- Door Sensor Ends Here -------------*/
  
  /* MQ2 Section Starts Here---*/
  long iPPM_LPG = 0;
  long iPPM_CO = 0;
  long iPPM_Smoke = 0;
  
  iPPM_LPG = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
  iPPM_CO = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
  iPPM_Smoke = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);
  
   
   Serial.println("Concentration of gas ");
   Serial.print("LPG: ");
   Serial.print(iPPM_LPG);
   Serial.println(" ppm");   
   
   Serial.print("CO: ");
   Serial.print(iPPM_CO);
   Serial.println(" ppm");    

   Serial.print("Smoke: ");
   Serial.print(iPPM_Smoke);
   Serial.println(" ppm");  

  /* MQ2 section of code ends here -----*/
   float h = dht.readHumidity();   //Reading Humidity Data
   float t = dht.readTemperature(); // Reading Temperature Data
   Serial.println(h);
   Serial.println(t);
   sensor_data["Location"] = "Pune";   //Location : ESP32
   sensor_data["Temperature"] = t;  //Update the Temperature value in JSON Key
   sensor_data["Humidity"] = h;   ////Update the Humidity value in JSON Key
   sensor_data["LPGPPM"] = iPPM_LPG;      //PPM - Particle per million
   sensor_data["COPPM"] = iPPM_CO;
   sensor_data["SMOKEPPM"] = iPPM_Smoke;
   sensor_data["PatientPresence"] = patientStatus;
   sensor_data["Door/Window"] = doorStatus;
   
   //Check if sensor fails to provide value ... will return nan
  if (isnan(h) || isnan(t)) 
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  
  //Send an HTTP POST request every 30 seconds
  if ((millis() - lastTime) > timerDelay) 
  {
    //Check WiFi connection status : If connected 
    if(WiFi.status()== WL_CONNECTED)
    {
      HTTPClient http;
      // Your Domain name with URL path or IP address with path
      http.begin(serverName);
      // If you need an HTTP request with a content type: application/json, use the following:
      http.addHeader("Content-Type", "application/json");
      //
      String json;
      /*
       * serializeJson() serializes a JsonDocument to create a minified JSON document, 
       * i.e. a document without spaces or line break between values.
       * 
       */
      serializeJson(sensor_data, json);
      //Post data to the IoT Platform/Teche IoT Platform
      //Return code : Response code : 2.X.X - Success , 4.X.X - Error
      int responsecode = http.POST(json);
      Serial.print("HTTP Response code: ");
      Serial.println(responsecode);  // Print the Response code
      // Free resources Close the resuest
      http.end();
    }
    else 
    {
      Serial.println("WiFi Disconnected");
    }
    
    lastTime = millis();
  }
}
/* MQ2 functinons starts here */

/******************* MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(4096-raw_adc)/raw_adc));
}
 
/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/ 
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro                                        
  return val;                                                      //according to the chart in the datasheet 

}
 
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
long MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
 
  return 0;
}
 
/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
long  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

/* ------------------- MQ2 functions Ends Here------------------ */
