/********************************************************************/
// First we include the libraries
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <PID_v1.h>
/********************************************************************/
// Data wire is plugged into pin 2 on the Arduino 
#define ONE_WIRE_BUS 2
// Relay is conected to pin 6 on the arduino
#define RELAY_PIN 13
/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
/********************************************************************/ 

//Define Variables need for PID
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp=1750, Ki=100, Kd=0;
// Initialize PID library
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int WindowSize_relay = 5000;
int WindowSize_Tsens = 200;
unsigned long windowStartTime_relay;
unsigned long windowStartTime_Tsens;

void setup(){
  windowStartTime_relay = millis();
  windowStartTime_Tsens = millis();
  Serial.begin(9600); 
  Serial.println("Sous vide starter op");
  pinMode(RELAY_PIN, OUTPUT);
  //initialize the variables we're linked to
    Setpoint = 58;

    //tell the PID to range between 0 and the full window size
    myPID.SetOutputLimits(0, WindowSize_relay);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
  
  
  // Start up the library 
  sensors.begin(); 
}

void loop(void){
  if(millis() - windowStartTime_relay > WindowSize_relay){ //time to shift the Relay Window
    windowStartTime_relay += WindowSize_relay;
    Serial.print("Temperaturen er: ");
    Serial.print(Input);
    Serial.print("  ...  ");
    Serial.print("Output er: ");
    Serial.print(Output/5000*100);
    Serial.println("%");
  }
  if(millis() - windowStartTime_Tsens > WindowSize_Tsens){
    windowStartTime_Tsens += WindowSize_Tsens;
  }
  
  if(millis() - windowStartTime_Tsens > WindowSize_Tsens){
    sensors.requestTemperatures(); // Send the command to get temperature readings 
    Input = sensors.getTempCByIndex(0);
    myPID.Compute();
  }
  if (Output > millis() - windowStartTime_relay) digitalWrite(RELAY_PIN, HIGH);
  else digitalWrite(RELAY_PIN, LOW);
} 
