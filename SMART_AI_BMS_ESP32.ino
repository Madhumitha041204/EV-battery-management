// SMART_AI_BMS_ESP32.ino
// Complete Integrated BMS Firmware
// Board: ESP32 Dev Module

#include <WiFi.h>
#include <ThingSpeak.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

const char* ssid = "YOUR_WIFI";
const char* password = "YOUR_PASSWORD";
unsigned long channelID = 0000000; // replace
const char* writeAPIKey = "WRITE_API_KEY";

WiFiClient client;
LiquidCrystal_I2C lcd(0x27,16,2);

#define CELL1_PIN 32
#define CELL2_PIN 33
#define CELL3_PIN 34
#define CELL4_PIN 35

#define BAL1 25
#define BAL2 26
#define BAL3 27
#define BAL4 14

#define RELAY_LOAD 12
#define RELAY_CHG  13

#define ADC_MAX 4095.0
#define ADC_REF 3.3
#define CELL_MAX 4.2
#define CELL_MIN 3.0
#define BAL_THRESHOLD 0.05

float cell[4];
float packVoltage;
float soc;

typedef enum {FAULT_NONE,FAULT_OVERVOLT,FAULT_UNDERVOLT,FAULT_CELL_IMBALANCE,FAULT_AI} FaultType;
FaultType faultCode;

typedef enum {STATE_IDLE,STATE_CHARGING,STATE_DISCHARGING,STATE_FAULT} BMS_State;
BMS_State bmsState;

float w1[1][3]={{0.6,-0.4,0.3}};
float w2[3]={0.5,0.6,-0.3};
float b1[3]={0.1,0.2,0.1};
float b2=0.05;

float relu(float x){return x>0?x:0;}
float sigmoid(float x){return 1.0/(1+exp(-x));}

float readCell(int pin){return analogRead(pin)/ADC_MAX*ADC_REF*2.0;}

float calcSOC(){float avg=(cell[0]+cell[1]+cell[2]+cell[3])/4;return constrain((avg-CELL_MIN)/(CELL_MAX-CELL_MIN)*100,0,100);} 

float predictAI(float v){float h[3];for(int i=0;i<3;i++)h[i]=relu(b1[i]+v*w1[0][i]);float o=b2;for(int i=0;i<3;i++)o+=h[i]*w2[i];return sigmoid(o);} 

void balance(){float m=cell[0];for(int i=1;i<4;i++)if(cell[i]>m)m=cell[i];digitalWrite(BAL1,cell[0]>m-BAL_THRESHOLD);digitalWrite(BAL2,cell[1]>m-BAL_THRESHOLD);digitalWrite(BAL3,cell[2]>m-BAL_THRESHOLD);digitalWrite(BAL4,cell[3]>m-BAL_THRESHOLD);} 

FaultType detectFault(float ai){float maxV=cell[0],minV=cell[0];for(int i=1;i<4;i++){maxV=max(maxV,cell[i]);minV=min(minV,cell[i]);}if(maxV>CELL_MAX)return FAULT_OVERVOLT;if(minV<CELL_MIN)return FAULT_UNDERVOLT;if((maxV-minV)>0.15)return FAULT_CELL_IMBALANCE;if(ai>0.6)return FAULT_AI;return FAULT_NONE;} 

void updateState(){if(faultCode!=FAULT_NONE){bmsState=STATE_FAULT;return;}if(packVoltage<15.5)bmsState=STATE_CHARGING;else if(packVoltage>16.6)bmsState=STATE_IDLE;else bmsState=STATE_DISCHARGING;} 

void controlRelays(){digitalWrite(RELAY_LOAD,LOW);digitalWrite(RELAY_CHG,LOW);if(bmsState==STATE_CHARGING)digitalWrite(RELAY_CHG,HIGH);if(bmsState==STATE_DISCHARGING)digitalWrite(RELAY_LOAD,HIGH);} 

void setup(){Serial.begin(9600);lcd.init();lcd.backlight();pinMode(RELAY_LOAD,OUTPUT);pinMode(RELAY_CHG,OUTPUT);pinMode(BAL1,OUTPUT);pinMode(BAL2,OUTPUT);pinMode(BAL3,OUTPUT);pinMode(BAL4,OUTPUT);WiFi.begin(ssid,password);while(WiFi.status()!=WL_CONNECTED)delay(500);ThingSpeak.begin(client);} 

void loop(){cell[0]=readCell(CELL1_PIN);cell[1]=readCell(CELL2_PIN);cell[2]=readCell(CELL3_PIN);cell[3]=readCell(CELL4_PIN);packVoltage=cell[0]+cell[1]+cell[2]+cell[3];soc=calcSOC();float ai=predictAI(packVoltage);faultCode=detectFault(ai);updateState();controlRelays();balance();ThingSpeak.setField(1,packVoltage);ThingSpeak.setField(2,soc);ThingSpeak.setField(3,faultCode);ThingSpeak.writeFields(channelID,writeAPIKey);lcd.clear();lcd.setCursor(0,0);lcd.print("SOC:");lcd.print(soc,0);lcd.print("%");lcd.setCursor(0,1);lcd.print("V:");lcd.print(packVoltage,1);delay(20000);}