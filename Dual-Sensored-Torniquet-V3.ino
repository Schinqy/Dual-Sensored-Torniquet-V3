#include <WiFi.h>
#include <WiFiClient.h>
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WebSocketsServer.h>
#include "HX710B.h"


const int DOUT_Pin = 12;   //sensor data pin
const int SCK_Pin  = 14;   //sensor clock pin
HX710B pressure_sensor; 
long offset = -7244154;
float realPressure = 0; 

const char* ssid = "Tourniquet";
const char* password = "passc0d3";

AsyncWebServer server(80);
WebSocketsServer webSocket(81);

float temperature = 0;
float roomTemperature = 0;
boolean gotRT = false;
float pressure = 0;
float threshTemperature = 0;
float threshPressure = 0;
float previousThreshPressure = 0;
int pumpRelay = 16;
int solenoidRelay = 17;
int heaterRelay = 21;

#define ONE_WIRE_BUS 4 // GPIO pin where DS18B20 is connected
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    // Handle WebSocket events if needed
}

void setup() {
    Serial.begin(115200);
    pinMode(pumpRelay, OUTPUT);
    pinMode(solenoidRelay, OUTPUT);
    pinMode(heaterRelay, OUTPUT);
    digitalWrite(heaterRelay, HIGH);
  digitalWrite(pumpRelay, HIGH);
    digitalWrite(solenoidRelay, HIGH);
    WiFi.softAP(ssid, password);
    Serial.println("SoftAP created with SSID: " + String(ssid));

    if (!SPIFFS.begin(true)) {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    sensors.begin();
pressure_sensor.begin(DOUT_Pin, SCK_Pin, 128);
  pressure_sensor.set_offset(offset);
    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });

    server.on("/getTemperature", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(temperature).c_str());
    });
     server.on("/getRoomTemperature", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(roomTemperature).c_str());
    });

    server.on("/getPressure", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send_P(200, "text/plain", String(pressure).c_str());
    });

server.on("/adjustTemperature", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("value")) {
        threshTemperature = request->getParam("value")->value().toFloat();
        Serial.print("Adjusted Temperature: ");
        Serial.println(threshTemperature);
        request->send(200, "text/plain", "Temperature adjusted");
        webSocket.broadcastTXT(String('{"temperature": ' + String(temperature) + ', "pressure": ' + String(pressure) + '}').c_str());
    } else {
        request->send(400, "text/plain", "Invalid request");
    }
});

server.on("/adjustPressure", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("value")) {
        threshPressure = request->getParam("value")->value().toFloat();
        Serial.print("Adjusted Pressure: ");
        Serial.println(threshPressure);
        request->send(200, "text/plain", "Pressure adjusted");
        webSocket.broadcastTXT(String('{"temperature": ' + String(temperature) + ', "pressure": ' + String(pressure) + '}').c_str());
    } else {
        request->send(400, "text/plain", "Invalid request");
    }
});


    server.begin();
}

void loop() {
    sensors.requestTemperatures();
    //temperature = random(0, 40);

    float totalTemp = 0;
    if(gotRT == false){
      for(int i=0; i<=30; i++)
      {
      temperature = sensors.getTempCByIndex(0);
      totalTemp = totalTemp + temperature;
      delay(30);
      }
      roomTemperature = totalTemp / 30;
      gotRT = true;
    }
      temperature = sensors.getTempCByIndex(0);
      if (pressure_sensor.is_ready()) {
   
    realPressure = pressure_sensor.mmHg();
      }
    
  Serial.println("This is the target temp:" +  String(threshTemperature));
 Serial.println("This is the target pressure:" +  String(threshPressure));
 Serial.println("This is the current temp:" +  String(temperature));
  Serial.println("This is the room temp:" +  String(roomTemperature));
   Serial.println("This is the real pressure:" +  String(realPressure));
controlTemperature(threshTemperature);
controlPressure(threshPressure);
    // WebSocket handling
    webSocket.loop();

    delay(200); // Adjust delay according to your needs
}

void controlTemperature(float tempThresh)
{
  Serial.print("Current temperature: ");
  Serial.println(temperature);

  if (temperature != DEVICE_DISCONNECTED_C && temperature > tempThresh)
  {
    Serial.println("Heater OFF");
    digitalWrite(heaterRelay, HIGH);
  }
  else if (temperature != DEVICE_DISCONNECTED_C && temperature < tempThresh - 2)
  {
    Serial.println("Heater ON");
    digitalWrite(heaterRelay, LOW);
  }
}


//void controlPressure(float preshThresh)
//{
//  if(pressure > preshThresh - 10)
//  {
//     Serial.println("VALVE OFF");
//     //Switch OFF VALVE
//  }
//  else if (pressure < preshThresh - 10)
//  {
//    Serial.println("VALVE ON"); 
//    //Switch OFF VALVE 
//  }
//}
void controlPressure(float preshThresh)
{
    float pressureChange = preshThresh - previousThreshPressure;
    int pumpDelay = mmHgToDelay(pressureChange);
    

    if (pumpDelay > 0)
    {
        // Inflate
        digitalWrite(pumpRelay, LOW); // Switch ON PUMP
        digitalWrite(solenoidRelay, LOW); // Close Solenoid

        for (int i = 0; i <= pumpDelay; i++)
        {
            delay(1);

            pressure += (1.0 / 70.0) ;  // Use 1.0 to force floating-point division
            Serial.println(pressure);
        }
        
            
        digitalWrite(pumpRelay, HIGH); // Switch OFF PUMP
    }
    else
    {
        // Deflate
        digitalWrite(pumpRelay, HIGH); // Switch ON PUMP
        digitalWrite(solenoidRelay, HIGH); // Close Solenoid

        int pumpDelayDeflate = -1 * pumpDelay;
        for (int j = 0; j <= pumpDelayDeflate; j++)
        {
            delay(1);

          
            pressure -= 1.0 / 70.0;  // Use 1.0 to force floating-point division
            Serial.println(pressure);
        }

        digitalWrite(solenoidRelay, LOW); // Switch OFF PUMP
    }
    int plusOrMinus =  random(0,1);
//    if(plusOrMinus == 0)
//    {
//      int seed = random(0, 10);
//        pressure = pressure + (seed / 1000000.0);
//    }
//   else
//   {
//      int seed = random(0, 10);
//        pressure = pressure - (seed / 1000000.0);
//   }
   
    previousThreshPressure = preshThresh;
}




int mmHgToDelay( float pressure_mmHg )
{
  // say 1mmHg = 30ms
  //x = 1ms
  //x =1/30
  int calibrationFactor = 70;
  return pressure_mmHg*calibrationFactor;
}



//         Notes: 
// 197 to 250mmHg recommended pressure for forearm
// 35-40 temperature recommended for forearm
