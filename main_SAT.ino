#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP32Servo.h>  // zoek op bibliotheek ESP32Servo


Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
// Possible PWM GPIO pins on the ESP32-S2: 0(used by on-board button),1-17,18(used by on-board LED),19-21,26,33-42
//int servoPin = 19;  // (oranje draad naar de servomotor)
int teller = 0;

int maxAltitude = 0;
int altitudeDifference;

unsigned long timer = 0;
bool right = false;
bool left  = false; 

bool firstStart = true;
bool firstPop = true;

float startPressure = 0;
float Altitude_main;

Adafruit_BME280 bme;

HardwareSerial sendToCCP(1);



// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int id;
    int T;
    int P;
    int A;
    int H;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    
  memcpy(&myData, incomingData, sizeof(myData));
  
  Serial.println("sendingSerialData to APC220");
   
  sendToCCP.print(myData.id); sendToCCP.print("A");
  sendToCCP.print(myData.T);  sendToCCP.print("B");
  sendToCCP.print(myData.P);  sendToCCP.print("C");
  sendToCCP.print(myData.A);  sendToCCP.print("D");
  sendToCCP.print(myData.H);  sendToCCP.print("E");
  sendToCCP.print("\n"); // end command of packet
  
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("BoardId: ");
  Serial.println(myData.id);
  Serial.print("Temperature: ");
  Serial.println(myData.T);
  Serial.print("Pressure: ");
  Serial.println(myData.P);
  Serial.print("Altitude: ");
  Serial.println(myData.A);
  Serial.print("Humidity: ");
  Serial.println(myData.H);
  Serial.println();  

 
}
 
void setup() {
  
  // Initialize  Monitor
  Serial.begin(19200);
  
  sendToCCP.begin(19200, SERIAL_8N1, 16,17); //(baud-rate, protocol, RX pin, TX pin) 
    
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  Wire.begin(18,23); // pin: ([SDA], [SCL])

  pinMode(19, OUTPUT); //sets pin 19 as output
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
      }
    
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  
  
  // Allow allocation of all timers used for the servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
//    myservo.setPeriodHertz(50);    // standard 50 hz servo
//    myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
    // using default min/max of 1000us and 2000us (eind begin stand servo?
    // different servos may require different min/max settings
    // for an accurate 0 to 180 sweep
  
  
  bme.begin(0x76);
      
      // indoor navigation
      
    // indoor navigation
      bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                      Adafruit_BME280::SAMPLING_X2,  // temperature
                      Adafruit_BME280::SAMPLING_X16, // pressure
                      Adafruit_BME280::SAMPLING_X1,  // humidity
                      Adafruit_BME280::FILTER_X16,
                      Adafruit_BME280::STANDBY_MS_0_5 );
}


 
void loop() {
  
   if (firstStart==true) {
    Serial.print("first start");
    digitalWrite(19, LOW);
//    myData.id = boardId;
    altCalibration();
    }
    
  else {  
     if (millis() > (timer+1000)) { 
        
        Serial.println("test");
               
        sendToCCP.print("test send to CCP220: = ");
        sendToCCP.println(teller);
        
        readReadings();
        
        altitudeDifference = maxAltitude - Altitude_main;
        Serial.print("max Altitude (dm): ");
        Serial.println(maxAltitude);
        Serial.print("altitudeDifference: ");
        Serial.println(altitudeDifference);
        Serial.println();  
        
        teller++;
        timer = millis();   
      }

      if (Altitude_main >1000 && (altitudeDifference) > 10) {
        
        digitalWrite(19, HIGH);
        
        
        if (firstPop == true) {
          Serial.println("POP");
          
        }
        firstPop = false;
      }
      
      if (Altitude_main >= maxAltitude) {
        maxAltitude = Altitude_main;
      }
    } 
  //print test elke  sec


}

void readReadings() {  
  myData.id = 1;
  myData.T = (bme.readTemperature() *100);
  myData.P =  bme.readPressure()/10;
  myData.A = (bme.readAltitude(startPressure)*10); //in dm
  myData.H = (bme.readHumidity()*100);

  Altitude_main = (bme.readAltitude(startPressure)*10); // in dm

////////////////////////////////////////////

  sendToCCP.print(myData.id); sendToCCP.print("A");
  sendToCCP.print(myData.T);  sendToCCP.print("B");
  sendToCCP.print(myData.P);  sendToCCP.print("C");
  sendToCCP.print(myData.A);  sendToCCP.print("D");
  sendToCCP.print(myData.H);  sendToCCP.print("E");
  sendToCCP.print("\n"); // end command of packet

  Serial.print("BoardId: ");
  Serial.println(myData.id);
  Serial.print("Temperature: ");
  Serial.println(myData.T);
  Serial.print("Pressure: ");
  Serial.println(myData.P);
  Serial.print("Altitude: ");
  Serial.println(myData.A);
  Serial.print("Humidity: ");
  Serial.println(myData.H);
  
//////////////////////////////////////////////////////////


}

void altCalibration(){

Serial.println("altCalibration");
  
float altRead = 0;
float averageAlt = 0;
int R= 0;

for (int i = 0; i<10; i++) { //take 10 airpressure samples
         
         altRead = bme.readPressure();

         Serial.print("raw alt read = ");
         Serial.print(altRead);
         Serial.println();
         delay(300);
 
if ((altRead < 105000) && (altRead > 90090)){ // check upper && lower reading
  
         averageAlt += altRead;
         R++;
         Serial.print("alt read # ");
         Serial.print(R);
         Serial.print(" =");
         Serial.print(altRead);
         Serial.println();        
     }  
  }

  
  if (R>6) {//at least 6 good readings
    startPressure = ((averageAlt/R)/100); //from pa to hpa
    Serial.print("startPressure =  ");
    Serial.print(startPressure);
    Serial.println();       
    firstStart = false; //run once
  }
}
