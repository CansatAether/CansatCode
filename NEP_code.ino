//6-2-2022
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_BMP280.h>


#define boardId 5
bool sensorBMP = false;
bool sensorBME = true;
int interval = 500; //sends every 0,5sec

Adafruit_BME280 bme; // I2C
Adafruit_BMP280 bmp; // I2C


unsigned long delayTime;
bool firstStart = true;
float startPressure = 0;



uint8_t broadcastAddress1[] = {0x9C, 0x9C, 0x1F, 0xE3, 0xC7, 0xA8}; 

// Structure  to send data (must match the receiver structure)
typedef struct struct_message {
  int id; //board_id
  int  T;  //temp
  int  P;  //pressure
  int  A;  //alt
  int  H;  //hum
} struct_message;

// Create a struct_message called test to store variables to be sent
struct_message myData;




void setup() {
Serial.begin(115200);
  
// Set device as a Wi-Fi Station
WiFi.mode(WIFI_STA);
WiFi.disconnect();

// Init ESP-NOW
if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
    }

// Set ESP-NOW role
esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

// Once ESPNow is successfully init, we will register for Send CB to
// get the status of Trasnmitted packet
esp_now_register_send_cb(OnDataSent);

 
// Register peer(s)
esp_now_add_peer(broadcastAddress1, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

Wire.begin(2,0); // pin: ([SDA], [SCL])


if (sensorBME==true  && sensorBMP==false){Serial.println("BME280 sensor active" );}
if (sensorBMP==true  && sensorBME==false){Serial.println("BMP280 sensor active" );}
if (sensorBMP==true  && sensorBME==true ){Serial.println("ERROR, 2 sensors selected" );}
if (sensorBMP==false && sensorBME==false){Serial.println("ERROR, 0 sensors selected" );}




//for (uint8_t t = 4; t > 0; t--) {
//    Serial.println(t);
//    delay(900);
//    }

if (sensorBME==true){
bme.begin(0x76);
    
    // indoor navigation
    Serial.println("-- Indoor Navigation Scenario --");
    Serial.println("normal mode, 16x pressure / 2x temperature / 1x humidity oversampling,");
    Serial.println("0.5ms standby period, filter 16x");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,  // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );
    
    // suggested rate is 25Hz
    // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5) + (2 * H_ovs + 0.5)
    // T_ovs = 2
    // P_ovs = 16
    // H_ovs = 1
    // = 40ms (25Hz)
    // with standby time that should really be 24.16913... Hz
}

if (sensorBMP==true){
bmp.begin(0x76);
bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */                 
}

    
myData.id = boardId;
}




void loop() {
if (firstStart==true) {
    Serial.print("first start");
    myData.id = boardId;
    altCalibration();
    }
    
else {  
  if (millis() > (interval + delayTime)) {  
      sendReadings();
      delayTime = millis();
      }
    }   
}


void sendReadings() {
  
if (sensorBME==true) {  
myData.T = (bme.readTemperature() *100);
myData.P =  bme.readPressure()/10;
myData.A = (bme.readAltitude(startPressure)*10);
myData.H = (bme.readHumidity()*100);
}

if (sensorBMP==true) {
myData.T = (bmp.readTemperature() *100);
myData.P = bmp.readPressure()/10;
myData.A = (bmp.readAltitude(startPressure)*10);
myData.H = 999;
}

esp_now_send(0, (uint8_t *) &myData, sizeof(myData));    
printValues();

}

void printValues() {
    
    Serial.print("Board.Id = ");
    Serial.println(myData.id);

    Serial.print("Temperature = ");
    Serial.print(myData.T);
    Serial.println(" GrC");

    Serial.print("Pressure = ");
    Serial.print(myData.P);
    Serial.println(" Pa");

    Serial.print("Approx. Altitude = ");
    Serial.print(myData.A);
    Serial.println(" dm");

    Serial.print("Humidity = ");
    Serial.print(myData.H);
    Serial.println(" %");
    Serial.println();
}





void altCalibration(){

Serial.println("altCalibration");
  
float altRead = 0;
float averageAlt = 0;
int R= 0;

for (int i = 0; i<10; i++) { //take 10 airpressure samples
             if (sensorBME==true) {altRead = bme.readPressure();}
             if (sensorBMP==true) {altRead = bmp.readPressure();}

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

else{ errorSettingAlt(); 
    }
}


void errorSettingAlt(){
Serial.println("Last errorSettingAlt");
myData.id = (boardId*1000)+404;
sendReadings();
firstStart = true; //retry 
  }


void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}
