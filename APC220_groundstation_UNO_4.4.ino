/*
Wirering:
APC220 > Arduino
gnd > gnd
vcc > 5V
en > Not connected
RXD > D3
TXD > D2
AUX > Not connected
SET > Not connected
*/


#include <SoftwareSerial.h>

SoftwareSerial sendToCCP(2, 3); 




char c;
String dataIn;
int8_t indexOfA, indexOfB, indexOfC, indexOfD, indexOfE;
String data1, data2, data3, data4, data5;

int incomming_Id; int id;
int incomming_T;  float T;
int incomming_P;  float P;
int incomming_A;  float A;
int incomming_H;  float H;


void setup() {
  Serial.begin(19200);
  Serial.println("Started listening");
  sendToCCP.begin(19200);
}

void loop() {

ListenToAPC220();
}






void ListenToAPC220() {
    while (sendToCCP.available() > 0) {
              c = sendToCCP.read();
                if ( c=='/n') {break;}
                 else {dataIn+=c;}
            }
            
    if(c=='\n') {
      ParseData();
      c=0;
      dataIn="";}

            
    while (Serial.available() > 0) {
           sendToCCP.write(Serial.read());
           yield();
           }
}



void ParseData(){
Serial.println("parsing begin");

String APC220str1, APC220str2, APC220str3, APC220str4, APC220str5 ;
int    APC220int1, APC220int2, APC220int3, APC220int4, APC220int5;

  indexOfA = dataIn.indexOf("A");
  indexOfB = dataIn.indexOf("B");
  indexOfC = dataIn.indexOf("C");
  indexOfD = dataIn.indexOf("D");
  indexOfE = dataIn.indexOf("E");
  
  
  APC220str1=dataIn.substring (0, indexOfA);
  APC220str2=dataIn.substring (indexOfA+1, indexOfB);
  APC220str3=dataIn.substring (indexOfB+1, indexOfC);
  APC220str4=dataIn.substring (indexOfC+1, indexOfD);
  APC220str5=dataIn.substring (indexOfD+1, indexOfE);
  
//concert from string to int
  incomming_Id = (APC220str1.toInt());
  incomming_T  = (APC220str2.toInt());
  incomming_P  = (APC220str3.toInt());
  incomming_A  = (APC220str4.toInt());
  incomming_H  = (APC220str5.toInt());

//convert to "float"/decimals
 id = incomming_Id;
  T = incomming_T*0.01;
  P = incomming_P*0.1;
  A = incomming_A*0.01;
  H = incomming_H*0.01;



if (id!=0) {
  
    Serial.println("received and corected data from ACP220");
    Serial.print("BoardId: ");
    Serial.println(id);
    
    Serial.print("Temperature: ");
    Serial.print(T);
    Serial.println(" *C");
    
    Serial.print("Pressure: ");
    Serial.println(P);
    Serial.print("Altitude: ");
    Serial.println(A);
    
    
      Serial.print("Humidity: ");
      Serial.println(H);
    
    
    Serial.println("");
    
    SafeValuesToLogfile();
}

//empty buffer
sendToCCP.flush();
}



void SafeValuesToLogfile(){

  // store values of id, T, P, A, H 
  // before next data transmission is receaved

}
  
