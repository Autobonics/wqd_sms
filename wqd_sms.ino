//PH sensor
#define phPin 13
int phValue = 0;
float phReading = 0;

//Temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>
// // GPIO where the DS18B20 is connected to
const int oneWireBus = 32;          
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature tempSensor(&oneWire);
// // Temperature value
float temperature;

//EC sensor
// #include <Adafruit_ADS1015.h>
#include <DFRobot_ESP_EC.h>
//Pin
#define ecPin 34
DFRobot_ESP_EC ec;
// Adafruit_ADS1115 ads;
float ecVoltage, ecValue = 0;


//WiFi
#define wifiLedPin 5



void setup() {

  Serial.begin(115200);
  Serial2.begin(9600);

  //Temperature
  tempSensor.begin();
 
  //EC
  ec.begin();
 
  //WIFI
  pinMode(wifiLedPin, OUTPUT);
  digitalWrite(wifiLedPin, LOW);
  
  test_sim800_module();
}

void loop() {

  updateSerial();

    //PH sensor
    readPH();
  
    //Temperature
    readTemp();

    //EC
    readEC();

    printData();    

    if(ecValue>10){
      send_SMS("Alert! High EC value");
    }
    if(temperature>35){
      send_SMS("Alert! High temperature");
    }
    if(phReading>8){
      send_SMS("Alert! High pH");
    }
}

unsigned int printDataPrevMillis = 0;
void printData(){
  if (millis() - printDataPrevMillis > 2000 || printDataPrevMillis == 0)
  {
    printDataPrevMillis = millis();
    //PH
    Serial.print("PH: ");
    Serial.print(phValue);
    Serial.print(" | ");
    Serial.println(phReading);
    //Temperature
    Serial.print("Temperature:");
    Serial.print(temperature, 2);
    Serial.println("ºC");
    //Print EC
    Serial.print("EC:");
    Serial.println(ecValue);
  }
}

void readPH(){
  phValue = analogRead(phPin);
  float voltage = phValue*(3.3/4095.0);
  phReading =((3.3*voltage));
} 

void readTemp(){
  tempSensor.requestTemperatures();
  temperature = tempSensor.getTempCByIndex(0);  // read your temperature sensor to execute temperature compensation
}

void readEC(){
  ecVoltage = analogRead(ecPin);
  ecValue = ec.readEC(ecVoltage, temperature); // convert voltage to EC with temperature compensation
}


void test_sim800_module()
{
  Serial2.println("AT");
  updateSerial();
  Serial.println();
  Serial2.println("AT+CSQ");
  updateSerial();
  Serial2.println("AT+CCID");
  updateSerial();
  Serial2.println("AT+CREG?");
  updateSerial();
  Serial2.println("ATI");
  updateSerial();
  Serial2.println("AT+CBC");
  updateSerial();
}
void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial2.available())
  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}
void send_SMS(String message)
{
  Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  Serial2.println("AT+CMGS=\"+919074514820\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  Serial2.print(message); //text content
  updateSerial();
  Serial.println();
  Serial.println("Message Sent");
  Serial2.write((char)26);
}